import sys
import os
import json
import base64
import subprocess
import csv
from datetime import datetime

# MediaPipe/OpenCV, PyVista/VTK ve Qt aynı süreçte bazı Windows makinelerde DLL çakışması yaşayabiliyor.
# Bu nedenle kamera ve MediaPipe ana GUI sürecine import edilmez; hand_tracker_worker.py ayrı process olarak çalışır.
os.environ.setdefault("GLOG_minloglevel", "2")
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import math
import time
import numpy as np

from scipy.optimize import minimize
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QSlider, QLabel, QPushButton,
                             QGroupBox, QLineEdit, QGridLayout, QFrame,
                             QDoubleSpinBox, QListWidget, QAbstractItemView,
                             QScrollArea, QDialog, QTextEdit, QSizePolicy, QComboBox,
                             QTabWidget)
from PyQt5.QtCore import Qt, QTimer, QThread, QProcess, pyqtSignal
from PyQt5.QtGui import QIcon, QImage, QPixmap
from pyvistaqt import QtInteractor
import pyvista as pv

try:
    import speech_recognition as sr
    SR_IMPORT_ERROR = None
except Exception as exc:
    sr = None
    SR_IMPORT_ERROR = exc

# VoiceWorker
class VoiceCommandWorker(QThread):
    """Mikrofonu GUI thread'inden ayırarak Türkçe sesli komut dinleyen worker."""
    command_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()
        if sr is None:
            self.recognizer = None
        else:
            self.recognizer = sr.Recognizer()
        self.is_running = True

    def run(self):
        if sr is None:
            detail = f" - {type(SR_IMPORT_ERROR).__name__}: {SR_IMPORT_ERROR}" if SR_IMPORT_ERROR else ""
            self.status_signal.emit("SES HATASI: speech_recognition import edilemedi" + detail, "#e74c3c")
            return
        try:
            with sr.Microphone() as source:
                self.status_signal.emit("SES: Mikrofon kalibre ediliyor...", "#f39c12")
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                self.status_signal.emit("SES KOMUTU AKTİF - Komut bekleniyor", "#2ecc71")

                while self.is_running:
                    try:
                        audio = self.recognizer.listen(source, timeout=2, phrase_time_limit=3)
                        text = self.recognizer.recognize_google(audio, language="tr-TR").lower().strip()
                        if text:
                            self.command_signal.emit(text)
                    except (sr.WaitTimeoutError, sr.UnknownValueError):
                        continue
                    except sr.RequestError:
                        self.status_signal.emit("SES HATASI: Google Speech servisine ulaşılamıyor", "#e74c3c")
                        time.sleep(1)
                    except Exception as exc:
                        self.status_signal.emit(f"SES HATASI: {type(exc).__name__}", "#e74c3c")
                        time.sleep(1)
        except OSError:
            self.status_signal.emit("SES HATASI: Mikrofon bulunamadı veya erişim izni yok", "#e74c3c")
        finally:
            self.status_signal.emit("SES KOMUTU PASİF", "#95a5a6")

    def stop(self):
        self.is_running = False
        if self.isRunning():
            self.wait(2500)



# Kamera / el takibi ana GUI içinde değil, ayrı process olarak hand_tracker_worker.py dosyasında çalışır.
# Bu dosyada bilerek cv2 veya mediapipe import edilmez.

# ==========================================================
# 1. KONFİGÜRASYON VE MATEMATİK
# ==========================================================

try:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    BASE_DIR = os.getcwd()

file_mapping = {
    "base":  ["ABB_IRB1600_145-0.stl"],
    "link1": ["ABB_IRB1600_145-1-1.stl", "ABB_IRB1600_145-1-2.stl", "ABB_IRB1600_145-1-3.stl"],
    "link2": ["ABB_IRB1600_145-2.stl"],
    "link3": ["ABB_IRB1600_145-3-1.stl"],
    "link4": ["ABB_IRB1600_145-4-1.stl", "ABB_IRB1600_145-4-2.stl", "ABB_IRB1600_145-4-3.stl"],
    "link5": ["ABB_IRB1600_145-5.stl"],
    "link6": ["ABB_IRB1600_145-6.stl"]
}

link_colors = {
    "base":  "#333333", "link1": "#ec6602", "link2": "#ec6602",
    "link3": "#ec6602", "link4": "#ec6602", "link5": "#4a4a4a", "link6": "#808080"
}

joints = [
    {"axis": [0, 0, 1], "point": [0.0,   0.0,    0.0]},
    {"axis": [0, 1, 0], "point": [150.0, 0.0,  481.5]},
    {"axis": [0, 1, 0], "point": [150.0, 0.0, 1181.5]},
    {"axis": [1, 0, 0], "point": [150.0, 0.0, 1181.5]},
    {"axis": [0, 1, 0], "point": [750.0, 0.0, 1181.5]},
    {"axis": [1, 0, 0], "point": [750.0, 0.0, 1181.5]}
]

# ABB IRB 1600-1.45m Resmi Eklem Limitleri (Derece)
slider_limits = [
    (-180, 180),  # Eksen 1
    (-90, 120),   # Eksen 2 (Asimetrik)
    (-245, 65),   # Eksen 3 (Asimetrik)
    (-200, 200),  # Eksen 4
    (-115, 115),  # Eksen 5
    (-400, 400)   # Eksen 6
]
tcp_local_h = np.array([850.0, 0.0, 1181.5, 1.0])

def axis_angle_transform(angle_deg, axis, point):
    angle = np.radians(angle_deg)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    point = np.asarray(point, dtype=float)
    ux, uy, uz = axis
    c, s = np.cos(angle), np.sin(angle)
    
    R = np.array([
        [c + ux**2*(1-c),    ux*uy*(1-c)-uz*s, ux*uz*(1-c)+uy*s],
        [uy*ux*(1-c)+uz*s,  c + uy**2*(1-c),   uy*uz*(1-c)-ux*s],
        [uz*ux*(1-c)-uy*s,  uz*uy*(1-c)+ux*s,  c + uz**2*(1-c)]
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = point - R @ point
    return T

def get_fk_matrix(angles):
    T_matrix = np.eye(4)
    for i in range(6):
        Ti = axis_angle_transform(angles[i], joints[i]["axis"], joints[i]["point"])
        T_matrix = T_matrix @ Ti
    T_tcp = np.eye(4)
    T_tcp[:3, 3] = tcp_local_h[:3]
    return T_matrix @ T_tcp

def get_fk_position(angles):
    T_matrix = get_fk_matrix(angles)
    return T_matrix[:3, 3]

def get_rpy_from_matrix(R):
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)

# ==========================================================
# RAPID KODU DIŞA AKTARMA PENCERESİ
# ==========================================================
class CodeExportDialog(QDialog):
    def __init__(self, waypoints, speed, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ABB RAPID Post-Processor (Kod Üretici)")
        self.resize(600, 500)
        self.setStyleSheet("background-color: #1a1a1a; color: #e0e0e0; font-family: 'Consolas', monospace;")
        
        layout = QVBoxLayout(self)
        label = QLabel("Gerçek bir ABB robotunda çalıştırılabilir RAPID kodu:")
        label.setStyleSheet("font-weight: bold; color: #ec6602; font-family: 'Segoe UI'; font-size: 14px;")
        layout.addWidget(label)
        
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setStyleSheet("background-color: #0d0d0d; color: #00ffcc; font-size: 13px; padding: 10px; border: 1px solid #333;")
        layout.addWidget(self.text_edit)

        # --- OTOMASYON İÇİN GÜVENLİ BAŞLANGIÇ DEĞERLERİ ---
        self.box_type = None          # Hata vermesini önlemek için hafızada yer açıyoruz
        self.automation_active = False
        
        self.generate_code(waypoints, speed)

    def generate_code(self, waypoints, speed):
        speed_val = int(100 * speed)
        code = f"! =============================================\n"
        code += f"! RoboSim - Endüstriyel Post-Processor\n"
        code += f"! Robot Modeli: ABB IRB1600\n"
        code += f"! Otomatik Üretilen Yörünge Görevi\n"
        code += f"! =============================================\n\n"
        
        code += "MODULE MainModule\n\n"
        code += "    PROC main()\n"
        code += "        ! Yapilandirma uyarilarini kapat\n"
        code += "        ConfJ \\Off;\n"
        code += "        ConfL \\Off;\n\n"
        
        code += f"        ! Sistem Hiz Ayari (Override %{speed_val})\n"
        code += f"        VelSet {speed_val}, 1000;\n\n"
        
        if not waypoints:
            code += "        ! UYARI: Kaydedilmis nokta bulunamadi.\n"
        else:
            code += "        ! --- YORUNGE NOKTALARI BASLANGICI ---\n"
            for wp in waypoints:
                j = wp["angles"]
                code += f"        ! Hedef: {wp['name']} (X:{wp['xyz'][0]:.1f}, Y:{wp['xyz'][1]:.1f}, Z:{wp['xyz'][2]:.1f})\n"
                code += f"        MoveAbsJ [[{j[0]:.2f}, {j[1]:.2f}, {j[2]:.2f}, {j[3]:.2f}, {j[4]:.2f}, {j[5]:.2f}], "
                code += "[9E9, 9E9, 9E9, 9E9, 9E9, 9E9]], v1000, fine, tool0;\n"
            
            code += "        ! --- YORUNGE NOKTALARI SONU ---\n"
            
        code += "\n    ENDPROC\n"
        code += "ENDMODULE\n"
        
        self.text_edit.setText(code)

# ==========================================================
# 2. ARAYÜZ VE SİMÜLATÖR SINIFI
# ==========================================================
class RoboSimUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboSim")
        self.resize(1440, 900)
        
        icon_path = os.path.join(BASE_DIR, 'robot_icon.png')
        self.setWindowIcon(QIcon(icon_path))

        self.current_angles = [0.0] * 6
        self.link_actors = {}
        self.sliders = []
        self.spinboxes = [] 
        
        self.trail_points = []
        self.is_updating_ui = False
        self.speed_multiplier = 1.0 

        # --- SESLİ KOMUT BAŞLANGIÇ DEĞERLERİ ---
        self.voice_worker = None
        self.voice_active = False
        self.voice_jog_step = 50.0  # mm / komut

        # --- KAMERA / EL TAKİBİ BAŞLANGIÇ DEĞERLERİ ---
        self.hand_worker = None
        self.hand_process = None
        self.hand_stdout_buffer = b""
        self.hand_tracking_active = False
        self.hand_anchor = None
        self.robot_anchor = None
        self.hand_smoothed_offset = np.zeros(3, dtype=float)
        self.last_hand_solve_time = 0.0
        self.hand_update_period = 0.08  # saniye, IK yükünü sınırlamak için yaklaşık 12.5 Hz
        self.hand_y_scale = 1200.0      # sağ/sol kamera hareketi -> robot Y mm
        self.hand_z_scale = 1000.0      # yukarı/aşağı kamera hareketi -> robot Z mm
        self.hand_depth_scale = 1200.0  # yakın/uzak el boyutu değişimi -> robot X mm
        self.hand_deadzone = np.array([20.0, 20.0, 20.0], dtype=float)
        self.hand_smoothing_alpha = 0.25
        self.hand_frame_counter = 0
        self.hand_detect_counter = 0
        self.hand_last_visible_time = 0.0
        self.hand_workspace_limits = {
            "x": (250.0, 1250.0),
            "y": (-700.0, 700.0),
            "z": (250.0, 1500.0),
        }
        self._closing = False

        # --- OTOMASYON VE FİZİK GÜVENLİK BAŞLANGIÇ DEĞERLERİ ---
        self.box_type = None          # AttributeError hatasını engelleyen can simidi
        self.automation_active = False
        self.is_holding_box = False
        self.auto_state = "IDLE"
        self.dashboard_total_count = 0
        self.dashboard_small_count = 0
        self.dashboard_large_count = 0
        self.dashboard_good_count = 0
        self.dashboard_defect_count = 0
        self.dashboard_cycle_start_time = None
        self.dashboard_last_cycle_time = 0.0
        self.dashboard_cycle_times = []
        self.dashboard_last_event = "Sistem hazır"
        self.box_quality = None
        self.box_route = None
        self.factory_speed_factor = 1.55
        self.reject_x = -800.0
        self.reject_y = 0.0
        self.reject_table_z = 315.0

        self.anim_timer = QTimer()
        self.anim_timer.timeout.connect(self.anim_step)
        self.anim_start_angles = [0.0] * 6
        self.anim_target_angles = [0.0] * 6
        self.anim_steps = 1
        self.anim_current_step = 0
        
        # Canlı Drag (Fare Takibi)
        self.drag_target = None
        self.drag_timer = QTimer()
        self.drag_timer.timeout.connect(self.process_live_drag)
        self.drag_timer.start(30) 

        self.dashboard_timer = QTimer()
        self.dashboard_timer.timeout.connect(self.update_smartcell_dashboard)
        self.dashboard_timer.timeout.connect(self.update_plc_io_panel)
        self.dashboard_timer.start(500)
        
        self.waypoints = [] 
        self.is_playing_sequence = False
        self.sequence_index = 0
        
        self.workspace_actor = None
        self.workspace_mesh = pv.Sphere(radius=1450, center=(150, 0, 481.5), phi_resolution=40, theta_resolution=40)

        # --- UÇ TAKIMI / PROSES GÖRSELLEŞTİRME ---
        self.tool_mode = "vacuum"
        self.tool_actors = {}
        self.laser_cut_active = False
        self.laser_cut_points = []
        self.laser_cut_actor_name = "laser_cut_path"
        
        self.init_ui()
        self.init_3d_scene()
        self.apply_theme()

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.cleanup_resources)
        
        initial_pos = get_fk_position(self.current_angles)
        self.update_robot()
        
        self.target_widget = self.plotter.add_sphere_widget(
            callback=self.on_mouse_drag,
            center=initial_pos,
            radius=40,
            color="#ffcc00",       
            style="wireframe"      
        )

    def init_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(5, 5, 5, 5)

        self.left_scroll_area = QScrollArea()
        self.left_scroll_area.setFixedWidth(280)
        self.left_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.left_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.left_scroll_area.setWidgetResizable(False)
        self.left_scroll_area.setStyleSheet("QScrollArea { border: none; background-color: #171717; }")

        self.left_panel = QFrame()
        self.left_panel.setFixedWidth(262)
        self.left_panel_layout = QVBoxLayout(self.left_panel)
        self.left_panel_layout.setContentsMargins(8, 8, 8, 8)
        self.left_panel_layout.setSpacing(8)
        self.left_panel_layout.setAlignment(Qt.AlignTop)
        self.left_panel.setMinimumHeight(980)
        self.left_panel.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self.left_scroll_area.setWidget(self.left_panel)
        self.main_layout.addWidget(self.left_scroll_area)

        self.plotter = QtInteractor(self.central_widget)
        self.main_layout.addWidget(self.plotter.interactor, stretch=5)

        # MODERN TCP HUD 
        self.tcp_overlay = QLabel(self.plotter.interactor)
        self.tcp_overlay.setAttribute(Qt.WA_TransparentForMouseEvents) 
        self.tcp_overlay.setStyleSheet("""
            background-color: rgba(20, 20, 20, 210); 
            border: 1px solid #444; border-radius: 8px; 
            padding: 15px; font-family: 'Consolas'; font-size: 13px; font-weight: bold;
        """)
        self.tcp_overlay.move(20, 20)

        # DÖNÜŞÜM MATRİSİ HUD
        self.matrix_overlay = QLabel(self.plotter.interactor)
        self.matrix_overlay.setAttribute(Qt.WA_TransparentForMouseEvents) 
        self.matrix_overlay.setStyleSheet("""
            background-color: rgba(20, 20, 20, 180); 
            color: #95a5a6; border: 1px dashed #555; border-radius: 6px; 
            padding: 10px; font-family: 'Consolas'; font-size: 11px;
        """)
        
        self.scroll_area = QScrollArea()
        self.scroll_area.setFixedWidth(300)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll_area.setWidgetResizable(False)
        self.scroll_area.setStyleSheet("QScrollArea { border: none; background-color: #1a1a1a; }")

        # PANEL BURADA OLUŞMALI
        self.panel = QFrame()
        self.panel.setFixedWidth(282)
        self.panel_layout = QVBoxLayout(self.panel)
        self.panel_layout.setContentsMargins(8, 8, 8, 8)
        self.panel_layout.setSpacing(8)
        self.panel_layout.setAlignment(Qt.AlignTop)

        self.scroll_area.setWidget(self.panel)

        # AYARLAR EN SON
        self.panel.setMinimumHeight(980)
        self.panel.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

        self.main_layout.addWidget(self.scroll_area)

        self.process_tabs = QTabWidget()
        self.process_tabs.setDocumentMode(True)
        self.process_tabs.setTabPosition(QTabWidget.North)
        self.panel_layout.addWidget(self.process_tabs)

        self.io_tab = QWidget()
        self.io_tab_layout = QVBoxLayout(self.io_tab)
        self.io_tab_layout.setContentsMargins(4, 6, 4, 4)
        self.io_tab_layout.setSpacing(8)
        self.io_tab_layout.setAlignment(Qt.AlignTop)

        self.process_tab = QWidget()
        self.process_tab_layout = QVBoxLayout(self.process_tab)
        self.process_tab_layout.setContentsMargins(4, 6, 4, 4)
        self.process_tab_layout.setSpacing(8)
        self.process_tab_layout.setAlignment(Qt.AlignTop)

        self.program_tab = QWidget()
        self.program_tab_layout = QVBoxLayout(self.program_tab)
        self.program_tab_layout.setContentsMargins(4, 6, 4, 4)
        self.program_tab_layout.setSpacing(8)
        self.program_tab_layout.setAlignment(Qt.AlignTop)

        self.process_tabs.addTab(self.io_tab, "IO")
        self.process_tabs.addTab(self.process_tab, "Tool")
        self.process_tabs.addTab(self.program_tab, "Program")

        # 1. DURUM ÇUBUĞU EN ÜSTTE
        self.status_label = QLabel("SİSTEM HAZIR - OK")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setMaximumWidth(250)
        self.status_label.setStyleSheet("color: #2ecc71; font-weight: bold; padding: 12px; background: #222; border-radius: 4px; border: 1px solid #2ecc71; font-size: 13px;")
        self.left_panel_layout.addWidget(self.status_label)

        # --- SESLİ KOMUT PANELİ ---
        grp_voice = QGroupBox("🎙️ Sesli Komut (Hands-Free HMI)")
        lay_voice = QVBoxLayout()

        self.voice_status_label = QLabel("Pasif - Dinlemeyi başlatabilirsiniz")
        self.voice_status_label.setAlignment(Qt.AlignCenter)
        self.voice_status_label.setStyleSheet("color: #95a5a6; padding: 6px; background: #222; border-radius: 4px;")
        lay_voice.addWidget(self.voice_status_label)

        voice_btn_row = QHBoxLayout()
        self.btn_voice_start = QPushButton("🎙️ Dinlemeyi Başlat")
        self.btn_voice_start.setStyleSheet("background-color: #16a085; color: white; font-weight: bold;")
        self.btn_voice_start.clicked.connect(self.start_voice_control)
        voice_btn_row.addWidget(self.btn_voice_start)

        self.btn_voice_stop = QPushButton("⏹️ Durdur")
        self.btn_voice_stop.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
        self.btn_voice_stop.clicked.connect(self.stop_voice_control)
        self.btn_voice_stop.setEnabled(False)
        voice_btn_row.addWidget(self.btn_voice_stop)
        lay_voice.addLayout(voice_btn_row)

        voice_hint = QLabel("Komutlar: ileri, geri, sağ, sol, yukarı, aşağı, sıfırla, oynat")
        voice_hint.setWordWrap(True)
        voice_hint.setMaximumWidth(250)
        voice_hint.setStyleSheet("color: #aaa; font-size: 10px; padding: 4px;")
        lay_voice.addWidget(voice_hint)

        grp_voice.setLayout(lay_voice)
        self.io_tab_layout.addWidget(grp_voice)

        # --- KAMERA İLE EL TAKİBİ PANELİ ---
        grp_hand = QGroupBox("🖐️ Kamera ile El Takibi")
        grp_hand.setMaximumWidth(266)
        lay_hand = QVBoxLayout()
        lay_hand.setContentsMargins(8, 8, 8, 8)

        camera_source_row = QHBoxLayout()
        camera_source_row.addWidget(QLabel("Kamera:"))
        self.camera_index_combo = QComboBox()
        for idx in range(6):
            self.camera_index_combo.addItem(f"Kamera {idx}", idx)
        self.camera_index_combo.setCurrentIndex(0)
        self.camera_index_combo.setFixedWidth(170)
        self.camera_index_combo.setToolTip("Kamera açılmazsa farklı index deneyin. Worker DSHOW öncelikli açar.")
        camera_source_row.addWidget(self.camera_index_combo)
        lay_hand.addLayout(camera_source_row)

        self.camera_preview = QLabel("Kamera kapalı")
        self.camera_preview.setAlignment(Qt.AlignCenter)
        self.camera_preview.setFixedSize(250, 145)
        self.camera_preview.setStyleSheet("background-color: #0d0d0d; color: #777; border: 1px solid #333; border-radius: 4px;")
        lay_hand.addWidget(self.camera_preview)

        self.hand_status_label = QLabel("Pasif - Kamera takibini başlatabilirsiniz")
        self.hand_status_label.setAlignment(Qt.AlignCenter)
        self.hand_status_label.setWordWrap(True)
        self.hand_status_label.setMaximumWidth(250)
        self.hand_status_label.setStyleSheet("color: #95a5a6; padding: 6px; background: #222; border-radius: 4px;")
        lay_hand.addWidget(self.hand_status_label)

        hand_btn_row = QHBoxLayout()
        self.btn_hand_start = QPushButton("Başlat")
        self.btn_hand_start.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold;")
        self.btn_hand_start.clicked.connect(self.start_hand_tracking)
        hand_btn_row.addWidget(self.btn_hand_start)

        self.btn_hand_stop = QPushButton("Durdur")
        self.btn_hand_stop.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
        self.btn_hand_stop.clicked.connect(self.stop_hand_tracking)
        self.btn_hand_stop.setEnabled(False)
        hand_btn_row.addWidget(self.btn_hand_stop)
        lay_hand.addLayout(hand_btn_row)

        self.btn_hand_recalibrate = QPushButton("🎯 Eli Yeniden Kalibre Et")
        self.btn_hand_recalibrate.clicked.connect(self.recalibrate_hand_tracking)
        self.btn_hand_recalibrate.setEnabled(False)
        lay_hand.addWidget(self.btn_hand_recalibrate)

        hand_hint = QLabel("Kullanım: Elinizi kameranın ortasında tutun. Sağ-sol → Y, yukarı-aşağı → Z, yakın-uzak → X hareketidir.")
        hand_hint.setWordWrap(True)
        hand_hint.setMaximumWidth(250)
        hand_hint.setStyleSheet("color: #aaa; font-size: 10px; padding: 4px;")
        lay_hand.addWidget(hand_hint)

        grp_hand.setLayout(lay_hand)
        self.io_tab_layout.addWidget(grp_hand)

        # --- UÇ TAKIMI / PROSES PANELİ ---
        grp_tool = QGroupBox("🔧 Uç Takımı ve Proses")
        grp_tool.setMaximumWidth(266)
        lay_tool = QVBoxLayout()
        lay_tool.setContentsMargins(8, 8, 8, 8)

        tool_row = QHBoxLayout()
        tool_row.addWidget(QLabel("Tool:"))
        self.tool_mode_combo = QComboBox()
        self.tool_mode_combo.addItem("Vakum tutucu", "vacuum")
        self.tool_mode_combo.addItem("Lazer kesici", "laser")
        self.tool_mode_combo.addItem("Mil / spindle", "spindle")
        self.tool_mode_combo.currentIndexChanged.connect(self.on_tool_mode_changed)
        self.tool_mode_combo.setFixedWidth(170)
        tool_row.addWidget(self.tool_mode_combo)
        lay_tool.addLayout(tool_row)

        tool_btn_row = QHBoxLayout()
        self.btn_laser_cut = QPushButton("Lazer Kesim: OFF")
        self.btn_laser_cut.clicked.connect(self.toggle_laser_cut)
        self.btn_laser_cut.setStyleSheet("background-color: #7f1d1d; color: white; font-weight: bold;")
        tool_btn_row.addWidget(self.btn_laser_cut)

        self.btn_clear_laser_cut = QPushButton("İzi Temizle")
        self.btn_clear_laser_cut.clicked.connect(self.clear_laser_cut)
        tool_btn_row.addWidget(self.btn_clear_laser_cut)
        lay_tool.addLayout(tool_btn_row)

        self.tool_status_label = QLabel("Aktif tool: Vakum tutucu")
        self.tool_status_label.setWordWrap(True)
        self.tool_status_label.setMaximumWidth(250)
        self.tool_status_label.setStyleSheet("color: #aaa; font-size: 10px; padding: 4px;")
        lay_tool.addWidget(self.tool_status_label)

        grp_tool.setLayout(lay_tool)
        self.process_tab_layout.addWidget(grp_tool)

        # --- ÇALIŞMA ALANI GÖRSELLEŞTİRME BUTONU ---
        self.btn_workspace = QPushButton("🌐 Çalışma Alanını Göster")
        self.btn_workspace.setStyleSheet("background-color: #34495e; color: white; font-weight: bold; padding: 8px;")
        self.btn_workspace.clicked.connect(self.toggle_workspace)
        self.left_panel_layout.addWidget(self.btn_workspace)

        self.btn_clear_trail = QPushButton("🗑️ Çizilen İzi Temizle")
        self.btn_clear_trail.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; padding: 8px; font-size: 12px; border-radius: 4px;")
        self.btn_clear_trail.clicked.connect(self.clear_trail)
        self.left_panel_layout.addWidget(self.btn_clear_trail)

        # 2. HIZLI GÜVENLİ KAMERA REJİSİ (Çökmeyen Güvenli Sürüm)
        grp_cam = QGroupBox("🎥 Kamera Görünümleri")
        lay_cam = QHBoxLayout()
        
        btn_front = QPushButton("Ön")
        btn_front.clicked.connect(lambda: self.set_cam([(3000, 0, 600), (0, 0, 600), (0, 0, 1)]))
        
        btn_back = QPushButton("Arka")
        btn_back.clicked.connect(lambda: self.set_cam([(-3000, 0, 600), (0, 0, 600), (0, 0, 1)]))
        
        btn_right = QPushButton("Sağ")
        btn_right.clicked.connect(lambda: self.set_cam([(0, 3000, 600), (0, 0, 600), (0, 0, 1)]))
        
        btn_left = QPushButton("Sol")
        btn_left.clicked.connect(lambda: self.set_cam([(0, -3000, 600), (0, 0, 600), (0, 0, 1)]))
        
        btn_reset_cam = QPushButton("🔄 Başlangıç")
        btn_reset_cam.setStyleSheet("background-color: #f39c12; color: black; font-weight: bold;")
        btn_reset_cam.clicked.connect(self.reset_camera)

        lay_cam.addWidget(btn_front)
        lay_cam.addWidget(btn_back)
        lay_cam.addWidget(btn_right)
        lay_cam.addWidget(btn_left)
        lay_cam.addWidget(btn_reset_cam)
        
        grp_cam.setLayout(lay_cam)
        self.left_panel_layout.addWidget(grp_cam)

        # 3. DÜNYA KOORDİNATI SÜRÜŞÜ (WORLD JOGGING)
        grp_jog = QGroupBox("Öncelikli TCP Jog")
        lay_jog = QGridLayout()
        
        btn_px = QPushButton("+X Sür"); btn_px.clicked.connect(lambda: self.jog_world(10, 0, 0))
        btn_nx = QPushButton("-X Sür"); btn_nx.clicked.connect(lambda: self.jog_world(-10, 0, 0))
        btn_py = QPushButton("+Y Sür"); btn_py.clicked.connect(lambda: self.jog_world(0, 10, 0))
        btn_ny = QPushButton("-Y Sür"); btn_ny.clicked.connect(lambda: self.jog_world(0, -10, 0))
        btn_pz = QPushButton("+Z Sür"); btn_pz.clicked.connect(lambda: self.jog_world(0, 0, 10))
        btn_nz = QPushButton("-Z Sür"); btn_nz.clicked.connect(lambda: self.jog_world(0, 0, -10))
        
        for btn in [btn_px, btn_nx, btn_py, btn_ny, btn_pz, btn_nz]:
            btn.setAutoRepeat(True)         
            btn.setAutoRepeatDelay(100)      
            btn.setAutoRepeatInterval(15)    

        btn_px.setStyleSheet("background-color: #c0392b; color: white;")
        btn_nx.setStyleSheet("background-color: #e74c3c; color: white;")
        btn_py.setStyleSheet("background-color: #27ae60; color: white;")
        btn_ny.setStyleSheet("background-color: #2ecc71; color: white;")
        btn_pz.setStyleSheet("background-color: #2980b9; color: white;")
        btn_nz.setStyleSheet("background-color: #3498db; color: white;")
        
        lay_jog.addWidget(btn_px, 0, 0); lay_jog.addWidget(btn_nx, 1, 0)
        lay_jog.addWidget(btn_py, 0, 1); lay_jog.addWidget(btn_ny, 1, 1)
        lay_jog.addWidget(btn_pz, 0, 2); lay_jog.addWidget(btn_nz, 1, 2)
        grp_jog.setLayout(lay_jog)
        self.left_panel_layout.addWidget(grp_jog)

        # 4. Eklem Kontrol Grubu
        grp_joints = QGroupBox("Manuel Eksen Kontrolü")
        lay_joints = QGridLayout()
        for i in range(6):
            lbl = QLabel(f"E{i+1}")
            lbl.setFixedWidth(20)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(slider_limits[i][0])
            slider.setMaximum(slider_limits[i][1])
            slider.setValue(0)
            slider.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx, val))
            self.sliders.append(slider)
            
            spinbox = QDoubleSpinBox()
            spinbox.setRange(slider_limits[i][0], slider_limits[i][1])
            spinbox.setDecimals(1)
            spinbox.setSingleStep(1.0)
            spinbox.setSuffix("°")
            spinbox.setFixedWidth(58)
            spinbox.setAlignment(Qt.AlignRight)
            spinbox.valueChanged.connect(lambda val, idx=i: self.on_spinbox_change(idx, val))
            self.spinboxes.append(spinbox)

            lay_joints.addWidget(lbl, i, 0)
            lay_joints.addWidget(slider, i, 1)
            lay_joints.addWidget(spinbox, i, 2)
            
        btn_reset = QPushButton("Sıfır Pozisyonuna Dön")
        btn_reset.clicked.connect(self.reset_joints)
        lay_joints.addWidget(btn_reset, 6, 0, 1, 3)
        grp_joints.setLayout(lay_joints)
        self.left_panel_layout.addWidget(grp_joints)

        # 5. Ters Kinematik Hedef Paneli
        grp_ik = QGroupBox("TCP Hedefi (IK)")
        lay_ik = QGridLayout()
        self.ik_inputs = {}
        for i, axis in enumerate(["X", "Y", "Z"]):
            lay_ik.addWidget(QLabel(axis), i, 0)
            inp = QLineEdit("0.0")
            inp.setAlignment(Qt.AlignCenter)
            self.ik_inputs[axis] = inp
            lay_ik.addWidget(inp, i, 1)
            lay_ik.addWidget(QLabel("mm"), i, 2)

        btn_go = QPushButton("HEDEFE GİT")
        btn_go.setObjectName("actionButton")
        btn_go.clicked.connect(self.run_ik_from_ui)
        lay_ik.addWidget(btn_go, 3, 0, 1, 3)
        grp_ik.setLayout(lay_ik)
        self.left_panel_layout.addWidget(grp_ik)

        # 6. CANLI ÜRETİM DASHBOARD'U
        grp_dashboard = QGroupBox("📊 SmartCell Dashboard")
        grp_dashboard.setMaximumWidth(266)
        lay_dashboard = QGridLayout()
        lay_dashboard.setContentsMargins(8, 8, 8, 8)
        lay_dashboard.setHorizontalSpacing(8)
        lay_dashboard.setVerticalSpacing(4)
        self.dashboard_labels = {}
        dashboard_rows = [
            ("robot", "Robot Durumu"),
            ("state", "Aktif State"),
            ("vacuum", "Vakum Durumu"),
            ("box_type", "Kutu Tipi"),
            ("total", "Toplam Ürün"),
            ("small", "Küçük Ürün"),
            ("large", "Büyük Ürün"),
            ("good", "Sağlam Ürün"),
            ("defect", "Hatalı Ürün"),
            ("quality", "Kalite Oranı"),
            ("last_cycle", "Son Çevrim"),
            ("avg_cycle", "Ort. Çevrim"),
            ("hourly", "Saatlik Üretim"),
            ("event", "Son Olay"),
        ]
        for row, (key, title) in enumerate(dashboard_rows):
            title_lbl = QLabel(title)
            title_lbl.setStyleSheet("color:#9aa4ad; font-size:10px;")
            value_lbl = QLabel("--")
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_lbl.setWordWrap(True)
            value_lbl.setStyleSheet("color:#f0f3f5; font-family:Consolas; font-size:10px; font-weight:bold;")
            self.dashboard_labels[key] = value_lbl
            lay_dashboard.addWidget(title_lbl, row, 0)
            lay_dashboard.addWidget(value_lbl, row, 1)

        self.btn_export_report = QPushButton("📄 Üretim Raporu Dışa Aktar")
        self.btn_export_report.clicked.connect(self.export_production_report)
        self.btn_export_report.setStyleSheet("background-color: #2563eb; color: white; font-weight: bold;")
        lay_dashboard.addWidget(self.btn_export_report, len(dashboard_rows), 0, 1, 2)

        grp_dashboard.setLayout(lay_dashboard)
        self.program_tab_layout.addWidget(grp_dashboard)

        # 7. SANAL PLC I/O PANELİ
        grp_plc = QGroupBox("🧠 Sanal PLC I/O")
        grp_plc.setMaximumWidth(266)
        lay_plc = QGridLayout()
        lay_plc.setContentsMargins(8, 8, 8, 8)
        lay_plc.setHorizontalSpacing(6)
        lay_plc.setVerticalSpacing(3)
        self.plc_io_labels = {}
        plc_rows = [
            ("I0.0", "Sensor_IN"),
            ("I0.1", "Box_Present"),
            ("I0.2", "Quality_OK"),
            ("I0.3", "Quality_NG"),
            ("I0.4", "EStop"),
            ("Q0.0", "Conveyor_IN"),
            ("Q0.1", "Conveyor_OUT"),
            ("Q0.2", "Vacuum"),
            ("Q0.3", "Reject_Gate"),
            ("Q0.4", "Alarm_Lamp"),
        ]
        for row, (addr, name) in enumerate(plc_rows):
            addr_lbl = QLabel(addr)
            addr_lbl.setStyleSheet("color:#9aa4ad; font-family:Consolas; font-size:10px;")
            name_lbl = QLabel(name)
            name_lbl.setStyleSheet("color:#c9d1d9; font-size:10px;")
            value_lbl = QLabel("0")
            value_lbl.setAlignment(Qt.AlignCenter)
            value_lbl.setFixedWidth(24)
            self.plc_io_labels[addr] = value_lbl
            lay_plc.addWidget(addr_lbl, row, 0)
            lay_plc.addWidget(name_lbl, row, 1)
            lay_plc.addWidget(value_lbl, row, 2)
        grp_plc.setLayout(lay_plc)
        self.program_tab_layout.addWidget(grp_plc)

        # 8. YÖRÜNGE PROGRAMLAMA & RAPID KODU
        grp_traj = QGroupBox("Üretim Programı ve Fabrika")
        lay_traj = QVBoxLayout()
        
        self.list_waypoints = QListWidget()
        self.list_waypoints.setFixedHeight(100)
        self.list_waypoints.setSelectionMode(QAbstractItemView.SingleSelection)
        lay_traj.addWidget(self.list_waypoints)
        
        btn_lay1 = QHBoxLayout()
        btn_save_wp = QPushButton("+ Nokta Ekle")
        btn_save_wp.setObjectName("actionButton")
        btn_save_wp.clicked.connect(self.save_waypoint)
        btn_del_wp = QPushButton("- Temizle")
        btn_del_wp.clicked.connect(self.clear_waypoints)
        btn_lay1.addWidget(btn_save_wp)
        btn_lay1.addWidget(btn_del_wp)
        lay_traj.addLayout(btn_lay1)

        btn_lay2 = QHBoxLayout()
        btn_play_seq = QPushButton("▶ OYNAT")
        btn_play_seq.setStyleSheet("background-color: #2ecc71; color: black; font-weight: bold;")
        btn_play_seq.clicked.connect(self.play_sequence)
        
        btn_export = QPushButton("ABB RAPID KODU ÜRET")
        btn_export.setStyleSheet("background-color: #9b59b6; color: white; font-weight: bold;")
        btn_export.clicked.connect(self.show_export_dialog)
        
        btn_lay2.addWidget(btn_play_seq)
        btn_export.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        btn_lay2.addWidget(btn_export)
        lay_traj.addLayout(btn_lay2)
        
        grp_traj.setLayout(lay_traj)
        self.program_tab_layout.addWidget(grp_traj)

        # --- FABRİKA OTOMASYONU KONTROL PANELİ ---
        grp_factory = QGroupBox("🏭 Fabrika Otomasyonu")
        grp_factory.setMaximumWidth(266)
        lay_factory = QVBoxLayout()
        lay_factory.setContentsMargins(8, 8, 8, 8)

        self.btn_automation = QPushButton("🏭 Otomasyon Senaryosunu Başlat")
        self.btn_automation.setStyleSheet("""
            background-color: #27ae60;
            color: white;
            font-weight: bold;
            padding: 10px;
            margin-top: 10px;
        """)
        self.btn_automation.clicked.connect(self.start_factory_scenario)
        lay_factory.addWidget(self.btn_automation)

        self.btn_stop_automation = QPushButton("🛑 Otomasyon Senaryosunu Bitir")
        self.btn_stop_automation.setStyleSheet("""
            background-color: #e74c3c;
            color: white;
            font-weight: bold;
            padding: 10px;
            margin-top: 5px;
        """)
        self.btn_stop_automation.clicked.connect(self.stop_factory_scenario)
        lay_factory.addWidget(self.btn_stop_automation)

        grp_factory.setLayout(lay_factory)
        self.program_tab_layout.addWidget(grp_factory)
        
        self.left_panel_layout.addStretch()
        self.io_tab_layout.addStretch()
        self.process_tab_layout.addStretch()
        self.program_tab_layout.addStretch()
        self.panel_layout.addStretch()

    # --- SESLİ KOMUT KONTROLÜ ---
    def start_voice_control(self):
        if sr is None:
            self.set_voice_status("SES HATASI: speech_recognition kurulu değil", "#e74c3c")
            self.set_status("SpeechRecognition eksik: pip install SpeechRecognition pyaudio", "#e74c3c")
            return
        if self.voice_worker is not None and self.voice_worker.isRunning():
            self.set_voice_status("SES KOMUTU ZATEN AKTİF", "#f1c40f")
            return

        self.voice_worker = VoiceCommandWorker()
        self.voice_worker.command_signal.connect(self.handle_voice_command)
        self.voice_worker.status_signal.connect(self.set_voice_status)
        self.voice_worker.finished.connect(self.on_voice_worker_finished)
        self.voice_worker.start()

        self.voice_active = True
        self.btn_voice_start.setEnabled(False)
        self.btn_voice_stop.setEnabled(True)
        self.set_voice_status("SES: Başlatılıyor...", "#f39c12")

    def stop_voice_control(self):
        if self.voice_worker is not None:
            self.voice_worker.stop()
            self.voice_worker = None

        self.voice_active = False
        if hasattr(self, "btn_voice_start"):
            self.btn_voice_start.setEnabled(True)
        if hasattr(self, "btn_voice_stop"):
            self.btn_voice_stop.setEnabled(False)
        self.set_voice_status("SES KOMUTU PASİF", "#95a5a6")

    def on_voice_worker_finished(self):
        self.voice_active = False
        if hasattr(self, "btn_voice_start"):
            self.btn_voice_start.setEnabled(True)
        if hasattr(self, "btn_voice_stop"):
            self.btn_voice_stop.setEnabled(False)

    def set_voice_status(self, text, color="#95a5a6"):
        if hasattr(self, "voice_status_label"):
            self.voice_status_label.setText(text)
            self.voice_status_label.setStyleSheet(
                f"color: {color}; padding: 6px; background: #222; border-radius: 4px; font-weight: bold;"
            )
        # Ses hatalarını ana durum çubuğuna da yansıt
        if "HATA" in text:
            self.set_status(text, color)

    def handle_voice_command(self, text):
        """Google Speech API'den gelen Türkçe metni güvenli robot komutlarına çevirir."""
        command = text.lower().strip()
        self.set_voice_status(f"ALGILANDI: {command}", "#00ffcc")

        if self.is_playing_sequence:
            self.set_status("SES KOMUTU REDDEDİLDİ: Yörünge oynatma aktif", "#e67e22")
            return

        # Otomasyon / senaryo komutları önce kontrol edilir.
        if any(k in command for k in ["senaryoyu başlat", "senaryo başlat", "otomasyonu başlat", "fabrikayı başlat", "fabrika başlat"]):
            self.start_factory_scenario()
            return

        if any(k in command for k in ["senaryoyu bitir", "senaryo bitir", "senaryoyu durdur", "otomasyonu durdur", "otomasyonu bitir", "fabrika durdur"]):
            self.stop_factory_scenario()
            return

        if any(k in command for k in ["dinlemeyi durdur", "sesi kapat", "mikrofonu kapat"]):
            self.stop_voice_control()
            return

        # Robot hareket ve HMI komutları
        if any(k in command for k in ["sıfırla", "sifirla", "home", "başlangıç", "baslangic", "eve dön", "eve don"]):
            self.reset_joints()
        elif any(k in command for k in ["ileri", "öne", "one"]):
            self.jog_world(self.voice_jog_step, 0, 0)
        elif any(k in command for k in ["geri", "arkaya"]):
            self.jog_world(-self.voice_jog_step, 0, 0)
        elif any(k in command for k in ["sağ", "sag"]):
            self.jog_world(0, self.voice_jog_step, 0)
        elif any(k in command for k in ["sol"]):
            self.jog_world(0, -self.voice_jog_step, 0)
        elif any(k in command for k in ["yukarı", "yukari", "yüksel", "yuksel"]):
            self.jog_world(0, 0, self.voice_jog_step)
        elif any(k in command for k in ["aşağı", "asagi", "alçal", "alcal"]):
            self.jog_world(0, 0, -self.voice_jog_step)
        elif any(k in command for k in ["nokta ekle", "nokta kaydet", "pozisyon kaydet", "waypoint"]):
            self.save_waypoint()
        elif any(k in command for k in ["oynat", "yörüngeyi oynat", "yorungeyi oynat", "programı çalıştır", "programi calistir"]):
            self.play_sequence()
        elif any(k in command for k in ["izi temizle", "iz temizle"]):
            self.clear_trail()
            self.set_status("SES: TCP izi temizlendi", "#2ecc71")
        elif any(k in command for k in ["çalışma alanı", "calisma alani", "workspace"]):
            self.toggle_workspace()
        else:
            self.set_status(f"SES KOMUTU TANINMADI: {command}", "#e67e22")


    # --- KAMERA İLE EL TAKİBİ KONTROLÜ ---
    def get_hand_tracker_pidfile(self):
        """Worker'ın bıraktığı PID dosyasının tam yolu."""
        return os.path.join(BASE_DIR, "hand_tracker_worker.pid")

    def get_hand_tracker_stopfile(self):
        """Worker'ın nazik kapanış için izlediği stop dosyasının tam yolu."""
        return os.path.join(BASE_DIR, "hand_tracker_worker.stop")

    def request_hand_tracker_stop(self):
        """Worker'a sert kill öncesi normal döngüden çıkması için işaret verir."""
        try:
            with open(self.get_hand_tracker_stopfile(), "w", encoding="utf-8") as f:
                f.write(str(time.time()))
        except Exception:
            pass

    def clear_hand_tracker_stopfile(self):
        try:
            stopfile = self.get_hand_tracker_stopfile()
            if os.path.exists(stopfile):
                os.remove(stopfile)
        except Exception:
            pass

    def kill_stale_hand_tracker_from_pidfile(self):
        """Önceki oturumdan açık kalmış el takip worker'ını kapatır.

        Böylece kullanıcı Kamera Aç'a bastığında webcam zaten bizim eski worker
        sürecimiz tarafından tutuluyorsa önce kapanır, sonra temiz başlatılır.
        """
        pidfile = self.get_hand_tracker_pidfile()
        if not os.path.exists(pidfile):
            return

        try:
            with open(pidfile, "r", encoding="utf-8") as f:
                pid = int(f.read().strip())
        except Exception:
            try:
                os.remove(pidfile)
            except Exception:
                pass
            return

        if pid <= 0 or pid == os.getpid():
            return

        killed = False
        try:
            if os.name == "nt":
                result = subprocess.run(
                    ["taskkill", "/F", "/T", "/PID", str(pid)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2,
                )
                killed = result.returncode == 0
            else:
                os.kill(pid, 15)
                killed = True
        except Exception:
            pass

        if killed:
            try:
                os.remove(pidfile)
            except Exception:
                pass
            self.clear_hand_tracker_stopfile()

    def kill_orphan_hand_tracker_by_commandline(self):
        """PID dosyası yoksa bile eski hand_tracker_worker.py süreçlerini kapatır.

        PowerShell komutu Base64 ile gönderilir; böylece komutun kendi satırı
        `hand_tracker_worker.py` içermez ve yanlışlıkla kendisini eşleştirmez.
        """
        if os.name != "nt":
            return

        script = r"""
$me = $PID
Get-CimInstance Win32_Process | Where-Object {
    $_.CommandLine -and
    $_.CommandLine -like '*hand_tracker_worker.py*' -and
    $_.ProcessId -ne $me
} | ForEach-Object {
    Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
}
"""
        try:
            encoded = base64.b64encode(script.encode("utf-16le")).decode("ascii")
            result = subprocess.run(
                ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-EncodedCommand", encoded],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3,
            )
            if result.returncode == 0:
                return
        except Exception:
            pass

        try:
            result = subprocess.run(
                [
                    "wmic",
                    "process",
                    "where",
                    "CommandLine like '%hand_tracker_worker.py%'",
                    "get",
                    "ProcessId",
                    "/value",
                ],
                capture_output=True,
                text=True,
                timeout=3,
            )
            for line in result.stdout.splitlines():
                if not line.startswith("ProcessId="):
                    continue
                try:
                    pid = int(line.split("=", 1)[1].strip())
                except Exception:
                    continue
                if pid > 0 and pid != os.getpid():
                    subprocess.run(
                        ["taskkill", "/F", "/T", "/PID", str(pid)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=2,
                    )
        except Exception:
            pass

    def start_hand_tracking(self):
        """El takip worker process'ini açar; açıksa önce kapatıp yeniden açar."""
        if self.is_playing_sequence or getattr(self, 'automation_active', False):
            self.set_status("EL TAKİBİ BAŞLATILAMADI: Otomatik görev aktif", "#e67e22")
            return

        # Kamera zaten bu uygulamanın worker'ı tarafından açıksa önce kapat, sonra yeniden aç.
        # Bu hem 'kamera açık' hatasını önler hem de Başlat butonunu güvenli restart yapar.
        if self.hand_process is not None and self.hand_process.state() != QProcess.NotRunning:
            self.set_hand_status("KAMERA ZATEN AÇIK - yeniden başlatılıyor...", "#f39c12")
            self.stop_hand_tracking(silent=True)
            QApplication.processEvents()
            time.sleep(0.25)

        # Uygulama daha önce kapanırken worker yetim kaldıysa önce PID dosyasından,
        # sonra komut satırından yakala ve kapat. Böylece kamera kilidi temizlenir.
        self.kill_stale_hand_tracker_from_pidfile()
        self.kill_orphan_hand_tracker_by_commandline()
        self.clear_hand_tracker_stopfile()
        time.sleep(0.20)

        self.hand_anchor = None
        self.robot_anchor = None
        self.hand_smoothed_offset = np.zeros(3, dtype=float)
        self.hand_stdout_buffer = b""
        self.last_hand_solve_time = 0.0
        self.camera_preview.setText("Kamera başlatılıyor...")
        camera_index = 0
        if hasattr(self, "camera_index_combo"):
            camera_index = int(self.camera_index_combo.currentData())

        worker_path = os.path.join(BASE_DIR, "hand_tracker_worker.py")
        if not os.path.exists(worker_path):
            self.set_hand_status("KAMERA HATASI: hand_tracker_worker.py bulunamadı", "#e74c3c")
            return

        self.hand_process = QProcess(self)
        self.hand_process.setProgram(sys.executable)
        self.hand_process.setArguments([worker_path, str(camera_index)])
        self.hand_process.readyReadStandardOutput.connect(self.read_hand_worker_stdout)
        self.hand_process.readyReadStandardError.connect(self.read_hand_worker_stderr)
        self.hand_process.finished.connect(self.on_hand_worker_finished)
        self.hand_process.start()

        if not self.hand_process.waitForStarted(1500):
            self.set_hand_status("KAMERA HATASI: Worker process başlatılamadı", "#e74c3c")
            self.hand_process = None
            return

        self.hand_tracking_active = True
        self.btn_hand_start.setEnabled(False)
        self.btn_hand_stop.setEnabled(True)
        self.btn_hand_recalibrate.setEnabled(True)
        self.set_hand_status("KAMERA: Başlatılıyor...", "#f39c12")

    def stop_hand_tracking(self, silent=False):
        """Kamera worker process'ini güvenli kapatır ve önizlemeyi temizler."""
        proc = self.hand_process
        self.hand_process = None
        self.hand_stdout_buffer = b""

        if proc is not None:
            try:
                try:
                    proc.readyReadStandardOutput.disconnect(self.read_hand_worker_stdout)
                except Exception:
                    pass
                try:
                    proc.readyReadStandardError.disconnect(self.read_hand_worker_stderr)
                except Exception:
                    pass
                try:
                    proc.finished.disconnect(self.on_hand_worker_finished)
                except Exception:
                    pass

                pid = None
                try:
                    pid = int(proc.processId())
                except Exception:
                    pid = None

                if proc.state() != QProcess.NotRunning:
                    # Önce nazik kapatma, sonra kesin kapatma. Windows'ta webcam kilidini
                    # bırakmayan worker kalırsa taskkill ile işlem ağacını da kapatıyoruz.
                    self.request_hand_tracker_stop()
                    if not proc.waitForFinished(1200):
                        try:
                            proc.write(b"stop\n")
                            proc.closeWriteChannel()
                        except Exception:
                            pass
                    if not proc.waitForFinished(800):
                        proc.terminate()
                    if not proc.waitForFinished(1200):
                        proc.kill()
                        proc.waitForFinished(800)

                if os.name == "nt" and pid:
                    try:
                        subprocess.run(
                            ["taskkill", "/F", "/T", "/PID", str(pid)],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=2,
                        )
                    except Exception:
                        pass
            finally:
                self.clear_hand_tracker_stopfile()
                try:
                    proc.deleteLater()
                except Exception:
                    pass

        if self.hand_worker is not None:
            self.hand_worker.stop()
            self.hand_worker = None

        self.hand_tracking_active = False
        self.hand_anchor = None
        self.robot_anchor = None
        self.hand_smoothed_offset = np.zeros(3, dtype=float)
        if hasattr(self, "btn_hand_start"):
            self.btn_hand_start.setEnabled(True)
        if hasattr(self, "btn_hand_stop"):
            self.btn_hand_stop.setEnabled(False)
        if hasattr(self, "btn_hand_recalibrate"):
            self.btn_hand_recalibrate.setEnabled(False)
        if hasattr(self, "camera_preview"):
            self.camera_preview.setText("Kamera kapalı")
            self.camera_preview.setPixmap(QPixmap())
        if (not silent) and (not getattr(self, "_closing", False)):
            self.set_hand_status("KAMERA / EL TAKİBİ PASİF", "#95a5a6")

    def on_hand_worker_finished(self):
        self.hand_tracking_active = False
        self.hand_anchor = None
        self.robot_anchor = None
        self.hand_smoothed_offset = np.zeros(3, dtype=float)
        self.hand_process = None
        if hasattr(self, "btn_hand_start"):
            self.btn_hand_start.setEnabled(True)
        if hasattr(self, "btn_hand_stop"):
            self.btn_hand_stop.setEnabled(False)
        if hasattr(self, "btn_hand_recalibrate"):
            self.btn_hand_recalibrate.setEnabled(False)

    def read_hand_worker_stdout(self):
        if self.hand_process is None:
            return
        self.hand_stdout_buffer += bytes(self.hand_process.readAllStandardOutput())
        while b"\n" in self.hand_stdout_buffer:
            line, self.hand_stdout_buffer = self.hand_stdout_buffer.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                data = json.loads(line.decode("utf-8", errors="replace"))
            except json.JSONDecodeError:
                continue

            if data.get("type") == "status":
                color = "#2ecc71" if data.get("level") == "ok" else "#e74c3c"
                self.set_hand_status(data.get("message", "KAMERA DURUMU"), color)
                continue

            self.update_camera_preview_from_payload(data)
            self.handle_hand_tracking_data(data)

    def read_hand_worker_stderr(self):
        if self.hand_process is None:
            return
        text = bytes(self.hand_process.readAllStandardError()).decode("utf-8", errors="replace").strip()
        if not text:
            return
        if os.environ.get("ROBOSIM_CAMERA_DEBUG") != "1":
            return
        for line in text.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            print("[RoboSim ElTakip stderr] " + stripped, flush=True)

    def set_hand_status(self, text, color="#95a5a6"):
        print(f"[RoboSim ElTakip] {text}", flush=True)
        if hasattr(self, "hand_status_label"):
            self.hand_status_label.setText(text)
            self.hand_status_label.setStyleSheet(
                f"color: {color}; padding: 6px; background: #222; border-radius: 4px; font-weight: bold;"
            )
        if "HATA" in text or "UYARI" in text:
            self.set_status(text, color)

    def update_camera_preview(self, qimage):
        if not hasattr(self, "camera_preview"):
            return
        pixmap = QPixmap.fromImage(qimage)
        self.camera_preview.setPixmap(
            pixmap.scaled(
                self.camera_preview.width(),
                self.camera_preview.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def update_camera_preview_from_payload(self, data):
        frame_b64 = data.get("frame_b64")
        if not frame_b64 or not hasattr(self, "camera_preview"):
            return
        try:
            image_bytes = base64.b64decode(frame_b64)
            qimage = QImage()
            if qimage.loadFromData(image_bytes, "JPG"):
                self.update_camera_preview(qimage)
        except Exception:
            pass

    def recalibrate_hand_tracking(self):
        """Bir sonraki görülen el pozunu yeni referans kabul eder."""
        self.hand_anchor = None
        self.robot_anchor = None
        self.hand_smoothed_offset = np.zeros(3, dtype=float)
        self.set_hand_status("KALİBRASYON: Elinizi ortada sabit tutun", "#f39c12")

    def handle_hand_tracking_data(self, data):
        """Kalibre edilmiş el merkezini güvenli TCP hedef konumuna map eder."""
        if not self.hand_tracking_active:
            return

        if self.is_playing_sequence or getattr(self, 'automation_active', False):
            self.set_hand_status("EL TAKİBİ BEKLEMEDE - Yörünge/otomasyon aktif", "#f1c40f")
            return

        self.hand_frame_counter = int(data.get("frame", self.hand_frame_counter))
        self.hand_detect_counter = int(data.get("detected", self.hand_detect_counter))

        if not data.get("hand_visible", data.get("visible", False)):
            if self.hand_frame_counter % 20 == 0:
                self.set_hand_status(
                    f"EL GÖRÜLMEDİ | Frame:{self.hand_frame_counter} Algılama:{self.hand_detect_counter}",
                    "#e67e22",
                )
            return

        now = time.time()
        self.hand_last_visible_time = now
        hx = float(np.clip(data.get("cx", data.get("x", 0.5)), 0.0, 1.0))
        hy = float(np.clip(data.get("cy", data.get("y", 0.5)), 0.0, 1.0))
        hs = float(np.clip(data.get("size", 0.2), 0.03, 0.9))

        if self.hand_anchor is None:
            self.hand_anchor = {
                "x": hx,
                "y": hy,
                "size": max(hs, 1e-3),
            }
            self.robot_anchor = get_fk_position(self.current_angles).copy()
            self.hand_smoothed_offset = np.zeros(3, dtype=float)
            self.set_hand_status(
                f"EL VAR / KALİBRE | cx:{hx:.2f} cy:{hy:.2f} boy:{hs:.2f}",
                "#2ecc71",
            )
            return

        if now - self.last_hand_solve_time < self.hand_update_period:
            return
        self.last_hand_solve_time = now

        depth_delta = (hs - self.hand_anchor["size"]) / max(self.hand_anchor["size"], 1e-3)
        raw_offset = np.array([
            depth_delta * self.hand_depth_scale,
            (hx - self.hand_anchor["x"]) * self.hand_y_scale,
            (self.hand_anchor["y"] - hy) * self.hand_z_scale,
        ], dtype=float)

        raw_offset[np.abs(raw_offset) < self.hand_deadzone] = 0.0
        self.hand_smoothed_offset = (
            (1.0 - self.hand_smoothing_alpha) * self.hand_smoothed_offset
            + self.hand_smoothing_alpha * raw_offset
        )

        target = self.robot_anchor + self.hand_smoothed_offset

        target[0] = np.clip(target[0], *self.hand_workspace_limits["x"])
        target[1] = np.clip(target[1], *self.hand_workspace_limits["y"])
        target[2] = np.clip(target[2], *self.hand_workspace_limits["z"])

        current_pos = get_fk_position(self.current_angles)
        dist = float(np.linalg.norm(target - current_pos))

        # Görünür debug: el algılanıyor mu ve hedef değişiyor mu artık açıkça görülür.
        debug_text = (
            f"EL VAR | cx:{hx:.2f} cy:{hy:.2f} boy:{hs:.2f} | "
            f"Hedef X:{target[0]:.0f} Y:{target[1]:.0f} Z:{target[2]:.0f} | Δ:{dist:.0f}"
        )

        if dist < 6.0:
            self.set_hand_status(debug_text + " | hedefte", "#00ffcc")
            return

        moved = self.solve_ik_and_move(
            target,
            animate=False,
            is_drag=True,
            max_iter_override=45,
            acceptance_mm=160.0,
            render_now=True,
        )
        if moved:
            self.set_hand_status(debug_text, "#00ffcc")
        else:
            # IK hedefi zorlanırsa yine de kullanıcıya elin algılandığını ve hedefin üretildiğini göster.
            self.set_hand_status("IK ZORLANDI | " + debug_text, "#e67e22")

    def set_status(self, text, color):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold; padding: 12px; background: #222; border-radius: 4px; border: 1px solid {color}; font-size: 13px;")

    def toggle_workspace(self):
        if self.workspace_actor is None:
            # Robotu kapatmayacak, arkada yarı saydam estetik bir tel kafes (wireframe) çizecek
            self.workspace_actor = self.plotter.add_mesh(
                self.workspace_mesh,
                color="#ec6602",
                style="wireframe",
                opacity=0.1,
                line_width=1,
                name="workspace_envelope"
            )
            self.btn_workspace.setText("🌐 Çalışma Alanını Gizle")
            self.btn_workspace.setStyleSheet("background-color: #e67e22; color: white; font-weight: bold; padding: 8px;")
            self.set_status("ÇALIŞMA ALANI Sınırları Gösteriliyor", "#3498db")
        else:
            self.plotter.remove_actor("workspace_envelope")
            self.workspace_actor = None
            self.btn_workspace.setText("🌐 Çalışma Alanını Göster")
            self.btn_workspace.setStyleSheet("background-color: #34495e; color: white; font-weight: bold; padding: 8px;")
            self.set_status("ÇALIŞMA ALANI Sınırları Gizlendi", "#2ecc71")
        
        self.plotter.render()
    def init_3d_scene(self):
        self.plotter.set_background("#121212")
        floor = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=4000, j_size=4000)
        self.plotter.add_mesh(floor, style='wireframe', color='#333333', line_width=1)
        self.plotter.add_axes()

        for link_name in ["base", "link1", "link2", "link3", "link4", "link5", "link6"]:
            files = file_mapping[link_name]
            combined_mesh = None
            for file in files:
                path = os.path.join(BASE_DIR, file)
                if os.path.exists(path):
                    mesh = pv.read(path).clean().compute_normals(cell_normals=False, point_normals=True, split_vertices=True)
                    combined_mesh = mesh if combined_mesh is None else combined_mesh.merge(mesh)
            
            if combined_mesh:
                actor = self.plotter.add_mesh(
                    combined_mesh, 
                    color=link_colors.get(link_name, "white"),
                    smooth_shading=True, specular=0.5, ambient=0.2, diffuse=0.8
                )
                self.link_actors[link_name] = actor

        self.tcp_sphere = self.plotter.add_mesh(pv.Sphere(radius=15), color="#00ffcc")
        self.init_tooling_scene()
        self.reset_camera()

    def init_tooling_scene(self):
        """TCP'ye bağlı seçilebilir uç takımlarını yerel TCP koordinatında oluşturur."""
        self.tool_actors = {}

        vacuum_parts = [
            self.plotter.add_mesh(pv.Cylinder(center=(42, 0, 0), direction=(1, 0, 0), radius=18, height=84), color="#2f3640", smooth_shading=True),
            self.plotter.add_mesh(pv.Cylinder(center=(95, 0, 0), direction=(1, 0, 0), radius=34, height=24), color="#111111", smooth_shading=True),
            self.plotter.add_mesh(pv.Sphere(center=(112, 0, 0), radius=22), color="#1abc9c", opacity=0.35, smooth_shading=True),
        ]
        self.tool_actors["vacuum"] = vacuum_parts

        laser_parts = [
            self.plotter.add_mesh(pv.Cylinder(center=(48, 0, 0), direction=(1, 0, 0), radius=14, height=96), color="#151515", smooth_shading=True),
            self.plotter.add_mesh(pv.Cone(center=(112, 0, 0), direction=(1, 0, 0), height=38, radius=18), color="#c0392b", smooth_shading=True),
            self.plotter.add_mesh(pv.Cylinder(center=(260, 0, 0), direction=(1, 0, 0), radius=4, height=260), color="#ff1f1f", opacity=0.65),
        ]
        self.tool_actors["laser"] = laser_parts

        spindle_parts = [
            self.plotter.add_mesh(pv.Cylinder(center=(45, 0, 0), direction=(1, 0, 0), radius=22, height=90), color="#566573", smooth_shading=True),
            self.plotter.add_mesh(pv.Cylinder(center=(108, 0, 0), direction=(1, 0, 0), radius=10, height=50), color="#d5d8dc", smooth_shading=True),
            self.plotter.add_mesh(pv.Cone(center=(145, 0, 0), direction=(1, 0, 0), height=34, radius=9), color="#f4d03f", smooth_shading=True),
        ]
        self.tool_actors["spindle"] = spindle_parts

        self.update_tool_visibility()

    def on_tool_mode_changed(self, _index=None):
        if not hasattr(self, "tool_mode_combo"):
            return
        self.tool_mode = self.tool_mode_combo.currentData()
        labels = {
            "vacuum": "Aktif tool: Vakum tutucu",
            "laser": "Aktif tool: Lazer kesici",
            "spindle": "Aktif tool: Mil / spindle",
        }
        if hasattr(self, "tool_status_label"):
            self.tool_status_label.setText(labels.get(self.tool_mode, "Aktif tool: -"))
        self.update_tool_visibility()
        self.set_status(labels.get(self.tool_mode, "Tool değiştirildi"), "#3498db")
        self.plotter.render()

    def update_tool_visibility(self):
        for mode, actors in getattr(self, "tool_actors", {}).items():
            visible = mode == self.tool_mode
            for actor in actors:
                try:
                    actor.SetVisibility(visible)
                except Exception:
                    pass
        if self.tool_mode != "laser" and self.laser_cut_active:
            self.laser_cut_active = False
            if hasattr(self, "btn_laser_cut"):
                self.btn_laser_cut.setText("Lazer Kesim: OFF")
                self.btn_laser_cut.setStyleSheet("background-color: #7f1d1d; color: white; font-weight: bold;")

    def toggle_laser_cut(self, _checked=False):
        if self.tool_mode != "laser":
            self.tool_mode = "laser"
            if hasattr(self, "tool_mode_combo"):
                idx = self.tool_mode_combo.findData("laser")
                if idx >= 0:
                    self.tool_mode_combo.setCurrentIndex(idx)
            self.update_tool_visibility()

        self.laser_cut_active = not self.laser_cut_active
        if hasattr(self, "btn_laser_cut"):
            if self.laser_cut_active:
                self.btn_laser_cut.setText("Lazer Kesim: ON")
                self.btn_laser_cut.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
                self.set_status("LAZER KESİM AKTİF - TCP izi kırmızı olarak işleniyor", "#e74c3c")
            else:
                self.btn_laser_cut.setText("Lazer Kesim: OFF")
                self.btn_laser_cut.setStyleSheet("background-color: #7f1d1d; color: white; font-weight: bold;")
                self.set_status("LAZER KESİM PASİF", "#95a5a6")

    def clear_laser_cut(self, _checked=False):
        self.laser_cut_points = []
        self.plotter.remove_actor(self.laser_cut_actor_name)
        self.set_status("LAZER KESİM İZİ TEMİZLENDİ", "#2ecc71")
        self.plotter.render()

    def reset_smartcell_dashboard(self):
        self.dashboard_total_count = 0
        self.dashboard_small_count = 0
        self.dashboard_large_count = 0
        self.dashboard_good_count = 0
        self.dashboard_defect_count = 0
        self.dashboard_cycle_start_time = None
        self.dashboard_last_cycle_time = 0.0
        self.dashboard_cycle_times = []
        self.dashboard_last_event = "Fabrika senaryosu başlatıldı"
        self.update_smartcell_dashboard()

    def mark_cycle_started(self):
        self.dashboard_cycle_start_time = time.time()
        self.dashboard_last_event = "Kalite kontrol başladı"
        self.update_smartcell_dashboard()

    def mark_product_completed(self):
        now = time.time()
        if self.dashboard_cycle_start_time is not None:
            cycle_time = max(0.0, now - self.dashboard_cycle_start_time)
            self.dashboard_last_cycle_time = cycle_time
            self.dashboard_cycle_times.append(cycle_time)
            if len(self.dashboard_cycle_times) > 50:
                self.dashboard_cycle_times = self.dashboard_cycle_times[-50:]
        self.dashboard_cycle_start_time = None

        self.dashboard_total_count += 1
        if self.box_quality == "HATALI":
            self.dashboard_defect_count += 1
            self.dashboard_last_event = "Hatalı ürün ayrıldı"
        else:
            self.dashboard_good_count += 1
            if self.box_type == "KÜÇÜK":
                self.dashboard_last_event = "Sağlam küçük ürün"
            elif self.box_type == "BÜYÜK":
                self.dashboard_last_event = "Sağlam büyük ürün"
            else:
                self.dashboard_last_event = "Sağlam ürün"

        if self.box_type == "KÜÇÜK":
            self.dashboard_small_count += 1
        elif self.box_type == "BÜYÜK":
            self.dashboard_large_count += 1
        self.update_smartcell_dashboard()

    def update_smartcell_dashboard(self):
        if not hasattr(self, "dashboard_labels"):
            return

        if self.automation_active:
            robot_status = "ÇALIŞIYOR" if (self.anim_timer.isActive() or self.auto_state != "IDLE") else "BEKLEME"
        elif self.is_playing_sequence:
            robot_status = "YÖRÜNGE"
        else:
            robot_status = "HAZIR"

        avg_cycle = float(np.mean(self.dashboard_cycle_times)) if self.dashboard_cycle_times else 0.0
        hourly = 3600.0 / avg_cycle if avg_cycle > 0.001 else 0.0
        quality_rate = (100.0 * self.dashboard_good_count / self.dashboard_total_count) if self.dashboard_total_count else 0.0
        active_cycle = 0.0
        if self.dashboard_cycle_start_time is not None:
            active_cycle = time.time() - self.dashboard_cycle_start_time

        if self.box_quality == "HATALI":
            box_label = "Hatalı ürün"
        elif self.box_type == "KÜÇÜK":
            box_label = "Küçük sağlam"
        elif self.box_type == "BÜYÜK":
            box_label = "Büyük sağlam"
        else:
            box_label = "-"

        values = {
            "robot": robot_status,
            "state": self.auto_state,
            "vacuum": "AKTİF" if self.is_holding_box else "PASİF",
            "box_type": box_label,
            "total": str(self.dashboard_total_count),
            "small": str(self.dashboard_small_count),
            "large": str(self.dashboard_large_count),
            "good": str(self.dashboard_good_count),
            "defect": str(self.dashboard_defect_count),
            "quality": f"{quality_rate:.1f}%" if self.dashboard_total_count else "--",
            "last_cycle": f"{self.dashboard_last_cycle_time:.1f} sn",
            "avg_cycle": f"{avg_cycle:.1f} sn",
            "hourly": f"{hourly:.0f} adet/saat" if hourly > 0 else "--",
            "event": self.dashboard_last_event,
        }

        if active_cycle > 0 and self.auto_state not in ("IDLE", "SPAWN_BOX"):
            values["event"] = f"{self.dashboard_last_event} | {active_cycle:.1f} sn"

        for key, value in values.items():
            label = self.dashboard_labels.get(key)
            if label is not None:
                label.setText(value)

    def get_production_report_snapshot(self):
        if self.automation_active:
            robot_status = "ÇALIŞIYOR" if (self.anim_timer.isActive() or self.auto_state != "IDLE") else "BEKLEME"
        elif self.is_playing_sequence:
            robot_status = "YÖRÜNGE"
        else:
            robot_status = "HAZIR"

        avg_cycle = float(np.mean(self.dashboard_cycle_times)) if self.dashboard_cycle_times else 0.0
        hourly = 3600.0 / avg_cycle if avg_cycle > 0.001 else 0.0
        quality_rate = (100.0 * self.dashboard_good_count / self.dashboard_total_count) if self.dashboard_total_count else 0.0

        return {
            "Tarih/saat": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Robot durumu": robot_status,
            "Aktif state": self.auto_state,
            "Toplam ürün": self.dashboard_total_count,
            "Küçük ürün": self.dashboard_small_count,
            "Büyük ürün": self.dashboard_large_count,
            "Sağlam ürün": self.dashboard_good_count,
            "Hatalı ürün": self.dashboard_defect_count,
            "Kalite oranı": f"{quality_rate:.1f}%",
            "Son çevrim süresi": f"{self.dashboard_last_cycle_time:.2f} sn",
            "Ortalama çevrim süresi": f"{avg_cycle:.2f} sn",
            "Tahmini saatlik üretim": f"{hourly:.0f} adet/saat" if hourly > 0 else "--",
            "Son olay": self.dashboard_last_event,
        }

    def export_production_report(self, _checked=False):
        try:
            reports_dir = os.path.join(BASE_DIR, "reports")
            os.makedirs(reports_dir, exist_ok=True)

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"robosim_uretim_raporu_{stamp}"
            txt_path = os.path.join(reports_dir, base_name + ".txt")
            csv_path = os.path.join(reports_dir, base_name + ".csv")
            snapshot = self.get_production_report_snapshot()

            with open(txt_path, "w", encoding="utf-8") as f:
                f.write("RoboSim Üretim Raporu\n")
                f.write("=" * 28 + "\n\n")
                for key, value in snapshot.items():
                    f.write(f"{key}: {value}\n")

                if self.dashboard_cycle_times:
                    f.write("\nCycle Time Listesi\n")
                    f.write("-" * 20 + "\n")
                    for idx, cycle_time in enumerate(self.dashboard_cycle_times, start=1):
                        f.write(f"{idx}: {cycle_time:.2f} sn\n")

            with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Alan", "Değer"])
                for key, value in snapshot.items():
                    writer.writerow([key, value])

                writer.writerow([])
                writer.writerow(["Cycle No", "Çevrim Süresi (sn)"])
                for idx, cycle_time in enumerate(self.dashboard_cycle_times, start=1):
                    writer.writerow([idx, f"{cycle_time:.2f}"])

            self.set_status("Üretim raporu kaydedildi", "#2ecc71")
            self.dashboard_last_event = f"Rapor kaydedildi: {os.path.basename(txt_path)}"
            self.update_smartcell_dashboard()
        except Exception as exc:
            self.set_status(f"RAPOR HATASI: {type(exc).__name__}", "#e74c3c")

    def get_plc_io_values(self):
        box_visible = False
        box_x = None
        if hasattr(self, 'box_actor') and self.box_actor is not None:
            try:
                box_visible = bool(self.box_actor.GetVisibility())
                box_x = float(self.box_actor.GetPosition()[0])
            except Exception:
                box_visible = False
                box_x = None

        sensor_in = (
            self.auto_state in ("APPROACH", "PICK", "LIFT")
            or (self.auto_state == "CONVEYOR_IN" and box_x is not None and box_x <= 560.0)
        )
        reject_states = {
            "REJECT_PREPARE", "REJECT_TURN", "REJECT_APPROACH",
            "REJECT_DROP", "REJECT_LIFT", "REJECT_WAIT",
        }
        vacuum_states = {
            "PICK", "LIFT", "DROP",
            "REJECT_PREPARE", "REJECT_TURN", "REJECT_APPROACH",
        }
        status_text = self.status_label.text() if hasattr(self, "status_label") else ""
        estop = bool(getattr(self, "_closing", False) or "HATA" in status_text or "ERROR" in status_text)

        return {
            "I0.0": int(sensor_in),
            "I0.1": int(box_visible),
            "I0.2": int(box_visible and self.box_quality == "SAĞLAM"),
            "I0.3": int(box_visible and self.box_quality == "HATALI"),
            "I0.4": int(estop),
            "Q0.0": int(self.auto_state == "CONVEYOR_IN"),
            "Q0.1": int(self.auto_state == "CONVEYOR_OUT"),
            "Q0.2": int(bool(self.is_holding_box) or self.auto_state in vacuum_states),
            "Q0.3": int(self.auto_state in reject_states),
            "Q0.4": int(estop),
        }

    def update_plc_io_panel(self):
        if not hasattr(self, "plc_io_labels"):
            return
        values = self.get_plc_io_values()
        for addr, value in values.items():
            label = self.plc_io_labels.get(addr)
            if label is None:
                continue
            label.setText(str(int(value)))
            if value:
                label.setStyleSheet(
                    "background:#16a34a; color:white; border:1px solid #22c55e; "
                    "border-radius:4px; padding:2px; font-family:Consolas; font-weight:bold;"
                )
            else:
                label.setStyleSheet(
                    "background:#374151; color:#cbd5e1; border:1px solid #4b5563; "
                    "border-radius:4px; padding:2px; font-family:Consolas; font-weight:bold;"
                )

    # --- SIFIR GECİKMELİ GÜVENLİ KAMERA REJİSİ ---
    def set_cam(self, view_tuple):
        self.plotter.camera_position = view_tuple
        
    def reset_camera(self):
        self.plotter.camera_position = [(5200, -3900, 2850), (150, 0, 700), (0, 0, 1)]

    # --- KOD ÜRETİCİ ---
    def show_export_dialog(self):
        dialog = CodeExportDialog(self.waypoints, self.speed_multiplier, self)
        dialog.exec_()

    # --- YÖRÜNGE VE JOGGING ---
    def jog_world(self, dx, dy, dz):
        current_pos = get_fk_position(self.current_angles)
        target = [current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz]
        self.solve_ik_and_move(target, animate=False, is_drag=True)

    def save_waypoint(self):
        pos = get_fk_position(self.current_angles)
        point_data = {
            "name": f"P{len(self.waypoints) + 1}",
            "angles": self.current_angles.copy(),
            "xyz": pos
        }
        self.waypoints.append(point_data)
        self.list_waypoints.addItem(f"{point_data['name']} : X:{pos[0]:.0f} Y:{pos[1]:.0f} Z:{pos[2]:.0f}")
        self.set_status(f"{point_data['name']} KAYDEDİLDİ", "#f1c40f")

    def clear_waypoints(self):
        self.waypoints.clear()
        self.list_waypoints.clear()
        self.set_status("YÖRÜNGE HAFIZASI SİLİNDİ", "#e74c3c")
        
    def play_sequence(self):
        if not self.waypoints:
            self.set_status("HATA: OYNATILACAK NOKTA YOK!", "#e74c3c")
            return
        
        self.clear_trail()
        self.is_playing_sequence = True
        self.sequence_index = 0
        self.go_to_next_waypoint()

    def go_to_next_waypoint(self):
        if self.sequence_index < len(self.waypoints):
            target = self.waypoints[self.sequence_index]
            self.list_waypoints.setCurrentRow(self.sequence_index)
            self.set_status(f"GÖREV ÇALIŞIYOR: {target['name']}'e Gidiliyor...", "#3498db")
            
            max_diff = np.max(np.abs(np.array(target["angles"]) - np.array(self.current_angles)))
            base_duration = int((max_diff * 15) + 500)
            final_duration = int(base_duration / self.speed_multiplier)
            
            self.animate_to(target["angles"], duration_ms=final_duration)
        else:
            self.is_playing_sequence = False
            self.set_status("YÖRÜNGE TAMAMLANDI - SİSTEM HAZIR", "#2ecc71")

    # --- ANİMASYON MANTIĞI ---
    def animate_to(self, target_angles, duration_ms=1000):
        if self.anim_timer.isActive():
            self.anim_timer.stop()
            
        duration_ms = max(200, min(duration_ms, 5000)) 
        
        self.anim_start_angles = self.current_angles.copy()
        self.anim_target_angles = np.array(target_angles).copy()
        self.anim_steps = max(1, int(duration_ms / 16))
        self.anim_current_step = 0
        
        if not self.is_playing_sequence:
            self.set_status("ROBOT HAREKET EDİYOR...", "#f39c12")
        self.anim_timer.start(16)

    def anim_step(self):
        self.anim_current_step += 1
        t = self.anim_current_step / self.anim_steps
        
        if t >= 1.0:
            t = 1.0
            self.anim_timer.stop()
            
            if self.is_playing_sequence:
                self.sequence_index += 1
                QTimer.singleShot(200, self.go_to_next_waypoint)
            # --- OTOMASYON GEÇİŞ KONTROLÜ ---
            elif hasattr(self, 'automation_active') and self.automation_active:
                QTimer.singleShot(120, self.next_automation_step)
            else:
                self.set_status("HEDEFE ULAŞILDI - SİSTEM HAZIR", "#2ecc71")

        # Döngüden hemen önce ekle:
        t_smooth = t * t * (3 - 2 * t)
        self.is_updating_ui = True
        for i in range(6):
            angle = self.anim_start_angles[i] + (self.anim_target_angles[i] - self.anim_start_angles[i]) * t_smooth
            self.current_angles[i] = angle
            self.sliders[i].setValue(int(angle))
            self.spinboxes[i].setValue(float(angle))
        self.is_updating_ui = False
        
        self.update_robot_and_target()

    # --- KONTROL MEKANİZMALARI ---
    def on_slider_change(self, index, value):
        if self.is_updating_ui or self.is_playing_sequence: return
        if self.anim_timer.isActive():
            self.anim_timer.stop()
        
        self.is_updating_ui = True
        self.current_angles[index] = value
        self.spinboxes[index].setValue(float(value))
        self.update_robot_and_target()
        self.is_updating_ui = False

    def on_spinbox_change(self, index, value):
        if self.is_updating_ui or self.is_playing_sequence: return
        
        diff = abs(self.current_angles[index] - value)
        if diff > 2.0:
            target = self.current_angles.copy()
            target[index] = value
            self.animate_to(target, duration_ms=int(diff * 15 / self.speed_multiplier))
        else: 
            if self.anim_timer.isActive():
                self.anim_timer.stop()
            self.is_updating_ui = True
            self.current_angles[index] = value
            self.sliders[index].setValue(int(value))
            self.update_robot_and_target()
            self.is_updating_ui = False

    def reset_joints(self):
        self.clear_trail()
        self.is_playing_sequence = False
        self.animate_to([0.0] * 6, duration_ms=int(1500 / self.speed_multiplier))

    def update_robot_and_target(self):
        self.update_robot()
        current_pos = get_fk_position(self.current_angles)
        if self.drag_target is None:
            self.target_widget.SetCenter(current_pos)

        # --- VAKUM KONTROLÜ: KUTUYU ROBOTUN UCUNA KİLİTLE ---
        if (
            hasattr(self, 'automation_active') and self.automation_active
            and getattr(self, 'is_holding_box', False)
            and hasattr(self, 'box_actor')
        ):

            T_tcp = get_fk_matrix(self.current_angles)
            x, y, z = T_tcp[:3, 3]

            # SADECE POZİSYON GÜNCELLE (RECREATE YOK!)
            self.box_actor.SetPosition(
                x,
                y,
                z - (self.box_current_size / 2)
            )

        if (getattr(self, 'automation_active', False) 
            and getattr(self, 'is_holding_box', False) 
            and hasattr(self, 'box_actor')):
            
            T_tcp = get_fk_matrix(self.current_angles)
            T_box_new = T_tcp @ self.grip_offset
            
            # Update position (PyVista handles the rest)
            self.box_actor.SetPosition(T_box_new[0, 3], T_box_new[1, 3], T_box_new[2, 3])

    def clear_trail(self):
        self.trail_points = []
        self.plotter.remove_actor("tcp_trail")
    
    def run_ik_from_ui(self):
        try:
            target_pos = [
                float(self.ik_inputs["X"].text()),
                float(self.ik_inputs["Y"].text()),
                float(self.ik_inputs["Z"].text())
            ]
            self.solve_ik_and_move(target_pos, animate=True)
            self.target_widget.SetCenter(target_pos)
        except ValueError:
            self.set_status("HATA: GEÇERSİZ KOORDİNAT FORMATI!", "#e74c3c")

    # --- CANLI FARE TAKİBİ (LIVE IK DRAG) ---
    def on_mouse_drag(self, center):
        if self.is_playing_sequence: return 
        if self.anim_timer.isActive():
            self.anim_timer.stop()
        self.drag_target = np.array(center)

    def process_live_drag(self):
        if self.drag_target is not None:
            curr_pos = get_fk_position(self.current_angles)
            dist = np.linalg.norm(curr_pos - self.drag_target)
            
            if dist < 2.0:
                self.drag_target = None
                return

            old_angles = self.current_angles.copy()
            self.solve_ik_and_move(self.drag_target, animate=False, is_drag=True)
            
            angle_diff = np.max(np.abs(np.array(self.current_angles) - np.array(old_angles)))
            if angle_diff < 0.1:
                self.drag_target = None

    def solve_ik_and_move(self, target_pos, animate=False, is_drag=False, max_iter_override=None, acceptance_mm=30.0, render_now=False):
        target_pos = np.array(target_pos, dtype=float)
        
        def objective(angles):
            current_pos = get_fk_position(angles)
            pos_error = np.linalg.norm(current_pos - target_pos)
            if not is_drag:
                wrist_penalty = 0.001 * (angles[3]**2 + angles[4]**2 + angles[5]**2)
                return pos_error + wrist_penalty
            return pos_error

        m_iter = 5 if is_drag else 50
        if max_iter_override is not None:
            m_iter = int(max_iter_override)
        f_tol = 1e-2 if is_drag else 1e-3

        result = minimize(
            objective, 
            self.current_angles,          
            bounds=slider_limits,         
            method='L-BFGS-B',
            options={'ftol': f_tol, 'maxiter': m_iter} 
        )

        current_error = float(np.linalg.norm(get_fk_position(self.current_angles) - target_pos))
        result_error = float(np.linalg.norm(get_fk_position(result.x) - target_pos))

        if result.success or result_error < acceptance_mm or result_error < (current_error - 3.0):
            new_angles = result.x
            
            if animate:
                max_diff = np.max(np.abs(new_angles - self.current_angles))
                dur = int((max_diff * 15 + 500) / self.speed_multiplier)
                if getattr(self, 'automation_active', False):
                    dur = int(dur / max(getattr(self, 'factory_speed_factor', 1.0), 1.0))
                self.animate_to(new_angles, duration_ms=dur)
            else:
                self.is_updating_ui = True
                for i in range(6):
                    self.current_angles[i] = float(new_angles[i])
                    self.sliders[i].setValue(int(new_angles[i]))
                    self.spinboxes[i].setValue(float(new_angles[i]))
                self.is_updating_ui = False
                self.update_robot_and_target()
                if render_now:
                    self.plotter.render()
                if not is_drag:
                    self.set_status("HEDEF TAKİP EDİLİYOR...", "#3498db")
            return True
        else:
            if not is_drag:
                self.set_status("UYARI: HEDEF ERİŞİM ALANI DIŞINDA!", "#e67e22")
            return False

    # --- ULTRA PERFORMANSLI GÖRSEL VE HUD GÜNCELLEMESİ ---
    def update_robot(self):
        T_matrix = np.eye(4)
        transforms = []
        for i in range(6):
            Ti = axis_angle_transform(self.current_angles[i], joints[i]["axis"], joints[i]["point"])
            T_matrix = T_matrix @ Ti
            transforms.append(T_matrix.copy())

        for i in range(1, 7):
            link = f"link{i}"
            if link in self.link_actors:
                self.link_actors[link].user_matrix = transforms[i-1]

        T_tcp = get_fk_matrix(self.current_angles)
        x, y, z = T_tcp[:3, 3]
        roll, pitch, yaw = get_rpy_from_matrix(T_tcp[:3, :3])
        
        self.tcp_sphere.position = [x, y, z]
        self.update_tcp_tool(T_tcp)

        # MESAFE FİLTRESİ: Robot en az 15mm hareket etmediyse yeni çizgi noktası ekleme (Kasmayı önleyen kalbimiz)
        if len(self.trail_points) == 0:
            self.trail_points.append([x, y, z])
        else:
            dist_from_last = np.linalg.norm(np.array([x, y, z]) - np.array(self.trail_points[-1]))
            if dist_from_last > 15.0: 
                self.trail_points.append([x, y, z])

        if len(self.trail_points) >= 2:
            trail_mesh = pv.lines_from_points(np.array(self.trail_points))
            # Ağır 3D tüp çizimi yerine performans canavarı düz çizgiye geçildi
            self.plotter.add_mesh(trail_mesh, color='#00ffcc', line_width=3, name="tcp_trail")

        self.update_laser_cut_path(T_tcp)

        self.tcp_overlay.setText(
            f"<span style='color:#ec6602;'>■</span> UÇ İŞLEVCİ (TCP)<br><br>"
            f"<span style='color:#888;'>POZİSYON:</span><br>"
            f"X: {x:>7.1f} mm<br>"
            f"Y: {y:>7.1f} mm<br>"
            f"Z: {z:>7.1f} mm<br><br>"
            f"<span style='color:#888;'>YÖNELİM (RPY):</span><br>"
            f"Rx: {roll:>6.1f}°<br>"
            f"Ry: {pitch:>6.1f}°<br>"
            f"Rz: {yaw:>6.1f}°"
        )
        self.tcp_overlay.adjustSize()

        self.matrix_overlay.setText(
            f"<b>Homojen Dönüşüm Matrisi (T)</b><br>"
            f"[{T_tcp[0,0]:>5.2f}  {T_tcp[0,1]:>5.2f}  {T_tcp[0,2]:>5.2f} | {x:>6.1f}]<br>"
            f"[{T_tcp[1,0]:>5.2f}  {T_tcp[1,1]:>5.2f}  {T_tcp[1,2]:>5.2f} | {y:>6.1f}]<br>"
            f"[{T_tcp[2,0]:>5.2f}  {T_tcp[2,1]:>5.2f}  {T_tcp[2,2]:>5.2f} | {z:>6.1f}]<br>"
            f"[ 0.00   0.00   0.00 |    1.0 ]"
        )
        self.matrix_overlay.adjustSize()
        self.matrix_overlay.move(20, self.plotter.interactor.height() - self.matrix_overlay.height() - 20)
        
        if not self.is_updating_ui:
            self.ik_inputs["X"].setText(f"{x:.1f}")
            self.ik_inputs["Y"].setText(f"{y:.1f}")
            self.ik_inputs["Z"].setText(f"{z:.1f}")

    def update_tcp_tool(self, T_tcp):
        for actors in getattr(self, "tool_actors", {}).values():
            for actor in actors:
                try:
                    actor.user_matrix = T_tcp
                except Exception:
                    pass

    def update_laser_cut_path(self, T_tcp):
        if not getattr(self, "laser_cut_active", False) or self.tool_mode != "laser":
            return

        tip_local = np.array([390.0, 0.0, 0.0, 1.0])
        p = (T_tcp @ tip_local)[:3]
        if not self.laser_cut_points:
            self.laser_cut_points.append(p)
        else:
            last = np.array(self.laser_cut_points[-1])
            if np.linalg.norm(p - last) > 8.0:
                self.laser_cut_points.append(p)

        if len(self.laser_cut_points) > 600:
            self.laser_cut_points = self.laser_cut_points[-600:]

        if len(self.laser_cut_points) >= 2:
            cut_mesh = pv.lines_from_points(np.array(self.laser_cut_points))
            self.plotter.add_mesh(
                cut_mesh,
                color="#ff2a2a",
                line_width=5,
                name=self.laser_cut_actor_name,
            )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'matrix_overlay'):
            self.matrix_overlay.move(20, self.plotter.interactor.height() - self.matrix_overlay.height() - 20)

    def finish_box(self):
        if hasattr(self, "box_actor") and self.box_actor is not None:
            self.plotter.remove_actor(self.box_actor)
            self.box_actor = None        

    def apply_theme(self):
        css = """
            QMainWindow, QWidget { background-color: #1a1a1a; color: #e0e0e0; font-family: 'Segoe UI'; font-size: 11px; }
            QGroupBox { border: 1px solid #30363d; border-radius: 6px; margin-top: 10px; font-weight: bold; padding-top: 6px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #f0f3f5; }
            QSlider::groove:horizontal { border: none; height: 4px; background: #333; border-radius: 2px; }
            QSlider::handle:horizontal { background: #ec6602; width: 12px; margin: -5px 0; border-radius: 6px; }
            QSlider::handle:horizontal:hover { background: #ff7f1f; }
            QPushButton { background-color: #24292f; border: 1px solid #3a4149; border-radius: 5px; padding: 5px; font-weight: bold; }
            QPushButton:hover { background-color: #3a3a3a; }
            QPushButton:pressed { background-color: #ec6602; color: #000; }
            QPushButton#actionButton { background-color: #ec6602; color: #fff; }
            QPushButton#actionButton:hover { background-color: #ff7f1f; }
            QLabel#orangeText { color: #ec6602; font-weight: bold; font-family: 'Consolas'; font-size: 13px; }
            QLineEdit { background-color: #2a2a2a; border: 1px solid #444; color: #fff; padding: 4px; border-radius: 3px; }
            QDoubleSpinBox { background-color: #2a2a2a; border: 1px solid #444; color: #ec6602; padding: 3px; border-radius: 3px; font-weight: bold;}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { background: #333; width: 12px; }
            QDoubleSpinBox:focus { border: 1px solid #ec6602; }
            QListWidget { background-color: #222; border: 1px solid #444; color: #ccc; border-radius: 4px; padding: 5px; font-family: 'Consolas'; font-size: 10px; }
            QListWidget::item:selected { background-color: #ec6602; color: white; font-weight: bold; }
            QTabWidget::pane { border: 1px solid #30363d; border-radius: 6px; background: #171a1d; top: -1px; }
            QTabBar::tab { background: #24292f; color: #c9d1d9; border: 1px solid #30363d; padding: 6px 10px; min-width: 54px; }
            QTabBar::tab:selected { background: #ec6602; color: white; }
            QTabBar::tab:first { border-top-left-radius: 5px; }
            QTabBar::tab:last { border-top-right-radius: 5px; }
            QScrollBar:vertical { background: #1a1a1a; width: 10px; margin: 0px; }
            QScrollBar::handle:vertical { background: #444; border-radius: 5px; min-height: 20px; }
            QScrollBar::handle:vertical:hover { background: #ec6602; }
            QScrollBar:horizontal { height: 0px; background: transparent; }
        """
        self.setStyleSheet(css)

    def start_factory_scenario(self):
        if getattr(self, "laser_cut_active", False):
            self.toggle_laser_cut()
        self.tool_mode = "vacuum"
        if hasattr(self, "tool_mode_combo"):
            idx = self.tool_mode_combo.findData("vacuum")
            if idx >= 0:
                self.tool_mode_combo.setCurrentIndex(idx)
        self.update_tool_visibility()

        # Eğer önceden açık kalan bir senaryo elemanı varsa tamamen temizle (Mükerrer önleyici)
        if hasattr(self, 'factory_actors'):
            for actor in self.factory_actors:
                self.plotter.remove_actor(actor)
        self.factory_actors = []

        self.automation_active = True
        self.auto_state = "SPAWN_BOX"
        self.is_holding_box = False
        self.reset_smartcell_dashboard()
        self.set_status("FABRİKA OTOMASYONU AKTİF - Ağır Sanayi Konveyörleri Devreye Alındı", "#2ecc71")
        
        # --- BÜYÜTÜLMÜŞ GERÇEKÇİ ÖLÇEKLİ KONVEYÖRLER ---
        # Giriş Konveyörü (X Ekseni Boyunca Geniş Bant)
        self.build_realistic_conveyor(center=(650, 0, 10), length=850, width=450, height=230, orientation="X", color="#2c3e50")
        
        # Çıkış Konveyörü 1 (Küçük Kutular İçin - Sol)
        self.build_realistic_conveyor(center=(250, -800, 10), length=450, width=900, height=230, orientation="Y", color="#34495e")
        
        # Çıkış Konveyörü 2 (Büyük Kutular İçin - Sağ)
        self.build_realistic_conveyor(center=(250, 800, 10), length=450, width=900, height=230, orientation="Y", color="#34495e")

        self.build_tool_changer_station()
        self.build_laser_process_station()
        self.build_reject_station()
        
        # 2. Genişletilmiş Kızılötesi Sensör Çizgisi (Yeni genişliğe tam oturması için y_length=450 yapıldı)
        laser = pv.Cube(center=(550, 0, 160), x_length=12, y_length=450, z_length=6)
        laser_actor = self.plotter.add_mesh(laser, color="#e74c3c", opacity=0.7, name="laser_sensor")
        self.factory_actors.append(laser_actor)
        
        # 3. Kutu Aktör Havuzu Kontrolü
        if not hasattr(self, 'box_actor'):
            self.box_mesh = pv.Cube(x_length=1, y_length=1, z_length=1)
            self.box_actor = self.plotter.add_mesh(self.box_mesh, color="#3498db", name="production_box")
        self.box_actor.SetVisibility(False)
        
        # 4. Zamanlayıcı ve Fizik Motorunu Başlat
        self.last_time = time.time()
        if not hasattr(self, 'physics_timer'):
            self.physics_timer = QTimer()
            self.physics_timer.timeout.connect(self.physics_tick)
        self.physics_timer.start(16)
        
        self.next_automation_step()

    def stop_factory_scenario(self):
        """Senaryoyu durdurur, tüm fabrikasyon elemanları siler ve robotu sıfırlar."""
        # 1. Otomasyon mantıksal kilitlerini kapat
        self.automation_active = False
        self.is_holding_box = False
        self.auto_state = "IDLE"
        self.box_quality = None
        self.box_route = None
        self.dashboard_last_event = "Fabrika senaryosu durduruldu"
        self.update_smartcell_dashboard()
        
        # 2. Fizik zamanlayıcısını durdur (Kutunun akması kesilir)
        if hasattr(self, 'physics_timer') and self.physics_timer.isActive():
            self.physics_timer.stop()
            
        # 3. Üretim kutusunu gizle
        if hasattr(self, 'box_actor') and self.box_actor is not None:
            self.box_actor.SetVisibility(False)
            
        # 4. Sahneye inşa edilmiş tüm konveyör parçalarını ve lazeri tek tek sil
        if hasattr(self, 'factory_actors'):
            for actor in self.factory_actors:
                self.plotter.remove_actor(actor)
            self.factory_actors.clear() # Bellekteki listeyi sıfırla
            
        # 5. Robotun ekranda bıraktığı kırmızı çalışma çizgilerini temizle
        self.clear_trail()
        
        # 6. Robotu pürüzsüzce başlangıç (Sıfır) konumuna geri döndür
        self.animate_to([0.0] * 6, duration_ms=900)
        
        # 7. Kullanıcı arayüzünü bilgilendir ve ekranı tazele
        self.set_status("SİSTEM RESETLENDİ: Fabrika Modu Kapatıldı ve Temizlendi", "#e74c3c")
        self.plotter.render()    

    def build_realistic_conveyor(self, center, length, width, height, orientation, color):
        # Aktörleri takip etmek için liste yoksa oluştur
        if not hasattr(self, 'factory_actors'):
            self.factory_actors = []
            
        cx, cy, cz = center
        
        # 1. Ana Taşıyıcı Ağır Şase
        frame = pv.Cube(center=center, x_length=length, y_length=width, z_length=height)
        a1 = self.plotter.add_mesh(frame, color=color, smooth_shading=True)
        self.factory_actors.append(a1)
        
        # Üst yüzey kotu hesabı
        bt = cz + height/2 + 2
        
        # 2. Siyah Hareketli Bant ve Kenar Ruloları
        if orientation == "X":
            belt = pv.Cube(center=(cx, cy, bt), x_length=length, y_length=width-30, z_length=4)
            r1 = pv.Cylinder(center=(cx - length/2, cy, bt), direction=(0,1,0), radius=18, height=width-20)
            r2 = pv.Cylinder(center=(cx + length/2, cy, bt), direction=(0,1,0), radius=18, height=width-20)
            
            # Endüstriyel Sarı Güvenlik Bariyerleri (Yan Korumalar)
            g1 = pv.Cube(center=(cx, cy - width/2, bt+15), x_length=length, y_length=6, z_length=30)
            g2 = pv.Cube(center=(cx, cy + width/2, bt+15), x_length=length, y_length=6, z_length=30)
        else:
            belt = pv.Cube(center=(cx, cy, bt), x_length=length-30, y_length=width, z_length=4)
            r1 = pv.Cylinder(center=(cx, cy - width/2, bt), direction=(1,0,0), radius=18, height=length-20)
            r2 = pv.Cylinder(center=(cx, cy + width/2, bt), direction=(1,0,0), radius=18, height=length-20)
            
            g1 = pv.Cube(center=(cx - length/2, cy, bt+15), x_length=6, y_length=width, z_length=30)
            g2 = pv.Cube(center=(cx + length/2, cy, bt+15), x_length=6, y_length=width, z_length=30)
            
        a2 = self.plotter.add_mesh(belt, color="#161616")
        a3 = self.plotter.add_mesh(r1, color="#7f8c8d")
        a4 = self.plotter.add_mesh(r2, color="#7f8c8d")
        a5 = self.plotter.add_mesh(g1, color="#f1c40f")
        a6 = self.plotter.add_mesh(g2, color="#f1c40f")
        
        # Oluşan tüm 3D nesneleri yok edilmek üzere hafızaya kaydet
        self.factory_actors.extend([a2, a3, a4, a5, a6])

        # 3. Taşıyıcı ayaklar ve taban pabuçları
        leg_z = cz - height / 2 - 55
        foot_z = leg_z - 70
        leg_positions = [
            (cx - length * 0.42, cy - width * 0.38),
            (cx - length * 0.42, cy + width * 0.38),
            (cx + length * 0.42, cy - width * 0.38),
            (cx + length * 0.42, cy + width * 0.38),
        ]
        detail_actors = []
        for lx, ly in leg_positions:
            leg = pv.Cylinder(center=(lx, ly, leg_z), direction=(0, 0, 1), radius=10, height=140)
            foot = pv.Cube(center=(lx, ly, foot_z), x_length=58, y_length=58, z_length=10)
            detail_actors.append(self.plotter.add_mesh(leg, color="#596275", smooth_shading=True))
            detail_actors.append(self.plotter.add_mesh(foot, color="#2d3436"))

        # 4. Ara rulolar: bant altında ritmik metal detay
        roller_count = 5 if orientation == "X" else 6
        for k in range(roller_count):
            t = (k + 1) / (roller_count + 1)
            if orientation == "X":
                rx = cx - length * 0.42 + t * length * 0.84
                roller = pv.Cylinder(center=(rx, cy, bt - 18), direction=(0, 1, 0), radius=10, height=width - 42)
            else:
                ry = cy - width * 0.42 + t * width * 0.84
                roller = pv.Cylinder(center=(cx, ry, bt - 18), direction=(1, 0, 0), radius=10, height=length - 42)
            detail_actors.append(self.plotter.add_mesh(roller, color="#b2bec3", smooth_shading=True))

        # 5. Motor-redüktör bloğu ve acil stop/sensör görselleri
        if orientation == "X":
            motor_center = (cx + length / 2 + 48, cy - width * 0.32, bt + 18)
            sensor_base = (cx - length * 0.12, cy + width / 2 + 32, bt + 65)
            arrow_centers = [(cx + shift, cy, bt + 9) for shift in (-length * 0.22, 0, length * 0.22)]
            arrow_dir = (-1, 0, 0)
        else:
            motor_center = (cx - length * 0.32, cy + width / 2 + 48, bt + 18)
            sensor_base = (cx + length / 2 + 32, cy, bt + 65)
            arrow_centers = [(cx, cy + shift, bt + 9) for shift in (-width * 0.22, 0, width * 0.22)]
            arrow_dir = (0, 1 if cy >= 0 else -1, 0)

        motor = pv.Cube(center=motor_center, x_length=72, y_length=58, z_length=54)
        detail_actors.append(self.plotter.add_mesh(motor, color="#636e72", smooth_shading=True))

        post = pv.Cylinder(center=(sensor_base[0], sensor_base[1], sensor_base[2] - 35), direction=(0, 0, 1), radius=8, height=70)
        eye = pv.Sphere(center=sensor_base, radius=18)
        detail_actors.append(self.plotter.add_mesh(post, color="#2d3436", smooth_shading=True))
        detail_actors.append(self.plotter.add_mesh(eye, color="#00d2d3", opacity=0.7, smooth_shading=True))

        estop = pv.Sphere(center=(motor_center[0], motor_center[1], motor_center[2] + 38), radius=18)
        detail_actors.append(self.plotter.add_mesh(estop, color="#d63031", smooth_shading=True))

        for ac in arrow_centers:
            shaft = pv.Cylinder(center=ac, direction=arrow_dir, radius=4, height=54)
            head = pv.Cone(center=(ac[0] + arrow_dir[0] * 36, ac[1] + arrow_dir[1] * 36, ac[2]), direction=arrow_dir, height=30, radius=14)
            detail_actors.append(self.plotter.add_mesh(shaft, color="#f9ca24"))
            detail_actors.append(self.plotter.add_mesh(head, color="#f9ca24"))

        self.factory_actors.extend(detail_actors)

    def build_tool_changer_station(self):
        if not hasattr(self, 'factory_actors'):
            self.factory_actors = []

        actors = []
        base = pv.Cube(center=(-650, -720, 70), x_length=360, y_length=150, z_length=140)
        rail = pv.Cube(center=(-650, -720, 155), x_length=400, y_length=28, z_length=22)
        actors.append(self.plotter.add_mesh(base, color="#2d3436", smooth_shading=True))
        actors.append(self.plotter.add_mesh(rail, color="#b2bec3", smooth_shading=True))

        tool_specs = [
            (-760, "#1abc9c", "vac"),
            (-650, "#e74c3c", "laser"),
            (-540, "#f1c40f", "mill"),
        ]
        for x, color, _name in tool_specs:
            socket = pv.Cylinder(center=(x, -720, 190), direction=(0, 0, 1), radius=30, height=35)
            body = pv.Cylinder(center=(x, -720, 245), direction=(0, 0, 1), radius=18, height=80)
            actors.append(self.plotter.add_mesh(socket, color="#111111", smooth_shading=True))
            actors.append(self.plotter.add_mesh(body, color=color, smooth_shading=True))

        beacon_pole = pv.Cylinder(center=(-850, -720, 250), direction=(0, 0, 1), radius=7, height=220)
        beacon = pv.Sphere(center=(-850, -720, 375), radius=24)
        actors.append(self.plotter.add_mesh(beacon_pole, color="#576574", smooth_shading=True))
        actors.append(self.plotter.add_mesh(beacon, color="#2ecc71", opacity=0.75, smooth_shading=True))

        self.factory_actors.extend(actors)

    def build_laser_process_station(self):
        if not hasattr(self, 'factory_actors'):
            self.factory_actors = []

        actors = []
        table = pv.Cube(center=(640, 640, 105), x_length=430, y_length=330, z_length=42)
        plate = pv.Cube(center=(640, 640, 138), x_length=330, y_length=230, z_length=10)
        actors.append(self.plotter.add_mesh(table, color="#3d3d3d", smooth_shading=True))
        actors.append(self.plotter.add_mesh(plate, color="#95a5a6", smooth_shading=True))

        for sx in (-170, 170):
            for sy in (-125, 125):
                leg = pv.Cylinder(center=(640 + sx, 640 + sy, 45), direction=(0, 0, 1), radius=9, height=120)
                actors.append(self.plotter.add_mesh(leg, color="#596275", smooth_shading=True))

        guard_back = pv.Cube(center=(640, 825, 280), x_length=470, y_length=18, z_length=270)
        guard_left = pv.Cube(center=(405, 640, 280), x_length=18, y_length=330, z_length=270)
        guard_right = pv.Cube(center=(875, 640, 280), x_length=18, y_length=330, z_length=270)
        actors.append(self.plotter.add_mesh(guard_back, color="#1e272e", opacity=0.35))
        actors.append(self.plotter.add_mesh(guard_left, color="#1e272e", opacity=0.35))
        actors.append(self.plotter.add_mesh(guard_right, color="#1e272e", opacity=0.35))

        for offset in (-70, 0, 70):
            line = pv.Cube(center=(640 + offset, 640, 145), x_length=7, y_length=205, z_length=4)
            actors.append(self.plotter.add_mesh(line, color="#ff3838", opacity=0.85))
        cross = pv.Cube(center=(640, 640, 147), x_length=250, y_length=7, z_length=4)
        actors.append(self.plotter.add_mesh(cross, color="#ff3838", opacity=0.85))

        self.factory_actors.extend(actors)

    def build_reject_station(self):
        if not hasattr(self, 'factory_actors'):
            self.factory_actors = []

        actors = []
        rx = getattr(self, "reject_x", -800.0)
        ry = getattr(self, "reject_y", 0.0)

        bin_base = pv.Cube(center=(rx, ry, 95), x_length=300, y_length=300, z_length=110)
        actors.append(self.plotter.add_mesh(bin_base, color="#1b1b1b", smooth_shading=True))

        wall_specs = [
            ((rx, ry - 158, 185), 320, 18, 180),
            ((rx, ry + 158, 185), 320, 18, 180),
            ((rx - 158, ry, 185), 18, 320, 180),
            ((rx + 158, ry, 185), 18, 320, 180),
        ]
        for center, sx, sy, sz in wall_specs:
            wall = pv.Cube(center=center, x_length=sx, y_length=sy, z_length=sz)
            actors.append(self.plotter.add_mesh(wall, color="#7f1d1d", opacity=0.82, smooth_shading=True))

        lip = pv.Cube(center=(rx, ry, 285), x_length=340, y_length=340, z_length=18)
        actors.append(self.plotter.add_mesh(lip, color="#111111", smooth_shading=True))

        for x in (rx - 95, rx, rx + 95):
            stripe = pv.Cube(center=(x, ry - 172, 300), x_length=42, y_length=10, z_length=14)
            actors.append(self.plotter.add_mesh(stripe, color="#ff3838", smooth_shading=True))

        beacon = pv.Sphere(center=(rx - 180, ry - 170, 375), radius=22)
        pole = pv.Cylinder(center=(rx - 180, ry - 170, 315), direction=(0, 0, 1), radius=6, height=110)
        actors.append(self.plotter.add_mesh(pole, color="#2d3436", smooth_shading=True))
        actors.append(self.plotter.add_mesh(beacon, color="#ff0000", opacity=0.75, smooth_shading=True))

        self.factory_actors.extend(actors)

    def next_automation_step(self):
        if not self.automation_active: return

        # --- STATE: SPAWN_BOX ---
        if self.auto_state == "SPAWN_BOX":
            product_kind = np.random.choice(["SMALL_OK", "LARGE_OK", "DEFECT"], p=[0.42, 0.42, 0.16])
            if product_kind == "SMALL_OK":
                self.box_current_size = 40
                self.box_type = "KÜÇÜK"
                self.box_quality = "SAĞLAM"
                self.box_route = "LEFT"
            elif product_kind == "LARGE_OK":
                self.box_current_size = 90
                self.box_type = "BÜYÜK"
                self.box_quality = "SAĞLAM"
                self.box_route = "RIGHT"
            else:
                self.box_current_size = 65
                self.box_type = "HATALI"
                self.box_quality = "HATALI"
                self.box_route = "REJECT"
            
            # Reset scaling safely to prevent compounding vector scales
            self.box_actor.SetScale(1, 1, 1) 
            self.box_actor.SetScale(self.box_current_size, self.box_current_size, self.box_current_size)
            self.box_actor.SetPosition(1000, 0, 155 + self.box_current_size / 2) # Start far right on intake belt
            
            # FIXED: VTK requires normalized RGB values (0.0 - 1.0) instead of a hex string
            if self.box_quality == "HATALI":
                self.box_actor.GetProperty().SetColor(0.88, 0.08, 0.10)  # Reject red
            elif self.box_type == "KÜÇÜK":
                self.box_actor.GetProperty().SetColor(0.204, 0.596, 0.859)  # #3498db (Blue)
            else:
                self.box_actor.GetProperty().SetColor(0.608, 0.349, 0.714)  # #9b59b6 (Purple)
                
            self.box_actor.SetVisibility(True)
            self.auto_state = "CONVEYOR_IN"
            self.mark_cycle_started()
            # Control is now fully handed over to physics_tick() to move the conveyor belt

        # --- STATE: APPROACH ---
        elif self.auto_state == "APPROACH":
            self.auto_state = "PICK"
            self.dashboard_last_event = "Robot kutuya yaklaşıyor"
            # Descend precisely onto the calculated top surface of the moving box
            self.solve_ik_and_move([550, 0, 155 + self.box_current_size], animate=True)

        # --- STATE: PICK ---
        elif self.auto_state == "PICK":
            self.is_holding_box = True
            self.dashboard_last_event = "Vakum kavrama aktif"
            
            # Lock the coordinate transform relationship exactly at the split-second of the pick
            T_tcp = get_fk_matrix(self.current_angles)
            T_box = np.eye(4)
            T_box[:3, 3] = self.box_actor.GetPosition()
            self.grip_offset = np.linalg.inv(T_tcp) @ T_box

            self.auto_state = "LIFT"
            self.solve_ik_and_move([550, 0, 400], animate=True)

        # --- STATE: LIFT ---
        elif self.auto_state == "LIFT":
            if self.box_route == "REJECT":
                self.auto_state = "REJECT_PREPARE"
                self.dashboard_last_event = "Hatalı ürün reject tezgahına güvenli rota ile bırakılıyor"
                self.set_status("KALİTE: Reject için güvenli yüksek rota hazırlanıyor", "#e67e22")
                self.solve_ik_and_move([550, 0, 720], animate=True, max_iter_override=70, acceptance_mm=90.0)
            else:
                self.auto_state = "DROP"
                self.dashboard_last_event = "Kutu hedef banda taşınıyor"
                target_y = -450 if self.box_route == "LEFT" else 450
                self.solve_ik_and_move([250, target_y, 350], animate=True)

        # --- REJECT SAFE ROUTE: high lift -> turn around -> hover -> drop -> lift -> home ---
        elif self.auto_state == "REJECT_PREPARE":
            self.auto_state = "REJECT_TURN"
            reject_turn_pose = [160.0, -35.0, 55.0, 0.0, 50.0, 0.0]
            self.dashboard_last_event = "Reject rotası: robot arkasını dönüyor"
            self.animate_to(reject_turn_pose, duration_ms=900)

        elif self.auto_state == "REJECT_TURN":
            self.auto_state = "REJECT_APPROACH"
            self.dashboard_last_event = "Reject rotası: tezgah üstüne güvenli yaklaşma"
            reject_above_pose = [160.0, -48.0, 78.0, 0.0, 32.0, 0.0]
            self.animate_to(reject_above_pose, duration_ms=780)

        elif self.auto_state == "REJECT_APPROACH":
            self.auto_state = "REJECT_DROP"
            self.dashboard_last_event = "Reject rotası: kontrollü aşağı iniş"
            reject_drop_pose = [160.0, -30.0, 42.0, 0.0, 62.0, 0.0]
            self.animate_to(reject_drop_pose, duration_ms=620)

        elif self.auto_state == "REJECT_DROP":
            self.is_holding_box = False
            self.dashboard_last_event = "Hatalı ürün ayrıldı"
            if hasattr(self, 'box_actor') and self.box_actor is not None:
                self.box_actor.SetPosition(self.reject_x, self.reject_y, self.reject_table_z + self.box_current_size / 2)
            self.mark_product_completed()
            self.auto_state = "REJECT_LIFT"
            self.set_status("KALİTE: Hatalı ürün reject tezgahına bırakıldı", "#e74c3c")
            reject_lift_pose = [160.0, -42.0, 68.0, 0.0, 42.0, 0.0]
            self.animate_to(reject_lift_pose, duration_ms=620)

        elif self.auto_state == "REJECT_LIFT":
            self.auto_state = "HOME"
            self.dashboard_last_event = "Reject rotası tamamlandı, home pozisyonuna dönüyor"
            self.animate_to([0.0] * 6, duration_ms=800)

        elif self.auto_state == "HOME":
            if self.box_route == "REJECT" and hasattr(self, 'box_actor') and self.box_actor is not None:
                self.box_actor.SetVisibility(False)
            self.auto_state = "SPAWN_BOX"
            self.next_automation_step()

        # --- STATE: DROP ---
        elif self.auto_state == "DROP":
            self.is_holding_box = False   # Deactivate vacuum effector
            self.drop_velocity = 0.0      # Reset gravity step accumulator
            self.auto_state = "RELEASE"   # Pass control to physics_tick for gravity fall
            self.dashboard_last_event = "Vakum bırakıldı"
            
            # Send the robot back to its home configuration smoothly while the object drops
            self.animate_to([0.0] * 6, duration_ms=800)

    def physics_tick(self):
        """Continuous state machine, physics updates, and conveyor translations."""
        if not getattr(self, 'automation_active', False): 
            return

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        belt_speed = 390.0 # mm/s - daha gerçekçi otomasyon temposu
        gravity = 9810.0   # mm/s^2 (Standard 9.81 m/s^2 scaled to mm)
        
        # --- HANDLING INBOUND CONVEYOR ---
        if self.auto_state == "CONVEYOR_IN":
            if hasattr(self, 'box_actor') and self.box_actor is not None:
                curr_pos = self.box_actor.GetPosition()
                new_x = curr_pos[0] - (belt_speed * dt)
                self.box_actor.SetPosition(new_x, curr_pos[1], curr_pos[2])
                
                if new_x <= 550:
                    self.auto_state = "APPROACH"
                    self.dashboard_last_event = "Sensör kutuyu algıladı"
                    self.set_status("SENSÖR: Kutu Ayrıştırma Noktasında!", "#f39c12")
                    self.next_automation_step()

        # --- HANDLING KINEMATIC GRAVITY DROP ---
        elif self.auto_state == "RELEASE":
            if hasattr(self, 'box_actor') and self.box_actor is not None:
                curr_pos = self.box_actor.GetPosition()
                
                # Apply velocity acceleration step over time delta
                self.drop_velocity += gravity * dt
                new_z = curr_pos[2] - (self.drop_velocity * dt)
                
                # Floor collision profile limit (the top surface altitude of the exit belts)
                target_floor = 155 + self.box_current_size / 2
                if new_z <= target_floor:
                    new_z = target_floor
                    self.drop_velocity = 0.0
                    if self.box_route == "REJECT":
                        self.auto_state = "REJECT_WAIT"
                        self.dashboard_last_event = "Hatalı ürün ayrıldı"
                        self.mark_product_completed()
                        self.set_status("KALİTE: Hatalı ürün reject alanına ayrıldı", "#e74c3c")
                    else:
                        self.auto_state = "CONVEYOR_OUT" # Settle onto conveyor
                        self.dashboard_last_event = "Kutu çıkış bandına indi"
                        self.set_status("FİZİK: Kutu Çıkış Bandına Yerleşti", "#2ecc71")
                    
                self.box_actor.SetPosition(curr_pos[0], curr_pos[1], new_z)

        # --- HANDLING OUTBOUND CONVEYOR ---
        elif self.auto_state == "CONVEYOR_OUT":
            if hasattr(self, 'box_actor') and self.box_actor is not None:
                curr_pos = self.box_actor.GetPosition()
                direction = -1 if self.box_route == "LEFT" else 1
                new_y = curr_pos[1] + (belt_speed * dt * direction)
                self.box_actor.SetPosition(curr_pos[0], new_y, curr_pos[2])
                
                # Out-of-bounds cleanup zone check
                if abs(new_y) > 950:
                    self.box_actor.SetVisibility(False)
                    self.mark_product_completed()
                    
                    # Wait for robot to finish returning home before spawning next loop item
                    if not self.anim_timer.isActive():
                        self.auto_state = "SPAWN_BOX"
                        self.next_automation_step()

        elif self.auto_state == "REJECT_WAIT":
            if not self.anim_timer.isActive():
                if hasattr(self, 'box_actor') and self.box_actor is not None:
                    self.box_actor.SetVisibility(False)
                self.auto_state = "SPAWN_BOX"
                self.next_automation_step()

        self.plotter.render()


    def cleanup_resources(self):
        """Uygulama kapanırken kamera, mikrofon ve timer kaynaklarını kesin serbest bırakır."""
        if getattr(self, "_closing", False):
            return
        self._closing = True
        try:
            self.stop_voice_control()
        except Exception:
            pass
        try:
            self.stop_hand_tracking()
        except Exception:
            pass
        try:
            if hasattr(self, 'physics_timer') and self.physics_timer.isActive():
                self.physics_timer.stop()
        except Exception:
            pass
        try:
            if self.anim_timer.isActive():
                self.anim_timer.stop()
        except Exception:
            pass
        try:
            if self.drag_timer.isActive():
                self.drag_timer.stop()
        except Exception:
            pass
        try:
            if hasattr(self, 'dashboard_timer') and self.dashboard_timer.isActive():
                self.dashboard_timer.stop()
        except Exception:
            pass

    def closeEvent(self, event):
        """Pencere kapanınca worker process'i de kapanır; webcam ışığı açık kalmaz."""
        self.cleanup_resources()
        event.accept()


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(sys.argv)
    window = RoboSimUI()
    window.show()
    sys.exit(app.exec_())
