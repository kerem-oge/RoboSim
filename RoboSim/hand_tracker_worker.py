"""
RoboSim SmartCell kamera worker s?reci.

OpenCV ve MediaPipe el takibi ana PyQt5/PyVista uygulamas?ndan ayr?
?al??t?r?l?r. Worker, kamera durumunu ve el koordinatlar?n? JSON sat?rlar?
olarak ana uygulamaya g?nderir; kapan??ta kamera kayna??n? serbest b?rak?r.
"""

import atexit
import base64
import json
import os
import signal
import subprocess
import sys
import time

os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

import cv2

_stop_requested = False
_cap = None
PID_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_tracker_worker.pid")
STOP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_tracker_worker.stop")


def emit(payload):
    print(json.dumps(payload, separators=(",", ":")), flush=True)


def write_pidfile():
    try:
        with open(PID_FILE, "w", encoding="utf-8") as f:
            f.write(str(os.getpid()))
    except Exception:
        pass


def remove_pidfile():
    try:
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
    except Exception:
        pass


def remove_stopfile():
    try:
        if os.path.exists(STOP_FILE):
            os.remove(STOP_FILE)
    except Exception:
        pass


def request_stop(signum=None, frame=None):
    global _stop_requested
    _stop_requested = True


def kill_previous_worker_from_pidfile():
    try:
        if not os.path.exists(PID_FILE):
            return
        with open(PID_FILE, "r", encoding="utf-8") as f:
            pid = int(f.read().strip())
    except Exception:
        remove_pidfile()
        return

    if pid <= 0 or pid == os.getpid():
        return

    try:
        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(pid)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2,
            )
        else:
            os.kill(pid, signal.SIGTERM)
    except Exception:
        pass

    time.sleep(0.4)
    remove_pidfile()


def release_camera():
    global _cap
    try:
        if _cap is not None:
            _cap.release()
            _cap = None
    except Exception:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    remove_stopfile()
    remove_pidfile()


def make_openers(camera_index):
    openers = [
        ("DSHOW", lambda: cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)),
        ("DSHOW_OFFSET", lambda: cv2.VideoCapture(camera_index + cv2.CAP_DSHOW)),
    ]
    if hasattr(cv2, "CAP_MSMF"):
        openers.extend([
            ("MSMF", lambda: cv2.VideoCapture(camera_index, cv2.CAP_MSMF)),
            ("MSMF_OFFSET", lambda: cv2.VideoCapture(camera_index + cv2.CAP_MSMF)),
        ])
    openers.extend([
        ("ANY", lambda: cv2.VideoCapture(camera_index, cv2.CAP_ANY)),
        ("DEFAULT", lambda: cv2.VideoCapture(camera_index)),
    ])
    return openers


def apply_camera_profile(cap, profile):
    fourcc, width, height, fps = profile
    try:
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            cap.set(cv2.CAP_PROP_FPS, fps)
        if hasattr(cv2, "CAP_PROP_CONVERT_RGB"):
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass


def try_open_camera(camera_index):
    openers = make_openers(camera_index)
    profiles = [
        (None, None, None, None),
        ("MJPG", 640, 480, 30),
        ("MJPG", 1280, 720, 30),
        ("YUY2", 640, 480, 30),
        ("YUY2", 320, 240, 30),
        (None, 640, 480, 30),
    ]

    errors = []
    for attempt in range(3):
        if attempt:
            time.sleep(1.0)
        for backend_name, opener in openers:
            for profile in profiles:
                cap = None
                profile_name = "native" if profile[0] is None and profile[1] is None else f"{profile[0] or 'AUTO'} {profile[1]}x{profile[2]}@{profile[3]}"
                try:
                    cap = opener()
                    if not cap.isOpened():
                        errors.append(f"deneme {attempt + 1} Kamera {camera_index} / {backend_name} / {profile_name}: açılamadı")
                        cap.release()
                        continue

                    apply_camera_profile(cap, profile)
                    time.sleep(0.15)

                    ok = False
                    frame = None
                    for _ in range(40):
                        ok, frame = cap.read()
                        if ok and frame is not None and getattr(frame, "size", 0) > 0:
                            break
                        try:
                            if cap.grab():
                                ok, frame = cap.retrieve()
                                if ok and frame is not None and getattr(frame, "size", 0) > 0:
                                    break
                        except Exception:
                            pass
                        time.sleep(0.08)

                    if not ok or frame is None or getattr(frame, "size", 0) == 0:
                        errors.append(f"deneme {attempt + 1} Kamera {camera_index} / {backend_name} / {profile_name}: kare okunamadı")
                        cap.release()
                        time.sleep(0.15)
                        continue

                    return cap, f"{backend_name} / {profile_name}", errors
                except Exception as exc:
                    errors.append(f"deneme {attempt + 1} Kamera {camera_index} / {backend_name} / {profile_name}: {type(exc).__name__} - {exc}")
                    if cap is not None:
                        cap.release()
                    time.sleep(0.15)

    return None, None, errors


def main():
    global _cap, _stop_requested

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    atexit.register(release_camera)
    kill_previous_worker_from_pidfile()
    remove_stopfile()
    write_pidfile()

    camera_index = 0
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except Exception:
            camera_index = 0

    _cap, backend_name, open_errors = try_open_camera(camera_index)
    if _cap is None:
        emit({
            "type": "status",
            "level": "error",
            "message": (
                f"KAMERA HATASI: Kamera {camera_index} açılamadı. Kamera başka process tarafından kullanılıyor olabilir. "
                + " | ".join(open_errors[-20:])
            ),
        })
        return

    try:
        import mediapipe as mp
    except Exception as exc:
        emit({
            "type": "status",
            "level": "error",
            "message": f"MEDIAPIPE HATASI: {type(exc).__name__} - {exc}",
        })
        return

    try:
        _cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    mp_styles = mp.solutions.drawing_styles
    frame_count = 0
    detected_count = 0
    last_emit = 0.0

    emit({"type": "status", "level": "ok", "message": f"KAMERA + MEDIAPIPE AKTİF - Kamera {camera_index} / {backend_name}"})

    try:
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.35,
            min_tracking_confidence=0.35,
        ) as hands:
            while not _stop_requested and not os.path.exists(STOP_FILE):
                ok, frame = _cap.read()
                if not ok or frame is None:
                    emit({"type": "hand", "hand_visible": False, "frame": frame_count, "detected": detected_count})
                    time.sleep(0.05)
                    continue

                frame_count += 1
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                results = hands.process(rgb)
                rgb.flags.writeable = True

                payload = {
                    "type": "hand",
                    "hand_visible": False,
                    "frame": frame_count,
                    "detected": detected_count,
                }

                if results.multi_hand_landmarks:
                    hand_landmarks = results.multi_hand_landmarks[0]
                    landmarks = hand_landmarks.landmark
                    xs = [lm.x for lm in landmarks]
                    ys = [lm.y for lm in landmarks]
                    palm_ids = [0, 5, 9, 13, 17]
                    cx = sum(landmarks[i].x for i in palm_ids) / len(palm_ids)
                    cy = sum(landmarks[i].y for i in palm_ids) / len(palm_ids)
                    hand_size = max(max(xs) - min(xs), max(ys) - min(ys))
                    detected_count += 1

                    payload.update({
                        "hand_visible": True,
                        "cx": float(cx),
                        "cy": float(cy),
                        "size": float(hand_size),
                        "detected": detected_count,
                    })

                    h, w = frame.shape[:2]
                    cv2.circle(frame, (int(cx * w), int(cy * h)), 8, (0, 255, 0), -1)
                    cv2.putText(frame, "EL VAR", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    mp_draw.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_styles.get_default_hand_landmarks_style(),
                        mp_styles.get_default_hand_connections_style(),
                    )
                else:
                    cv2.putText(frame, "EL GORULMEDI", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (60, 120, 255), 2)

                now = time.time()
                if now - last_emit >= 1.0 / 15.0:
                    preview = cv2.resize(frame, (320, 210), interpolation=cv2.INTER_AREA)
                    ok, encoded = cv2.imencode(".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    if ok:
                        payload["frame_b64"] = base64.b64encode(encoded).decode("ascii")
                    emit(payload)
                    last_emit = now
    finally:
        release_camera()
        emit({"type": "status", "level": "ok", "message": "KAMERA KAPATILDI"})


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        release_camera()
    except Exception as exc:
        release_camera()
        emit({"type": "status", "level": "error", "message": f"EL TAKİBİ HATASI: {type(exc).__name__} - {exc}"})
