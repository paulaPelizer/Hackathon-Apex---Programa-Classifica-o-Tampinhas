import cv2
import numpy as np
import csv
import time
from collections import deque

# Configs
CAM_INDEX = 0
ROI_RATIO = (0.28, 0.18)
STABILITY_FRAMES = 10
MIN_AREA = 800
MAX_AREA = 90000
CIRCULARITY_MIN = 0.60
CSV_PATH = "reads.csv"
WINDOW = "Leitor de Tampinhas – Disparo Único"

# Envia sinal (continua em circuito) - inicio a partir de sinal de entrada do circuito
SINGLE_SHOT = True

# Delay da mensagem antes de fechar
POST_SEND_MS = 1200

#Sinal para servo
SERIAL_ENABLED = False        # (Arduido)
SERIAL_PORT = "COM3"          
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 2.0

# Mapeamento das cores / ângulos do servo (cinco posições iniciais entre 0º e 180º)
COLOR_ORDER = ["vermelho","verde","branco","azul","outros"]
ANGLE_MAP = {
    "vermelho": 0,
    "verde": 45,
    "branco": 90,
    "azul": 135,
    "outros": 180,
}

# HSV color ranges (OpenCV) - Preciso ajustar conforme a iluminação do LED
COLOR_RANGES = {
    "vermelho": [((0, 120, 60), (10, 255, 255)), ((170, 120, 60), (179, 255, 255))],
    "verde":    [((36,  70, 40), (85, 255, 255))],
    "branco":  [((20, 120, 80), (35, 255, 255))],
    "azul":     [((90,  80, 60), (130,255, 255))],
    "outros":     [((135, 80, 80), (170,255, 255))],
}

# Serial 
_ser = None
def serial_try_open():
    
    #time.sleep(2)
    global _ser
    if not SERIAL_ENABLED:
        return None
    try:
        import serial  # pyserial
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
        return _ser
    except Exception as e:
        print(f"[!] Erro ao abrir a serial ({SERIAL_PORT}): {e}")
        _ser = None
        return None

def serial_send(angle, color):
    if _ser is None:
        return False
    try:
        msg = f"SERVO:{angle};COLOR:{color}\n"
        _ser.write(msg.encode("utf-8"))
        return True
    except Exception as e:
        print(f"[!] Falha ao enviar para serial: {e}")
        return False

# Dados
def ensure_csv(path):
    try:
        with open(path, "x", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["dia-hora", "cor", "area_px", "diametro_px", "angulo_servo", "id_cor"])
    except FileExistsError:
        pass

def make_trackbars():
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H low",  WINDOW, 0,   179, lambda v: None)
    cv2.createTrackbar("H high", WINDOW, 179, 179, lambda v: None)
    cv2.createTrackbar("S low",  WINDOW, 40,  255, lambda v: None)
    cv2.createTrackbar("S high", WINDOW, 255, 255, lambda v: None)
    cv2.createTrackbar("V low",  WINDOW, 40,  255, lambda v: None)
    cv2.createTrackbar("V high", WINDOW, 255, 255, lambda v: None)

    cv2.createTrackbar("Min area", WINDOW, MIN_AREA, 200000, lambda v: None)
    cv2.createTrackbar("Max area", WINDOW, MAX_AREA, 300000, lambda v: None)
    cv2.createTrackbar("Circularity x100", WINDOW, int(CIRCULARITY_MIN*100), 100, lambda v: None)

def get_trackbar_values():
    vals = {}
    vals["hL"] = cv2.getTrackbarPos("H low", WINDOW)
    vals["hH"] = cv2.getTrackbarPos("H high", WINDOW)
    vals["sL"] = cv2.getTrackbarPos("S low", WINDOW)
    vals["sH"] = cv2.getTrackbarPos("S high", WINDOW)
    vals["vL"] = cv2.getTrackbarPos("V low", WINDOW)
    vals["vH"] = cv2.getTrackbarPos("V high", WINDOW)
    vals["min_area"] = cv2.getTrackbarPos("Min area", WINDOW)
    vals["max_area"] = cv2.getTrackbarPos("Max area", WINDOW)
    vals["circ_min"] = cv2.getTrackbarPos("Circularity x100", WINDOW)/100.0
    return vals

def color_name_from_hsv(hsv_crop, mask_obj):
    best = ("desconhecida", 0)
    for name, ranges in COLOR_RANGES.items():
        total = np.zeros(mask_obj.shape, dtype=np.uint8)
        for (lo, hi) in ranges:
            m = cv2.inRange(hsv_crop, np.array(lo, np.uint8), np.array(hi, np.uint8))
            total = cv2.bitwise_or(total, m)
        hit = cv2.countNonZero(cv2.bitwise_and(total, mask_obj))
        if hit > best[1]:
            best = (name, hit)
    return best[0]

def contour_circularity(cnt):
    area = cv2.contourArea(cnt)
    per  = cv2.arcLength(cnt, True)
    if per == 0:
        return 0.0
    return 4*np.pi*area/(per*per)

def draw_transparent_rect(frame, pt1, pt2, color=(60,60,60), alpha=0.20):
    overlay = frame.copy()
    cv2.rectangle(overlay, pt1, pt2, color, -1)
    return cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

# Programa
def main():
    ensure_csv(CSV_PATH)
    ser = serial_try_open()

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Não consegui abrir a câmera.")

    make_trackbars()
    centers = deque(maxlen=STABILITY_FRAMES)
    last_saved = 0

    print("[i] Uma tampinha por vez na ROI. Disparo único ativado. [q]=sair")
    sent_once = False

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        h, w = frame.shape[:2]
        roi_w = int(w * ROI_RATIO[0])
        roi_h = int(h * ROI_RATIO[1])
        cx, cy = w // 2, int(h * 0.55)
        x1, y1 = cx - roi_w // 2, cy - roi_h // 2
        x2, y2 = x1 + roi_w, y1 + roi_h

        vals = get_trackbar_values()
        min_area = max(50, vals["min_area"])
        max_area = max(min_area+1, vals["max_area"])
        circ_min = max(0.10, vals["circ_min"])

        roi = frame[y1:y2, x1:x2].copy()
        blur = cv2.GaussianBlur(roi, (5,5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 120)
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidate = None
        best_area = 0
        for cnt in contours:
            a = cv2.contourArea(cnt)
            if a < min_area or a > max_area:
                continue
            circ = contour_circularity(cnt)
            if circ < circ_min:
                continue
            if a > best_area:
                best_area = a
                candidate = cnt

        vis = draw_transparent_rect(frame, (x1, y1), (x2, y2), color=(0, 180, 0), alpha=0.15)
        color_name = None
        diameter_px = None
        ready = False

        if candidate is not None:
            (cx_obj, cy_obj), radius = cv2.minEnclosingCircle(candidate)
            center = (int(cx_obj), int(cy_obj))

            obj_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
            cv2.drawContours(obj_mask, [candidate], -1, 255, -1)
            color_name = color_name_from_hsv(hsv, obj_mask)

            centers.append(center)
            if len(centers) == STABILITY_FRAMES:
                std = np.std(np.array(centers), axis=0)
                if std[0] < 2.5 and std[1] < 2.5:
                    ready = True
            else:
                ready = False

            diameter_px = 2*radius
            # draw
            cg = (int(cx_obj)+x1, int(cy_obj)+y1)
            cv2.circle(vis, cg, int(radius), (0, 255, 255), 2)

            label = f"{color_name or '??'} | area:{int(best_area)} | dpx:{int(diameter_px or 0)}"
            cv2.putText(vis, label, (x1+8, y2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(vis, label, (x1+8, y2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        else:
            centers.clear()

        # Disparo único
        if SINGLE_SHOT and ready and (not sent_once) and (color_name in ANGLE_MAP):
            angle = ANGLE_MAP[color_name]
            color_index = COLOR_ORDER.index(color_name) + 1  # 1..5
            # CSV
            with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
                wcsv = csv.writer(f)
                wcsv.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), color_name, int(best_area), int(diameter_px), angle, color_index])
            # Serial
            ok_send = serial_send(angle, color_name)
            sent_once = True
            msg = f"LEITURA OK ✔  cor={color_name}  angle={angle}°  (serial={'ok' if ok_send else 'off'})"
            cv2.putText(vis, msg, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 3, cv2.LINE_AA)
            cv2.imshow(WINDOW, vis)
            cv2.waitKey(POST_SEND_MS)
            break

        cv2.imshow(WINDOW, vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    if _ser is not None:
        try:
            _ser.close()
        except:
            pass
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
