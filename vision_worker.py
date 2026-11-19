"""
vision_worker.py — รอภาพใหม่จาก Firebase -> Detect ยางลบด้วย best.pt -> เขียนผลกลับ Firebase

ล็อกอินด้วย email/password เดียวกับที่ใช้เข้า dashboard (ไม่ต้องใช้ serviceAccountKey.json)

ติดตั้ง:  python -m pip install ultralytics opencv-python numpy requests
ต้องมี:   best.pt
          worker_config.json  (ดูตัวอย่างใน worker_config.example.json — ห้าม push ขึ้น GitHub)
"""
import base64
import json
import time

import cv2
import numpy as np
import requests
from ultralytics import YOLO

CFG = json.load(open("worker_config.json", encoding="utf-8"))
API_KEY = CFG["api_key"]
DB_URL = CFG["db_url"].rstrip("/")
EMAIL = CFG["email"]
PASSWORD = CFG["password"]

MODEL_PATH = "best.pt"
CONF_TH = 0.5
POLL_SEC = 0.2            # เช็กภาพใหม่ทุกกี่วินาที
# class ใน best.pt ของเรา: "black-eraser", "wrapped eraser"
TARGET_CLASSES = {"black-eraser", "wrapped eraser"}

model = YOLO(MODEL_PATH)
http = requests.Session()

# ---------------- Auth (เหมือนใน dashboard.html) ----------------
_tok = {"id": "", "refresh": "", "exp": 0}


def sign_in():
    r = http.post(
        "https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=" + API_KEY,
        json={"email": EMAIL, "password": PASSWORD, "returnSecureToken": True}, timeout=15)
    j = r.json()
    if not r.ok:
        raise SystemExit("ล็อกอินไม่สำเร็จ: " + j.get("error", {}).get("message", str(r.status_code)))
    _tok.update(id=j["idToken"], refresh=j["refreshToken"],
                exp=time.time() + int(j.get("expiresIn", 3600)) - 300)
    print("ล็อกอินสำเร็จ:", EMAIL)


def refresh():
    r = http.post("https://securetoken.googleapis.com/v1/token?key=" + API_KEY,
                  data={"grant_type": "refresh_token", "refresh_token": _tok["refresh"]}, timeout=15)
    if not r.ok:
        return sign_in()
    j = r.json()
    _tok.update(id=j["id_token"], refresh=j["refresh_token"],
                exp=time.time() + int(j.get("expires_in", 3600)) - 300)


def token():
    if not _tok["id"]:
        sign_in()
    elif time.time() > _tok["exp"]:
        refresh()
    return _tok["id"]


def db_get(path):
    r = http.get(f"{DB_URL}{path}.json", params={"auth": token()}, timeout=15)
    if r.status_code == 401:
        _tok["exp"] = 0
        r = http.get(f"{DB_URL}{path}.json", params={"auth": token()}, timeout=15)
    r.raise_for_status()
    return r.json()


def db_put(path, data):
    r = http.put(f"{DB_URL}{path}.json", params={"auth": token()}, json=data, timeout=15)
    if r.status_code == 401:
        _tok["exp"] = 0
        r = http.put(f"{DB_URL}{path}.json", params={"auth": token()}, json=data, timeout=15)
    r.raise_for_status()


# ---------------- Vision ----------------
def cam_key(data):
    """ต้องตรงกับ camKey() ใน dashboard.html: ts:ความยาว:ท้ายภาพ 24 ตัว
    (ts เป็นวินาที ถ่ายต่อเนื่องอาจได้ ts ซ้ำ เลยใช้ ts อย่างเดียวไม่ได้)"""
    img = data["img"]
    ts = data.get("ts")
    if isinstance(ts, float) and ts.is_integer():   # ให้เหมือน JS: 1795069482.0 -> 1795069482
        ts = int(ts)
    return f"{ts}:{len(img)}:{img[-24:]}"


def decode_img(b64: str):
    if b64.startswith("data:"):          # เผื่อมี prefix data:image/jpeg;base64,
        b64 = b64.split(",", 1)[1]
    buf = np.frombuffer(base64.b64decode(b64), np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)   # BGR


def detect(img):
    h, w = img.shape[:2]
    r = model(img, conf=CONF_TH, verbose=False)[0]
    best = None
    for box in r.boxes:
        name = model.names[int(box.cls)]
        conf = float(box.conf)
        if name not in TARGET_CLASSES:
            continue
        if best is None or conf > best["conf"]:
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            best = {
                "label": name,
                "conf": round(conf, 3),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                # ตำแหน่งกลางกล่องแบบ normalize -1..1 (0 = กลางภาพ) ใช้สั่งแขน/หุ่นหมุนได้เลย
                "cx": round(((x1 + x2) / 2 / w) * 2 - 1, 3),
                "cy": round(((y1 + y2) / 2 / h) * 2 - 1, 3),
                # สัดส่วนพื้นที่กล่อง ใช้ประมาณระยะใกล้/ไกล
                "area": round((x2 - x1) * (y2 - y1) / (w * h), 4),
            }
    return best, (w, h), r


def process(data, key):
    t0 = time.time()
    img = decode_img(data["img"])
    if img is None:
        print("decode ไม่ได้")
        return
    best, (w, h), r = detect(img)
    ts = data.get("ts")
    result = {
        "src_ts": ts,
        "src_key": key,                       # ผูกกับภาพไหน
        "found": best is not None,
        "infer_ms": int((time.time() - t0) * 1000),
        "img_w": w,
        "img_h": h,
        "done_at": int(time.time()),
    }
    if best:
        result.update(best)
    db_put("/vision/result", result)
    print(time.strftime("%H:%M:%S"), "FOUND" if best else "-", best or "", f'{result["infer_ms"]}ms')

    # ส่งภาพที่วาดกรอบแล้วกลับไปให้ dashboard โชว์
    ok, jpg = cv2.imencode(".jpg", r.plot(), [cv2.IMWRITE_JPEG_QUALITY, 70])
    if ok:
        db_put("/vision/annotated", {"img": base64.b64encode(jpg).decode(), "ts": ts, "src_key": key})


def main():
    token()
    model(np.zeros((480, 640, 3), np.uint8), verbose=False)   # warm-up: ภาพแรกจะได้ไม่ช้า
    print("รอภาพใหม่ที่ camera/latest ... (Ctrl+C เพื่อหยุด)")
    last_key, last_ts, n = None, object(), 0
    while True:
        try:
            # เช็กแค่ ts ก่อน (เบามาก) ถ้าเปลี่ยนค่อยโหลดภาพเต็ม
            # ts ซ้ำได้ถ้าถ่ายในวินาทีเดียวกัน -> ทุก ๆ 4 รอบโหลดภาพเต็มเช็กเผื่อ
            ts = db_get("/camera/latest/ts")
            n += 1
            if ts != last_ts or n % 4 == 0:
                data = db_get("/camera/latest")
                if data and data.get("img"):
                    last_ts = data.get("ts")
                    key = cam_key(data)
                    if key != last_key:          # ภาพใหม่เท่านั้น
                        last_key = key
                        process(data, key)
        except requests.RequestException as e:
            print("เน็ต/Firebase error:", e)
            time.sleep(2)
        time.sleep(POLL_SEC)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("หยุดแล้ว")
