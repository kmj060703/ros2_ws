# 192.168.0.23

from ultralytics import YOLO
import cv2

import time
import socket

alert_mode = False
last_blink_time = 0
blink_visible = True
BLINK_INTERVAL = 500

UDP_IP = "127.0.0.1"
UDP_PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

time.sleep(2)

model = YOLO("src/best.pt")

cap = cv2.VideoCapture(0) # 여기에 UDP 수신한 영상받기


while True:
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.grab()

    ret, frame = cap.read()
    if not ret:
        continue

    # YOLO 객체 감지
    results = model(frame)
    person_count = 0
    
    MIN_BOX_AREA = 3000  
    
    for r in results:
        for box in r.boxes:
            x, y, w, h = box.xywh[0].tolist()
            area = w * h
            if area > MIN_BOX_AREA:
                person_count += 1

    # UDP 영상 송출
   

    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sock.close()

