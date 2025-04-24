#!/usr/bin/env python3

import cv2

print("?? Searching for working camera on /dev/video0-39...")

working = False
for device_id in range(0, 40):
    cap = cv2.VideoCapture(device_id)
    if cap.isOpened():
        print(f"[?] Found camera at /dev/video{device_id}")
        working = True
        break
    cap.release()

if not working:
    print("[?] No working camera found.")
    exit()

print("?? Press 'q' to quit the camera stream.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("[??] Frame capture failed.")
        break
    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
