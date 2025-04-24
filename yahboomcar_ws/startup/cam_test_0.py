#!/usr/bin/env python3

import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("[?] Cannot open /dev/video0")
    exit()

print("[?] /dev/video0 is active. Press 'q' to quit.")
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
