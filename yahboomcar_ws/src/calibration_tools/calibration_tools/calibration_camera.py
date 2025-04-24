#!/usr/bin/env python3

import cv2

def main():
    for device_id in range(0, 40):
        cap = cv2.VideoCapture(device_id)
        if cap.isOpened():
            print(f"[?] Camera found at /dev/video{device_id}")
            break
        cap.release()
    else:
        print("[?] No camera found on /dev/video0-39")
        return

    print("Press 'q' to quit the test.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[??] Frame capture failed.")
            break
        cv2.imshow(f"Camera /dev/video{device_id}", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
