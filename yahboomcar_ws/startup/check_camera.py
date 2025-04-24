#!/usr/bin/env python3

import cv2

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[?] Cannot open /dev/video0")
        return

    print("[?] Camera opened on /dev/video0. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[??] Frame capture failed.")
            break
        cv2.imshow("Camera Test (/dev/video0)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

