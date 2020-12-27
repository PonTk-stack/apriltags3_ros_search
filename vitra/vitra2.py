#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
if __name__ == '__main__':
    # KCF
    tracker = cv2.TrackerKCF_create()
    # Boosting
    #tracker = cv2.TrackerBoosting_create()
    # MedianFlow
    # tracker = cv2.TrackerMedianFlow_create()

    cap = cv2.VideoCapture(2)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        bbox = cv2.selectROI(frame, False)
        ok = tracker.init(frame, bbox)
        cv2.destroyAllWindows()
        break
    while True:
        ret, frame = cap.read()
        # トラッカーをアップデートする
        track, bbox = tracker.update(frame)
        if track:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0,255,0), 2, 1)
        else :
            cv2.putText(frame, "Failure", (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA);
        # 加工済の画像を表示する
        cv2.imshow("Tracking", frame)

        # キー入力を1ms待って、k が27（ESC）だったらBreakする
        k = cv2.waitKey(1)
        if k == 27 :
            break

# キャプチャをリリースして、ウィンドウをすべて閉じる
cap.release()
cv2.destroyAllWindows()
