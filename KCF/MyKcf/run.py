import argparse
import cv2
from kcf import Tracker

class DBox():
    selectingObject = False
    initTracking= False
    onTracking= False
    ix, iy, cx, cy = -1, -1, -1, -1
    w, h = 0, 0
    interval = 1
    duration = 1
    def __init__(self,capON = False):
        if(capON):
            self.cap = cv2.VideoCapture(2)
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()
    def draw_boundingbox(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            DBox.selectingObject = True
            DBox.onTracking = False
            DBox.ix, DBox.iy = x, y
            DBox.cx, DBox.cy = x, y

        elif event == cv2.EVENT_MOUSEMOVE:
            DBox.cx, DBox.cy = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            DBox.selectingObject = False
            if(abs(x-DBox.ix)>10 and abs(y-DBox.iy)>10):
                DBox.w, DBox.h = abs(x - DBox.ix), abs(y - DBox.iy)
                DBox.ix, DBox.iy = min(x, DBox.ix), min(y, DBox.iy)
                DBox.initTracking = True
            else:
                DBox.onTracking = False

        elif event == cv2.EVENT_RBUTTONDOWN:
            DBox.onTracking = False
            if(w>0):
                DBox.ix, DBox.iy = x-DBox.w/2, y-DBox.h/2
                DBox.initTracking = True
    def callback(self):
        cv2.namedWindow('tracking')
        cv2.setMouseCallback('tracking',self.draw_boundingbox)

    def test(self):
        kcf = KCF()
        self.callback()
        while(self.cap.isOpened()):
            ret, frame = self.cap.read()

            if(DBox.selectingObject):
                cv2.rectangle(frame,(DBox.ix,DBox.iy), (DBox.cx,DBox.cy), (0,255,255), 1)
            elif(DBox.initTracking):
                cv2.rectangle(frame,(DBox.ix,DBox.iy), (DBox.ix+DBox.w,DBox.iy+DBox.h), (0,255,255), 1)
                roi = DBox.cx, DBox.cy, DBox.w, DBox.h
                kcf.init(frame,roi)
                DBox.initTracking  = False
                DBox.onTracking =True
                #break
            elif(DBox.onTracking):
                kcf.update(frame)

            cv2.imshow('tracking', frame)
            c = cv2.waitKey(DBox.interval) & 0xFF
            if c==27 or c==ord('q'):
                exit(0)
                #break
        if roi:
            return roi
        else:
            print("not roi")
            exit(1)


class KCF():
    def __init__(self):
        self.tracker = Tracker()
    def init(self,frame,roi):
        self.tracker.init(frame, roi)
    def update(self,frame):
        x, y, w, h = self.tracker.update(frame)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 1)
        """
        cv2.imshow('tracking', frame)
        c = cv2.waitKey(1) & 0xFF
        """








if __name__ == '__main__':
    dbox = DBox(capON=True)
    dbox.test()
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("video", help="video you want to track", type=str)
    args = parser.parse_args()
    print(args)
    cap = cv2.VideoCapture(args.video)
    """



    """
    cap = cv2.VideoCapture(2)
    tracker = Tracker()
    ok, frame = cap.read()
    if not ok:
        print("error reading video")
        exit(-1)
    roi = cv2.selectROI("tracking", frame, False, False)
    #roi = (218, 302, 148, 108)
    tracker.init(frame, roi)
    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            break
        x, y, w, h = tracker.update(frame)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 1)
        cv2.imshow('tracking', frame)
        c = cv2.waitKey(1) & 0xFF
        if c==27 or c==ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    """
