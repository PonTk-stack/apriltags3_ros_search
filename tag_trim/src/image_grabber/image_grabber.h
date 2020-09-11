#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "../camera.h"
class ImageGrabber:Camera
{
    public:
        ImageGrabber(int id, const int w,const int h, const int c);
        void init(const int w, const int h);
        virtual ~ImageGrabber();
        virtual bool getImage(cv::Mat &) = 0;
        //virtual bool grabFrame(void) = 0;
        void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){Camera::setICP(info);};
        int getBufferSize(void);
//        virtual void getCamaraSettings();
    protected:
//        std::thread capture_thread;
        cv::Mat img;
        bool terminated;
        int id;
        int width;
        int height;
        int channel;
};
#endif //IMAGE_GRABBER_H
