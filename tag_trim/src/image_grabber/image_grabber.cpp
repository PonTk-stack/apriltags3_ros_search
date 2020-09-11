
#include "image_grabber.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

ImageGrabber::ImageGrabber(int iid, const int w,const int h, const int c):
    Camera(),terminated(false),id(iid),channel(c)
{
    init(w,h);
        if(channel == 1){
            img = cv::Mat(height, width, CV_8UC1);
        }
        else if (channel == 3){
            img = cv::Mat(height, width, CV_8UC3);
        }
        else{
            std::cerr << "ImageGrabber open error" << std::endl;
            exit(0);
        }
}
void ImageGrabber::init(const int w, const int h){
    const int* size = getImageSize();
    width = size[0];
    height = size[1];
    if(!(w == width || h ==height)){
                std::cerr << "ImageGrabber open error \n" << "please cheack image size configure in camera.h"<< std::endl;
                exit(0);
    }
}
ImageGrabber::~ImageGrabber(){};

int ImageGrabber::getBufferSize(void)
{
    return width * height * channel;
}
