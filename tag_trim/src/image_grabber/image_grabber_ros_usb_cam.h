#ifndef IMAGE_GRABBER_ROS_USB_CAM_H
#define IMAGE_GRABBER_ROS_USB_CAM_H

#include <iostream>
#include <mutex>
#include "image_grabber.h"
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../Real-Time-Video-Stabilization/stabilization.h"

class ImageGrabberRosUsbCam:public ImageGrabber
{
    public:
        ImageGrabberRosUsbCam(int id, const int w,const int h, const int c);
        ~ImageGrabberRosUsbCam(){}
//        void init(const int w, const int h);
        //        virtual void getCamaraSettings();
        sensor_msgs::ImagePtr getImageMsg(cv::Mat & );
        sensor_msgs::CameraInfoPtr getInfoMsg( );
        bool grabFrame(const sensor_msgs::ImageConstPtr &msg,const sensor_msgs::CameraInfo::ConstPtr &info) ;
        bool getImage(cv::Mat &) override;
        bool getSmoothedImage(cv::Mat &);
        bool getSmoothedImageAndMat(cv::Mat &dst_img,cv::Mat &mat);

        int channels(){return channel;};
        static std::mutex lock_obj;

        Stabilization image_stabilizer;
        protected:
        //        std::thread capture_thread;
            int camera_device_number;

        private:
            bool catch_msg2BGR(const sensor_msgs::ImageConstPtr& msg);
            bool usbCamSetICP(const sensor_msgs::CameraInfo::ConstPtr &info);
            //sensor_msgs::CameraInfo::ConstPtr info;

            cv_bridge::CvImagePtr cv_img_ptr;
            sensor_msgs::ImagePtr img_msg_ptr;
            sensor_msgs::CameraInfoPtr cam_info_ptr;
            sensor_msgs::CameraInfo cam_info;



};
#endif //IMAGE_GRABBER_ROS_USB_CAM_H
