#ifndef NODELET_IMAGE_TRIM_H
#define NODELET_IMAGE_TRIM_H


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <eigen3/Eigen/Dense>


#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>


#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <stdlib.h>
#include <math.h>

#include "apriltag_detector.h"
#include "tracking.h"
//#include "image_grabber/image_grabber.h"

class ROSCommonNode
{
    protected:
        ROSCommonNode(int argc, char** argv, const char* node_name)
        {
            ros::init(argc, argv, node_name);
        }
};



typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy; 

namespace nodelet_image_trim{

    class ImageConverter :  public nodelet::Nodelet   {

        private:
            ros::NodeHandle nh;
            ros::NodeHandle pnh;

            image_transport::Subscriber image_sub ;
            image_transport::Publisher image_pub;

            ros::Subscriber camera_info_sub;
            ros::Publisher camera_info_pub;

            sensor_msgs::ImagePtr img_msg;
            sensor_msgs::CameraInfoPtr camera_info_ptr;
            sensor_msgs::CameraInfo camera_info;

            ros::Subscriber tag_detection_sub;

            message_filters::Subscriber<sensor_msgs::Image> image_sub_mf;
            message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_mf;

            message_filters::Synchronizer<MySyncPolicy> *sync;//(MySyncPolicy(10), image_sub_mf, info_sub_mf);

            const int img_size[2] = {1280,720};

            cv::Mat image_ori;
            cv::Mat image_trim;
            cv::Mat image_conved = cv::Mat::zeros(img_size[0],img_size[1],CV_8UC3);

            cv::Point lefttop,rightbottom;
            int pv_cx,pv_cy,pv_w,pv_h;

            bool detect_flag = false;

            Eigen::Matrix<int,2,1> p1,p2,p3,p4;
            bool drawpoint_flag =false;


            Tracking2 track2;

            ApriltagDetector apriltag_detector;

//            ImageGrabber ig;
        public:
            virtual void onInit();
            ImageConverter();
            void imgconvCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfo::ConstPtr &info);
            void TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
            void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info);
            void RectanglePoint(const apriltag_ros::AprilTagDetection &detect);
            void paste(cv::Mat dst, cv::Mat src, int x, int y);
            void stampText(cv::Mat dst,Tracking2 track);
            cv::Mat tool(cv::Mat image_ori);
    };
}
/*
   inline Eigen::Vector3d Tracking::q2rpy_deg(Eigen::Quaterniond q){
   double qx = q.x();
   double qy = q.y();
   double qz = q.z();
   double qw = q.w();
   double sinr_cosp = 2.0 * (qw * qx + qy * qz);
   double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
   double roll = atan2(sinr_cosp,cosr_cosp);
   double sinp = 2.0 * (qw * qy - qz * qx);
   double pitch;
   if (abs(sinp) >= 1){pitch = PI / 2;}
   else{pitch = asin(sinp);}
   double siny_cosp = 2.0 * (qw * qz + qx * qy);
   double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
   double yaw = atan2(siny_cosp, cosy_cosp);
   if(cosy_cosp<0){
   if(siny_cosp >= 0) yaw+=180*PI/180;
   if(siny_cosp <  0) yaw-=180*PI/180;
   }
   std::cout <<yaw*180/PI<<" "<<  roll*180/PI<< " "<<pitch*180/PI<<std::endl;
   Eigen::Vector3d v(yaw*180/PI,roll*180/PI,pitch*180/PI );
   return v;
   }
   */


#endif //NODELET_IMAGE_TRIM_H
