#ifndef NODELET_TRACKER_H
#define NODELET_TRACKER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>


#include "../tracking.h"


#include "tag_trim/ltrbArray.h"
#include "tag_trim/ltrb.h"

#include "share.h"
#include "../camera.h"
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>

namespace nodelet_tracker{

    class MeasuringInstrument{
        public:
            void detected();
            void nondetected();
            void reset();

        protected:
            //計測
            bool run = false;
            int count = 0;
            int count_detected = 0;
            int count_err = 0;
            int count_pure_pixel = 0;
    };


    class Tracker : public nodelet::Nodelet {
        public:
            virtual void onInit();
            Tracker();
            ~Tracker();
            void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info);
            void TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
            void TagDetectProcess(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
            void TagDetectedInit();

            std::vector<std::vector<cv::Point>> ltrbs;
            Camera cam;
        protected:

            std::vector<cv::Point> ltrb;
            tag_trim::ltrb  box;
            ApriltagDetector apriltag_detector;
            Tracking2 track2;

            //計測
            bool continuation_flag = false;
            MeasuringInstrument  measure;
            int count = 0;
            int count_detected = 0;
            int count_err = 0;
            int count_pure_pixel = 0;
        private:
            Share* share = new Share();
            tag_trim::ltrbArray boxes;


            ros::NodeHandle nh;
            ros::NodeHandle pnh;


            ros::Subscriber camera_info_sub;
            ros::Subscriber tag_detection_sub;
    };
};
#endif //NODELET_TRACKER_H
