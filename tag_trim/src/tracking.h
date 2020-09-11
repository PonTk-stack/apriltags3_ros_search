#ifndef TRACKING_H
#define TRACKING_H

#include<vector>
#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include"apriltag_detector.h"
#include"uv_apriltag.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"


class Tracking2{
	public:
		Tracking2(){
            ltrbs.reserve(10);
        };
		~Tracking2(){};
        void setWindowParam(ApriltagDetector &apriltag_detector,
                        const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
        void setWindowParam(ApriltagDetector &apriltag_detector,
                        const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg,cv::Mat &smoothedMat);
        std::vector<cv::Point> getltrb(void);

		//void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){UvApriltag::setICP(info);};
        std::vector<std::vector<cv::Point>> getWindowParam(void);
	//	UvApriltag uv_tag;
        UvApriltag uv_apriltag;

		cv::Point getP1(){return uv_apriltag.getP1();};
		cv::Point getP2(){return uv_apriltag.getP2();};
		cv::Point getP3(){return uv_apriltag.getP3();};
		cv::Point getP4(){return uv_apriltag.getP4();};

	private:
		std::vector<std::vector<cv::Point>> ltrbs; //[[p,p],[p,p]]
};
#endif //TRACKING_G
