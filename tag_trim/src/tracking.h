#ifndef TRACKING_H
#define TRACKING_H

#include<vector>
#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include"apriltag_detector.h"
#include"uv_apriltag.h"

class Tracking2:UvApriltag{
	public:
		Tracking2(){};
		~Tracking2(){};
		//inline Eigen::Vector3d q2rpy_deg(Eigen::Quaterniond q);
		void setWindowParam(ApriltagDetector apriltag_detector);
		void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){UvApriltag::setICP(info);};
		std::vector<cv::Point> getWindowParam(ApriltagDetector apriltag_detector,unsigned int id);
		cv::Point getP1();
		cv::Point getP2();
		cv::Point getP3();
		cv::Point getP4();

	private:
		std::vector<std::vector<cv::Point>> ltrbs; //[[p,p],[p,p]]
		std::vector<std::vector<unsigned int>> p;
};
#endif //TRACKING_G
