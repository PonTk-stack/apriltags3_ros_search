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
		Tracking2(){
			ltrb = {lefttop , rightbottom};
			};
		~Tracking2(){};
		//inline Eigen::Vector3d q2rpy_deg(Eigen::Quaterniond q);
		void setWindowParam(ApriltagDetector apriltag_detector);
		void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){UvApriltag::setICP(info);};
		std::vector<cv::Point> getWindowParam(ApriltagDetector apriltag_detector,unsigned int id);

	//	UvApriltag uv_tag;
		//cv::Point getP1(){return p1;};
		//cv::Point getP2(){return p2;};
		//cv::Point getP3(){return p3;};
		//cv::Point getP4(){return p4;};

	private:
		std::vector<std::vector<cv::Point>> ltrbs; //[[p,p],[p,p]]
		std::vector<cv::Point> ltrb; //[p,p]
		cv::Point lefttop,rightbottom;
};
#endif //TRACKING_G
