#ifndef UV_APRILTAG_H
#define UV_APRILTAG_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include "apriltag.h"
#include "camera.h"

#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>


class UvApriltag: Apriltag, public Camera{
	protected:
		std::vector<cv::Point> getltrb(float anzen = 1.0,float velgain = 0.01);
	public:
		UvApriltag():Camera(){
			ltrb = {lefttop , rightbottom};
			//anzenKx = 1.0;
			//anzenKy = 1.0;
		};
		~UvApriltag(){};

		void setPose2Uv(Apriltag tag);
		void setWindowGain(float Kp,float Ki,float Kd);
		void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){Camera::setICP(info);};
//UvApriltag uv_tag;
		cv::Point getP1();
		cv::Point getP2();
		cv::Point getP3();
		cv::Point getP4();
		Eigen::Matrix<unsigned int,2,1> p1,p2,p3,p4;
	private:
		std::vector<cv::Point> ltrb; //[p,p]
		cv::Point lefttop,rightbottom;

		Eigen::Vector3d uv; // u,v,1 tag centor point

		Eigen::Vector3d pre_uv; // u,v,1 tag centor point
		Eigen::Vector3d vel_uv; // u,v,1 tag centor point velocity

		Eigen::Matrix<double,3,4> sizem3x4;
		Eigen::Matrix<double,3,4> posem3x4;
		Eigen::Matrix<double,3,4> pointm3x4;

		float anzenKx ;
		float anzenKy ;
		float tag_window_w; //tracking window
		float tag_window_h;

};

#endif//UV_APRILTAG_H
