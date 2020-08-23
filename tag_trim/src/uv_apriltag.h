#ifndef UV_APRILTAG_H
#define UV_APRILTAG_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include "apriltag.h"
#include "camera.h"


class UvApriltag: Apriltag, public Camera{
	public:
		UvApriltag():Camera(){};
		~UvApriltag(){};

		void setPose2Uv(Apriltag tag);

void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){Camera::setICP(info);};


		Eigen::Vector3d uv; // u,v,1 tag centor point
		float tag_window_w; //tracking window
		float tag_window_h;
		Eigen::Matrix<unsigned int,2,1> p1,p2,p3,p4;
	private:

		Eigen::Vector3d pre_uv; // u,v,1 tag centor point

		Eigen::Vector3d vel_uv; // u,v,1 tag centor point velocity

		Eigen::Matrix<double,3,4> sizem3x4;
		Eigen::Matrix<double,3,4> posem3x4;
		Eigen::Matrix<double,3,4> pointm3x4;

		std::vector<std::vector<unsigned int>> p;
};

#endif//UV_APRILTAG_H
