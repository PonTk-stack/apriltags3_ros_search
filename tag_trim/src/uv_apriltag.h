#ifndef UV_APRILTAG_H
#define UV_APRILTAG_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <vector>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include "apriltag.h"
#include "camera.h"
#include<opencv2/highgui.hpp>


class UvApriltag: public Camera{
	public:
		UvApriltag():Camera()
        {
            ltrb = {lefttop , rightbottom};
        };
		~UvApriltag(){};

		void setPose2Uv(Apriltag *tag);
		void setPose2UvWithSmoothedMat(Apriltag *tag, cv::Mat &smoothedMat);

        std::vector<cv::Point> getltrb();
        //void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){Camera::setICP(info);};

		Eigen::Vector3d uv; // u,v,1 tag centor point

		float tag_window_w; //tracking window
		float tag_window_h;

        cv::Point getP1(){return cv::Point(p1(0),p1(1));};
		cv::Point getP2(){return cv::Point(p2(0),p2(1));};
		cv::Point getP3(){return cv::Point(p3(0),p3(1));};
		cv::Point getP4(){return cv::Point(p4(0),p4(1));};
	private:
    //    Apriltag tag_obj;
        Eigen::Vector4d c_m4;
        double x,y,z;
        float tagsize;

		Eigen::Vector3d pre_uv; // u,v,1 tag centor point
		Eigen::Vector3d vel_uv; // u,v,1 tag centor point velocity

		Eigen::Matrix<double,3,4> sizem3x4;
		Eigen::Matrix<double,3,4> posem3x4;
		Eigen::Matrix<double,3,4> pointm3x4;

        //std::map< int, Eigen::Vector2f > 4ps;
        Eigen::Vector2f p1,p2,p3,p4;
        std::vector<cv::Point> ltrb; //[p,p]
        cv::Point lefttop,rightbottom;

};

#endif//UV_APRILTAG_H
