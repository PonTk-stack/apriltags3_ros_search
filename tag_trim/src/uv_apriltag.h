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
        Eigen::Matrix<double,3,3> Rt0,Rt,Rt2;
		UvApriltag():Camera()
        {
            ltrb = {lefttop , rightbottom};
            tag_velK = 0.90;
            anzenK =2.0;
            uv_velK = 1.0;
            
            pre_vel_uv<< 0,0,0;
        };
		~UvApriltag(){};

		void setPose2Uv(Apriltag *tag);
		void setPose2UvWithSmoothedMat(Apriltag *tag, cv::Mat &smoothedMat);

        std::vector<cv::Point> getltrb();
        std::vector<cv::Point> getMaxltrb();
        //void setICP(const sensor_msgs::CameraInfo::ConstPtr &info){Camera::setICP(info);};

		Eigen::Vector3d uv; // u,v,1 tag centor point

		double tag_basis_window_w; //basis window
		double tag_basis_window_h; //basis window
		double tag_window_w; //tracking window
		double tag_window_h;

        cv::Point getP1(){return cv::Point(p1(0),p1(1));};
		cv::Point getP2(){return cv::Point(p2(0),p2(1));};
		cv::Point getP3(){return cv::Point(p3(0),p3(1));};
		cv::Point getP4(){return cv::Point(p4(0),p4(1));};
        double getanzenK() {return anzenK;}
        double getuv_velK() {return uv_velK;}
        double gettag_velK() {return tag_velK;}
        void setanzenK(double gain) {anzenK = gain;}
        void setuv_velK(double gain) {uv_velK = gain;}
        void settag_velK(double gain) {tag_velK = gain;}

        int getPurePixelSize();
	private:
        Apriltag *tag_obj;
        Eigen::Vector4d c_m4;
        Eigen::Vector4d c_m4_vel;

        double tab_vec_x;
        double tab_vec_y;
        double tab_vec_z;
        double tab_pre_vec_x;
        double tab_pre_vec_y;
        double tab_pre_vec_z;

        double x,y,z;
        double vx,vy,vz;
        double tagsize;

		Eigen::Vector3d pre_uv; // u,v,1 tag centor point
		Eigen::Vector3d vel_uv; // u,v,1 tag centor point velocity
		Eigen::Vector3d pre_vel_uv; // u,v,1 tag centor point velocity

		Eigen::Matrix<double,3,4> sizem3x4;
		Eigen::Matrix<double,3,4> posem3x4;
		Eigen::Matrix<double,3,4> pointm3x4;

        //std::map< int, Eigen::Vector2f > 4ps;
        Eigen::Vector2f p1,p2,p3,p4;
        std::vector<cv::Point> ltrb; //[p,p]
        cv::Point lefttop,rightbottom;

        double anzenK;
        double uv_velK;
        double tag_velK;

        void makeltrb(int tag_basis_window_w,int tag_basis_window_h,
                        int tag_window_w,int tag_window_h);
};

#endif//UV_APRILTAG_H
