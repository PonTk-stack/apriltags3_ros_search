#ifndef CAMERA_H
#define CAMERA_H

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <eigen3/Eigen/Dense>
#include <vector>

class Camera{
	public:
		Camera(){};

		void setICP(const sensor_msgs::CameraInfo::ConstPtr &info);//set Intrinsic Camera ParameterImages
		void getA(){std::cout << A << std::endl;};


		sensor_msgs::CameraInfo camera_info;
	//protected:
//Eigen::Matrix3d A;  //カメラ内部パラメータ
		static Eigen::Matrix<double,3,4> A;//カメラ内部パラメータ
		Eigen::Matrix<double,3,3> A3x3;//カメラ内部パラメータ
		static Eigen::Matrix<double,3,3> K;//画素の有効サイズ(kx ky)
		Eigen::Matrix<double,3,4> f;//焦点距離
		Eigen::Matrix<double,5,1> dist;//歪みパラメータk1,k2,p1,p2, k3

		const int img_size[2] = {1280,720};
};

#endif//CAMERA_H
