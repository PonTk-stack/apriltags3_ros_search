#include"camera.h"
#include <ros/ros.h>
Eigen::Matrix<double,3,4> Camera::A;
Eigen::Matrix<double,3,3> Camera::A3x3;//カメラ内部パラメータ
Eigen::Matrix<double,3,3> Camera::K;
Eigen::Matrix<double,3,3> demo_A;//カメラ内部パラメータ
void Camera::setICP(const sensor_msgs::CameraInfo::ConstPtr &info){
	camera_info = *info;
	const double kx = camera_info.K[0];
	const double ox = camera_info.K[2];
	const double ky = camera_info.K[4];
	const double oy = camera_info.K[5];

	const double fx = camera_info.P[0];
	const double cx = camera_info.P[2];
	const double fy = camera_info.P[5];
	const double cy = camera_info.P[6];

	const double k1 = camera_info.D[0];
	const double k2 = camera_info.D[1];
	const double p1 = camera_info.D[2];
	const double p2 = camera_info.D[3];
	const double k3 = camera_info.D[4];
	K << kx,  0, ox,
				0, ky, oy,
				0,  0,  1;
	A << fx,  0, cx, 0,
				0, fy, cy, 0,
				0,  0,  1, 0;

	A3x3 <<fx,  0, cx,
            0, fy, cy,
            0,  0,  1;
	dist << k1, k2, p1, p2, k3;
    const std::vector<double> param_list = {fx,  0, cx,
                                            0, fy, cy,
                                            0,  0,  1 };
    ros::param::set("info_param/A", param_list );
    demo_A =  A3x3;
//f = K.inverse() * A;
}
