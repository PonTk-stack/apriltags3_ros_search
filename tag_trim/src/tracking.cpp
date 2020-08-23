#include "tracking.h"
#define PI 3.141592
/*
inline Eigen::Vector3d q2rpy_deg(Eigen::Quaterniond q){
	double qx = q.x();
	double qy = q.y();
	double qz = q.z();
	double qw = q.w();
	double sinr_cosp = 2.0 * (qw * qx + qy * qz);
	double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
	double roll = atan2(sinr_cosp,cosr_cosp);
	double sinp = 2.0 * (qw * qy - qz * qx);
	double pitch;
	if (abs(sinp) >= 1){pitch = PI / 2;}
	else{pitch = asin(sinp);}
	double siny_cosp = 2.0 * (qw * qz + qx * qy);
	double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
	double yaw = atan2(siny_cosp, cosy_cosp);
	if(cosy_cosp<0){
		if(siny_cosp >= 0) yaw+=180*PI/180;
		if(siny_cosp <  0) yaw-=180*PI/180;
	}
	std::cout <<yaw*180/PI<<" "<<  roll*180/PI<< " "<<pitch*180/PI<<std::endl;
	Eigen::Vector3d v(yaw*180/PI,roll*180/PI,pitch*180/PI );
	return v;
}
*/

void Tracking2::setWindowParam(ApriltagDetector apriltag_detector){
	/*
	for (int i=0;i<apriltag_detector.savedApriltagsLength();i++){ 
		setPose2Uv(apriltag_detector.getApriltag(i));
		lefttop.x=uv(0)-0.5*window_w;
		lefttop.y=uv(1)-0.5*window_h;
		rightbottom.x=uv(0)+0.5*window_w;
		rightbottom.y=uv(1)+0.5*window_h;
		ltrb[0] = lefttop;
		ltrb[1] = rightbottom;
		//ltrb[0] = {uv(0)-0.5*window_w,uv(1)-0.5*window_h};
		//ltrb[1] = {uv(0)+0.5*window_w,uv(1)+0.5*window_h};
		ltrbs[i] = ltrb;
	}
	*/
}
std::vector<cv::Point> Tracking2::getWindowParam(ApriltagDetector apriltag_detector,unsigned int id){
		setPose2Uv(apriltag_detector.getApriltag(id));
		lefttop.x=uv(0)-0.5*tag_window_w;
		lefttop.y=uv(1)-0.5*tag_window_h;
		rightbottom.x=uv(0)+0.5*tag_window_w;
		rightbottom.y=uv(1)+0.5*tag_window_h;
		if(lefttop.x<0)lefttop.x = 0;
		if(lefttop.y<0)lefttop.y = 0;
		if(rightbottom.x>img_size[0])rightbottom.x = img_size[0];
		if(rightbottom.y>img_size[1])rightbottom.y = img_size[1];
		if(lefttop.x > rightbottom.x || lefttop.y > rightbottom.y ){
			std::cout << "error tracking.cpp" << std::endl;
			exit(0);
		}



		ltrb[0] = lefttop;
		ltrb[1] = rightbottom;
		return ltrb;
}


