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

void Tracking2::setWindowParam(ApriltagDetector &apriltag_detector,
        const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){
    ltrbs.clear();
    for(int i=0; i<msg->detections.size(); i++){
        uv_apriltag.setPose2Uv( apriltag_detector.getApriltag( msg->detections[i].id[0] ));
        ltrbs.push_back( uv_apriltag.getltrb() );
    }
}
void Tracking2::setWindowParam(ApriltagDetector &apriltag_detector,
        const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg, cv::Mat &smoothedMat){
    ltrbs.clear();
    for(int i=0; i<msg->detections.size(); i++){
        uv_apriltag.setPose2Uv( apriltag_detector.getApriltag( msg->detections[i].id[0] ));
        ltrbs.push_back( uv_apriltag.getltrb() );
    }
}
std::vector<cv::Point> Tracking2::getltrb(void)
{
    return uv_apriltag.getltrb();
}
std::vector<std::vector<cv::Point>> Tracking2::getWindowParam(void)
{
    return ltrbs;
}


