#include "tracking.h"
#define PI 3.141592

void Tracking2::setWindowParam(ApriltagDetector apriltag_detector){
}
std::vector<cv::Point> Tracking2::getWindowParam(ApriltagDetector apriltag_detector,unsigned int id){
	setPose2Uv(apriltag_detector.getApriltag(id));

	return getltrb(2.0,0.10);
}

cv::Point Tracking2::getP1(){return UvApriltag::getP1();};
cv::Point Tracking2::getP2(){return UvApriltag::getP2();};
cv::Point Tracking2::getP3(){return UvApriltag::getP3();};
cv::Point Tracking2::getP4(){return UvApriltag::getP4();};

