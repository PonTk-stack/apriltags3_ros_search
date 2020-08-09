#include "apriltag_agent.h"

std::vector<Apriltag>  ApriltagAgent::getApriltag(void){
	return apriltag;
}
std::vector<Apriltag>  ApriltagAgent::setApriltag(int id,double x,double y,double z, Eigen::Quaterniond q ){
	Apriltag tag;
	apriltag.push_back(tag);
	return apriltag;
}
