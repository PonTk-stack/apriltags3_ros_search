#include "apriltag_detector.h"

Apriltag* ApriltagDetector::getApriltag(unsigned int id){
	index = findID(id);
	return &apriltags[index];
}
//void ApriltagDetector::setApriltag(int id,Eigen::Vector3d pose, Eigen::Quaterniond q ,float size){
void ApriltagDetector::setApriltag(const apriltag_ros::AprilTagDetection &detect){
	int id = detect.id[0];
	Eigen::Vector3d pose( detect.pose.pose.pose.position.x,
												detect.pose.pose.pose.position.y,
												detect.pose.pose.pose.position.z );
	Eigen::Quaterniond q( detect.pose.pose.pose.orientation.w,
												detect.pose.pose.pose.orientation.x,
												detect.pose.pose.pose.orientation.y,
												detect.pose.pose.pose.orientation.z );
	float size =  detect.size[0];

	index = findID(id);

	if(index >= 0){
		updateTag(&apriltags[index], id, pose, q);
	}
	else{
		Apriltag tag(id,pose,q,size);
		apriltags.push_back(tag);
		IDs.push_back(id);
		}
}
int ApriltagDetector::savedApriltagsLength(){
	return apriltags.size();
}
void ApriltagDetector::updateTag(Apriltag* tag, int id,Eigen::Vector3d pose, Eigen::Quaterniond q){
	tag->update(id,pose,q);
}
inline int ApriltagDetector::findID(int id){
	itr = std::find(IDs.begin(), IDs.end(),id);
  index = std::distance( IDs.begin(), itr );
	if (index != IDs.size()){return index;}// found it
	else {return -1;}// doesn't exist
}
