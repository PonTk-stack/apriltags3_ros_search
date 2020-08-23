#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include "apriltag.h"
#include <iostream>
#include <vector>
#include <map>
#include "apriltag_ros/AprilTagDetection.h"


class ApriltagDetector {
	public:
		ApriltagDetector(){};
		~ApriltagDetector(){};

		Apriltag getApriltag(unsigned int id);
		//void setApriltag(int id,Eigen::Vector3d pose,Eigen::Quaterniond q,float size);
		void setApriltag(const apriltag_ros::AprilTagDetection &detect);
		int savedApriltagsLength();

		std::vector<Apriltag> apriltags;
		std::vector<Apriltag> *p_apriltags;

	private:
		Apriltag obj_apriltag;
		int index;
		std::vector<int> IDs;
		std::vector<bool> Continuitys;
		std::vector<int>::iterator itr;
		inline int findID(int id);
		void updateTag(Apriltag* tag, int id, Eigen::Vector3d pose,Eigen::Quaterniond q);
};
#endif //APRILTAG_AGENT_H
