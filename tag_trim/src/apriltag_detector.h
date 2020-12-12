#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include "apriltag.h"
#include "uv_apriltag.h"
#include <iostream>
#include <vector>
#include <map>
#include "apriltag_ros/AprilTagDetection.h"


class ApriltagDetector {
	public:
		ApriltagDetector(){};
		~ApriltagDetector(){};

		//void setApriltag(int id,Eigen::Vector3d pose,Eigen::Quaterniond q,float size);
		Apriltag* getApriltag(unsigned int id);
		void setApriltag(const apriltag_ros::AprilTagDetection &detect);
		void resetApriltagVel();
		int savedApriltagsLength();

		std::vector<Apriltag> apriltags;
		//std::vector<UvApriltag> uv_apriltags;

	private:
		Apriltag obj_apriltag;
		int index;
		std::vector<int> IDs;
		std::vector<bool> Continuitys;
		std::vector<int>::iterator itr;
		inline int findID(int id);
        void updateTag(Apriltag* tag, int id,
                Eigen::Vector3d pose, Eigen::Quaterniond q,
                Eigen::Vector3d pre_pose, Eigen::Quaterniond pre_q);
};
#endif //APRILTAG_AGENT_H
