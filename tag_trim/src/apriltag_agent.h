#ifndef APRILTAG_AGENT_H
#define APRILTAG_AGENT_H

#include "apriltag.h"
#include <iostream>
#include <vector>

class ApriltagAgent{
	public:
		ApriltagAgent(){};
		~ApriltagAgent(){};

		std::vector<Apriltag> getApriltag(void);
		std::vector<Apriltag> setApriltag(int id,double x,double y,double z,Eigen::Quaterniond q);

		std::vector<Apriltag> apriltag;
	private:
		int i;

};
#endif //APRILTAG_AGENT_H
