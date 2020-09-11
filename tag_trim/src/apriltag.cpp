#include "apriltag.h"


void Apriltag::update(unsigned int iid,Eigen::Vector3d ppos,Eigen::Quaterniond qq){
	if(id == iid){
		update_velocity(ppos,qq);
		update_pose(ppos,qq);

	}
	else{
		std::cerr << "error:apriltag.cpp void update() /"<< id << " is miss-match"<<std::endl;
		exit(0);
	}

}
void Apriltag::reset(unsigned int iid,Eigen::Vector3d ppos,Eigen::Quaterniond qq){
	if(id = iid){
		update_pose(ppos,qq);
		reset_velocity();
	}
	else{
		std::cerr << "error:apriltag.cpp void reset()/ "<< id << " is miss-match"<<std::endl;
		exit(0);
	}
}
Eigen::Quaterniond Apriltag::getQuaterniond(){
	return this->q;

}
float Apriltag::getSize(){
	return this->size;
}
double Apriltag::getX(){
	return this->pose[0];

}
double Apriltag::getY(){
	return this->pose[1];
}
double Apriltag::getZ(){
	return this->pose[2];
}
double Apriltag::getVx(){
	return this->speed[0];
}
double Apriltag::getVy(){
	return this->speed[1];
}
double Apriltag::getVz(){
	return this->speed[2];
}
void Apriltag::update_pose(Eigen::Vector3d ppos, Eigen::Quaterniond qq){
	pose = ppos;
	q = qq;
}
void Apriltag::reset_velocity(){
	speed << 0 ,0 ,0;
	vq.w() = 0;
	vq.x() = 0;
	vq.y() = 0;
	vq.z() = 0;
}
void Apriltag::update_velocity(Eigen::Vector3d ppos,Eigen::Quaterniond qq){
	//vx = xx-x;
	///vy = yy-y;
	//vz = zz-z;
	speed = ppos - pose;

	vq.w() = qq.w() - q.w();
	vq.x() = qq.x() - q.x();
	vq.y() = qq.y() - q.y();
	vq.z() = qq.z() - q.z();
}

