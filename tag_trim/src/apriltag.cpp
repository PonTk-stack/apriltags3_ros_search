#include "apriltag.h"



void Apriltag::update(unsigned int iid,Eigen::Vector3d ppos,
        Eigen::Quaterniond qq,Eigen::Vector3d pre_pos,
        Eigen::Quaterniond pre_q){
	if(id == iid){
		update_velocity(ppos,qq,pre_pos,pre_q );
		update_pose(ppos,qq);

	}
	else{
		std::cerr << "error:apriltag.cpp void update() /"<< id << " is miss-match"<<std::endl;
		exit(0);
	}

}
void Apriltag::reset(unsigned int iid,Eigen::Vector3d ppos,Eigen::Quaterniond qq){
	if(id == iid){
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
double Apriltag::getSize(){
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
double Apriltag::getAx(){
	return this->accel[0];
}
double Apriltag::getAy(){
	return this->accel[1];
}
double Apriltag::getAz(){
	return this->accel[2];
}
void Apriltag::update_pose(Eigen::Vector3d ppos, Eigen::Quaterniond qq){
	pose = ppos;
	q = qq;
}
void Apriltag::reset_velocity(){
	//speed << 0.0 ,0.0 ,0.0;
	//pre_speed << 0.0 ,0.0 ,0.0;
	//prepre_speed << 0.0 ,0.0 ,0.0;
	vq.w() = 0.0;
	vq.x() = 0.0;
	vq.y() = 0.0;
	vq.z() = 0.0;
}
void Apriltag::update_velocity(Eigen::Vector3d ppos,
        Eigen::Quaterniond qq,Eigen::Vector3d pre_pos,
        Eigen::Quaterniond pre_q){
	//vx = xx-x;
	///vy = yy-y;
	//vz = zz-z;

    obj_speed = ppos - pre_pos;

	//speed = ppos - pre_pos;
	speed =1.0* obj_speed +0.0* pre_speed +0.0* prepre_speed;

	vq.w() = qq.w() - pre_q.w();
	vq.x() = qq.x() - pre_q.x();
	vq.y() = qq.y() - pre_q.y();
	vq.z() = qq.z() - pre_q.z();

    accel = speed - pre_speed;
    pre_speed = speed;
    prepre_speed = pre_speed;
}

