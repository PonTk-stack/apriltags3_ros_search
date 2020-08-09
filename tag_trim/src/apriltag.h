#ifndef APRILTAG_H
#define APRILTAG_H

#include <eigen3/Eigen/Dense>
class Apriltag{
	public:
		Apriltag() :x(0),y(0),z(0),vx(0),vy(0),vz(0),q(0,0,0,1),vq(0,0,0,0){};
		~Apriltag(){};
	private:
		unsigned int id ;
		double x,y,z ;//pose
		double vx,vy,vz ;//vec
		//double ax,ay,az ;//acc
		Eigen::Vector4d q;//Quartation (w, x, y, z)
		Eigen::Vector4d vq;//Quartation (w, x, y, z)
};
#endif //APRILTAG_H
