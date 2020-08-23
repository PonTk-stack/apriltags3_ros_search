#include"uv_apriltag.h"

void UvApriltag::setPose2Uv(Apriltag tag){

		double x=tag.getX();
		double y=tag.getY();
		double z=tag.getZ();
		Eigen::Vector4d c_m4(x, y, z, 1);

		uv = A*c_m4; // A = uv_R_c
		uv /=uv(2,0);  //tag centor point

		float tagsize = tag.getSize();
		sizem3x4 <<  tagsize/-2, tagsize/-2, tagsize/2, tagsize/2,
								tagsize/-2, tagsize/2, tagsize/2, tagsize/-2,
											0.0,       0.0,       0.0,       0.0;
		posem3x4 <<  x,  x,  x,  x,
								 y,  y,  y,  y,
								 z,  z,  z,  z;

		pointm3x4 =A3x3 *( posem3x4 +  (tag.getQuaterniond().toRotationMatrix() * sizem3x4) ) ;
		p1 << pointm3x4(0,0) / pointm3x4(2,0),
					pointm3x4(1,0) / pointm3x4(2,0);
		p2 << pointm3x4(0,1) / pointm3x4(2,1),
					pointm3x4(1,1) / pointm3x4(2,1);
		p3 << pointm3x4(0,2) / pointm3x4(2,2),
					pointm3x4(1,2) / pointm3x4(2,2);
		p4 << pointm3x4(0,3) / pointm3x4(2,3),
					pointm3x4(1,3) / pointm3x4(2,3);
		getA();
		
		float anzenKx = 1.2;
		float anzenKy = 1.2;
		vel_uv = uv - pre_uv;
		anzenKx += 0.01*abs(vel_uv(0));
		anzenKy += 0.01*abs(vel_uv(1));
		tag_window_w  = anzenKx * (std::max({p1(0), p2(0), p3(0), p4(0)}) - std::min( {p1(0), p2(0), p3(0), p4(0)}));
		tag_window_h  = anzenKy * (std::max({p1(1), p2(1), p3(1), p4(1)}) - std::min( {p1(1), p2(1), p3(1), p4(1)}));
		pre_uv = uv;
}
