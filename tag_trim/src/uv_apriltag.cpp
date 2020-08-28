#include"uv_apriltag.h"

void UvApriltag::setPose2Uv(Apriltag tag){

		double x=tag.getX() + 2*tag.getVx();
		double y=tag.getY() + 2*tag.getVy();// +0.01;
		double z=tag.getZ() + 2*tag.getVz();
		Eigen::Vector4d c_m4(x, y, z, 1);

		uv = A*c_m4; // A = uv_R_c
		uv /=uv(2,0);  //tag centor point
		vel_uv = uv - pre_uv;

		float tagsize1 = 0.5*tag.getSize();
		float tagsize2 = -0.5*tag.getSize();
		/*
		sizem3x4 <<  tagsize2, tagsize2, tagsize1, tagsize1,
								tagsize2, tagsize1, tagsize1, tagsize2,
											0.0,       0.0,       0.0,       0.0;
		*/
		sizem3x4 <<  tagsize2, tagsize2, tagsize1, tagsize1,
								tagsize2, tagsize1, tagsize1, tagsize2,
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
		pre_uv = uv;
}
void setWindowGain(float Kp,float Ki,float Kd){
}
std::vector<cv::Point> UvApriltag::getltrb(float anzen,float velgain){

	anzenKx =anzen+ velgain*abs(vel_uv(0));
	anzenKy =anzen+ velgain*abs(vel_uv(1));

	tag_window_w  = anzenKx * (std::max({p1(0), p2(0), p3(0), p4(0)}) - std::min( {p1(0), p2(0), p3(0), p4(0)}));
	tag_window_h  = anzenKy * (std::max({p1(1), p2(1), p3(1), p4(1)}) - std::min( {p1(1), p2(1), p3(1), p4(1)}));

	lefttop.x=uv(0)-0.5*tag_window_w;
	lefttop.y=uv(1)-0.5*tag_window_h;
	rightbottom.x=uv(0)+0.5*tag_window_w;
	rightbottom.y=uv(1)+0.5*tag_window_h;
	if(lefttop.x<0)lefttop.x = 0;
	if(lefttop.y<0)lefttop.y = 0;
	if(rightbottom.x<0||rightbottom.x>img_size[0])rightbottom.x = img_size[0];
	if(rightbottom.y<0||rightbottom.y>img_size[1])rightbottom.y = img_size[1];
	if(lefttop.x > rightbottom.x ||lefttop.y > rightbottom.y ){
		std::cout << "error : uv_apriltag" << std::endl;
		std::cout << lefttop<< std::endl;
		std::cout << rightbottom<< std::endl;
		
		lefttop.x = 0;
		lefttop.y = 0;
		rightbottom.x = img_size[0];
		rightbottom.y = img_size[1];
	}
	ltrb[0] = lefttop;
	ltrb[1] = rightbottom;
	return ltrb;
}
cv::Point UvApriltag::getP1(){return cv::Point(p1(0),p1(1));};
cv::Point UvApriltag::getP2(){return cv::Point(p2(0),p2(1));};
cv::Point UvApriltag::getP3(){return cv::Point(p3(0),p3(1));};
cv::Point UvApriltag::getP4(){return cv::Point(p4(0),p4(1));};


