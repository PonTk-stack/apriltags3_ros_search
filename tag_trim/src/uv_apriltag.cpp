#include"uv_apriltag.h"

void UvApriltag::setPose2Uv(Apriltag *tag){

		x = tag->getX() + tag->getVx() ;
		y = tag->getY() + tag->getVy() ;
		z = tag->getZ() + tag->getVz() ;

        c_m4 << x, y, z, 1 ;

		uv = getA()*c_m4; // A = uv_R_c
		uv /=uv(2,0);  //tag centor point

		tagsize = tag->getSize();
		sizem3x4 <<  tagsize/-2, tagsize/-2, tagsize/2, tagsize/2,
                     tagsize/-2, tagsize/2, tagsize/2, tagsize/-2,
                            0.0,       0.0,       0.0,       0.0;
		posem3x4 <<  x,  x,  x,  x,
                     y,  y,  y,  y,
                     z,  z,  z,  z;

		pointm3x4 =getA3x3() *( posem3x4 +  (tag->getQuaterniond().toRotationMatrix() * sizem3x4) ) ;
		p1 << pointm3x4(0,0) / pointm3x4(2,0),
                pointm3x4(1,0) / pointm3x4(2,0);
		p2 << pointm3x4(0,1) / pointm3x4(2,1),
                pointm3x4(1,1) / pointm3x4(2,1);
		p3 << pointm3x4(0,2) / pointm3x4(2,2),
                pointm3x4(1,2) / pointm3x4(2,2);
		p4 << pointm3x4(0,3) / pointm3x4(2,3),
                pointm3x4(1,3) / pointm3x4(2,3);
		float anzenKx = 1.9;
		float anzenKy = 1.9;
		vel_uv = uv - pre_uv;
		anzenKx += 0.01*abs(vel_uv(0));
		anzenKy += 0.01*abs(vel_uv(1));
		tag_window_w  = anzenKx * (std::max({p1(0), p2(0), p3(0), p4(0)}) - std::min( {p1(0), p2(0), p3(0), p4(0)}));
		tag_window_h  = anzenKy * (std::max({p1(1), p2(1), p3(1), p4(1)}) - std::min( {p1(1), p2(1), p3(1), p4(1)}));
        pre_uv = uv;
}
void setPose2UvWithSmoothedMat(Apriltag *tag, cv::Mat &smoothedMat){








}
std::vector<cv::Point> UvApriltag::getltrb()
{
    lefttop.x=uv(0)-0.5*tag_window_w;
    lefttop.y=uv(1)-0.5*tag_window_h;
    rightbottom.x=uv(0)+0.5*tag_window_w;
    rightbottom.y=uv(1)+0.5*tag_window_h;
    if(lefttop.x<0)lefttop.x = 0;
    if(lefttop.y<0)lefttop.y = 0;
    if(rightbottom.x>img_size[0])rightbottom.x = img_size[0];
    if(rightbottom.y>img_size[1])rightbottom.y = img_size[1];
    if(lefttop.x > rightbottom.x || lefttop.y > rightbottom.y ){
        lefttop.x = 0;
        lefttop.y = 0;
        rightbottom.x = img_size[0];
        rightbottom.y = img_size[1];
        std::cerr << "error tracking.cpp" << std::endl;
        //exit(0);
    }
    ltrb[0] = lefttop;
    ltrb[1] = rightbottom;
    return ltrb;
}

