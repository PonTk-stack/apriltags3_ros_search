#include "plugin_nodelet_image_trim.h"

#include <pluginlib/class_list_macros.h>

extern float Tracking::pre_t;
extern float Tracking::t;
extern Eigen::Matrix<double,3,4> Tracking::A;//カメラ内部パラメータ
extern Eigen::Matrix<double,3,3> Tracking::A3x3;//カメラ内部パラメータ
extern Eigen::Matrix<double,3,3> Tracking::K;//画素の有効サイズ(kx ky)
extern Eigen::Matrix<double,3,4> Tracking::f;//焦点距離
extern Eigen::Matrix<double,5,1> Tracking::dist;//歪みパラメータ k1, k2, p1,   p2, k3


void Tracking::measurePose(const apriltag_ros::AprilTagDetection &detect){
	id = detect.id[0];
	dt = t - pre_t;

	double qx=detect.pose.pose.pose.orientation.x;
	double qy=detect.pose.pose.pose.orientation.y;
	double qz=detect.pose.pose.pose.orientation.z;
	double qw=detect.pose.pose.pose.orientation.w;

	c_x = detect.pose.pose.pose.position.x;
	c_y = detect.pose.pose.pose.position.y;
	c_z = detect.pose.pose.pose.position.z;

	Eigen::Vector3d c_m3( c_x, c_y, c_z);
	Eigen::Quaterniond q0(qw,qx,qy,qz);
	float size = detect.size[0];
	//this->TagPose2uv(c_m3, q0 , size);


	/*Vector function*/
	if(!(this->preC_x==0&&this->preC_x==0&&this->preC_x==0)){
		c_vx = (c_x - this->preC_x);//  /dt ;
		c_vy = (c_y - this->preC_y);//  /dt ;
		c_vz = (c_z - this->preC_z);//  /dt ;
		Eigen::Vector3d dqv(qx - this->pre_q(0),//  /dt ;
												qy - this->pre_q(1),//  /dt ;
												qz - this->pre_q(2));//  /dt ;
		double dqw = qw - this->pre_q(3) ;// - this->pre_q.w;//  /dt ;

		Eigen::Quaterniond dq(dqw,dqv(0),dqv(1),dqv(2)  );

		Eigen::Vector3d nextC_m3( c_x+c_vx, c_y+c_vy, c_z+c_vz/**dt*/);
		Eigen::Quaterniond nextQ(qw+dqw, qx+dqv(0), qy+dqv(1), qz+dqv(2));

		this->TagPose2uv(nextC_m3, nextQ , size);
	}

	/*
		 double Q_sitaw2 =  acos(qw);
		 double sinw2 = sin(Q_sitaw2); 
		 double lamda_x = qx / sinw2;
		 double lamda_y = qy / sinw2;
		 double lamda_z = qz / sinw2;
		 this->Q_sita = Q_sitaw2 *2 ;
		 Eigen::Matrix<double,4,4> W_lamda;
		 Eigen::Matrix<double,4,4> W_c;

		 W_lamda <<          0,   lamda_z, -1*lamda_y, lamda_x,
		 -1*lamda_z,         0,    lamda_x, lamda_y,
		 lamda_y,-1*lamda_x,          0, lamda_z, 
		 -1*lamda_y,-1*lamda_x, -1*lamda_z;       0,

		 W_c = this->Q_sita * W_lamda;

		 Eigen::Quaterniond q_dasho;
		 Eigen::Vector4d qo(qx,qy,qz,qw);

		 q_dasho = 0.5 * W_c * qo; 
		 Eigen::Vector4d q_dash(qx,qy,qz,qw);
		 */

	Eigen::Vector4d q(qx,qy,qz,qw);
	this->pre_q =q;

	this->preQ_sita = this->Q_sita;

	this->preC_vx = c_vx;
	this->preC_vy = c_vy;
	this->preC_vz = c_vz;
	this->preC_x = c_x; 
	this->preC_y = c_y; 
	this->preC_z = c_z; 


	//Eigen::Vector3d rpy_rad = q.toRotationMatrix().eulerAngles(0, 1, 2);
	//	c_m << c_x/c_z,
	//				 c_y/c_z,
	//				 1;

	/*

		 this->uv_m = A*c_m; // A = uv_R_c
		 this->uv_m /=this->uv_m(2,0);  //tag centor point

		 this->uv_x = uv_m(0,0);
		 this->uv_y = uv_m(1,0);
		 */


	/*
		 Eigen::Matrix<double,3,4> sizev3x4;
	//sizev3x4<<size/-2, size/-2, size/2, size/2,
	//					size/2, size/-2, size/-2, size/2,
	//							0.0,    0.0,    0.0,    0.0;

	sizev3x4<<size/-2, size/-2, size/2, size/2,
	size/-2, size/2, size/2, size/-2,
	0.0,    0.0,    0.0,    0.0;

	c_m3x4 << c_x,c_x,c_x,c_x,
	c_y,c_y,c_y,c_y,
	c_z,c_z,c_z,c_z;

	uv_sizev3x4 =A3x3 *( c_m3x4 +  (q.toRotationMatrix() * sizev3x4) ) ;
	//uv_sizev /= uv_sizev(2,0);
	//this->size_w = uv_sizev(0,0);
	//this->size_h =  uv_sizev(1,0); 

	this->p1 << uv_sizev3x4(0,0)/uv_sizev3x4(2,0),
	uv_sizev3x4(1,0)/uv_sizev3x4(2,0);

	this->p2 << uv_sizev3x4(0,1)/uv_sizev3x4(2,1),
	uv_sizev3x4(1,1)/uv_sizev3x4(2,1);

	this->p3 << uv_sizev3x4(0,2)/uv_sizev3x4(2,2),
	uv_sizev3x4(1,2)/uv_sizev3x4(2,2);

	this->p4 << uv_sizev3x4(0,3)/uv_sizev3x4(2,3),
	uv_sizev3x4(1,3)/uv_sizev3x4(2,3);

	this->size_w  = std::max({p1(0), p2(0), p3(0), p4(0)}) - std::min( {p1(0), p2(0), p3(0), p4(0)});
	this->size_h  = std::max({p1(1), p2(1), p3(1), p4(1)}) - std::min( {p1(1), p2(1), p3(1), p4(1)});
	*/


	/*
		 system ("clear");
		 std::cout << "******size********" <<std::endl; 
		 std::cout << this->uv_sizev<<std::endl; 
		 std::cout << std::endl<<  q.toRotationMatrix() <<std::endl; 
		 std::cout << "******uv_m********" <<std::endl; 
		 std::cout << this->uv_m <<std::endl; 
		 */
}
//void Tracking::TagPose2uv(Eigen::Vector3d c_v, Eigen::Quaterniond q, float tagsize){

void Tracking::TagPose2uv(Eigen::Vector3d c_m3, Eigen::Quaterniond qu, float tagsize){

	double c_x_ = c_m3(0);
	double c_y_ = c_m3(1);
	double c_z_ = c_m3(2);

	Eigen::Vector4d c_m4(c_x_,c_y_,c_z_,1);

	uv_m = A*c_m4; // A = uv_R_c
	uv_m /=uv_m(2,0);  //tag centor point

	this->uv_x = uv_m(0,0);
	this->uv_y = uv_m(1,0);

	Eigen::Matrix<double,3,4> sizev3x4;
	/*
		 sizev3x4<<tagsize/-2, tagsize/-2,  tagsize/2, tagsize/2,
		 tagsize/2, tagsize/-2, tagsize/-2, tagsize/2,
		 0.0,     0.0,     0.0,    0.0;
		 */
	sizev3x4<<tagsize/-2, tagsize/-2, tagsize/2, tagsize/2,
		tagsize/-2, tagsize/2, tagsize/2, tagsize/-2,
		0.0,    0.0,    0.0,    0.0;

	Eigen::Matrix<double,3,4> c_m3x4;
	c_m3x4 << c_x_,c_x_,c_x_,c_x_,
				 c_y_,c_y_,c_y_,c_y_,
				 c_z_,c_z_,c_z_,c_z_;

	Eigen::Matrix<double,3,4> uv_sizev3x4;

	uv_sizev3x4 =A3x3 *( c_m3x4 +  (qu.toRotationMatrix() * sizev3x4) ) ;

	this->p1 << uv_sizev3x4(0,0)/uv_sizev3x4(2,0),
		uv_sizev3x4(1,0)/uv_sizev3x4(2,0);

	this->p2 << uv_sizev3x4(0,1)/uv_sizev3x4(2,1),
		uv_sizev3x4(1,1)/uv_sizev3x4(2,1);

	this->p3 << uv_sizev3x4(0,2)/uv_sizev3x4(2,2),
		uv_sizev3x4(1,2)/uv_sizev3x4(2,2);

	this->p4 << uv_sizev3x4(0,3)/uv_sizev3x4(2,3),
		uv_sizev3x4(1,3)/uv_sizev3x4(2,3);

	float anzenKx = 1.2;
	float anzenKy = 1.2;
	uv_vx = this->uv_x - this->preUv_x;
	uv_vy = this->uv_y - this->preUv_y;
	anzenKx += 0.5*abs(uv_vx)    ;
	anzenKy += 0.5*abs(uv_vy)    ;

	std::cout << "bbbbbbbbbb" <<std::endl; 
	std::cout << anzenKx <<std::endl; 
	std::cout << abs(uv_vx) <<std::endl; 

	this->size_w  = anzenKx * (std::max({p1(0), p2(0), p3(0), p4(0)}) - std::min( {p1(0), p2(0), p3(0), p4(0)}));
	this->size_h  = anzenKy * (std::max({p1(1), p2(1), p3(1), p4(1)}) - std::min( {p1(1), p2(1), p3(1), p4(1)}));


	this->preUv_x = this->uv_x;
	this->preUv_y = this->uv_y;

}




void callback(const sensor_msgs::ImageConstPtr& image,const sensor_msgs::CameraInfoConstPtr& cam_info){
}

//ImageConverter::ImageConverter(int argc,char** argv,const char* node_name): ROSCommonNode(argc, argv,node_name){  }
//

namespace nodelet_image_trim{

ImageConverter::ImageConverter(){
}


void ImageConverter::onInit(){

	nh = getNodeHandle();

	pnh = getPrivateNodeHandle();

	//ros::init(argc, argv,node_name);

	image_transport::ImageTransport it(nh);

	//	image_sub = it.subscribe("image_topic", 10, &ImageConverter::imgconvCallback,this);
	//	camera_info_sub = nh.subscribe("camera_info_topic",10,&ImageConverter::InfoCallback,this);
	tag_detection_sub = nh.subscribe("tag_topic", 10, &ImageConverter::TagDetectCallback,this);

	image_pub = it.advertise("image_trim_node/image_trimmed", 10);
	camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("image_trim_node/camera_info", 10);


	image_sub_mf.subscribe(nh,"image_topic", 1);
	info_sub_mf.subscribe(nh,"camera_info_topic", 1);

	//	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub_mf, info_sub_mf);
	//	sync.registerCallback(boost::bind(&ImageConverter::imgconvCallback ,this, _1, _2));

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10) , image_sub_mf,info_sub_mf); 
	sync->registerCallback(boost::bind(&ImageConverter::imgconvCallback ,this, _1, _2));



	lefttop.x = 0;
	lefttop.y = 0;
	rightbottom.x = img_size[0];
	rightbottom.y = img_size[1];
}


void ImageConverter::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info){
	camera_info = *info;

	const double kx = camera_info.K[0];
	const double ox = camera_info.K[2];
	const double ky = camera_info.K[4];
	const double oy = camera_info.K[5];

	const double fx = camera_info.P[0];
	const double cx = camera_info.P[2];
	const double fy = camera_info.P[5];
	const double cy = camera_info.P[6];

	const double k1 = camera_info.D[0];
	const double k2 = camera_info.D[1];
	const double p1 = camera_info.D[2];
	const double p2 = camera_info.D[3];
	const double k3 = camera_info.D[4];

	Tracking::K << kx,  0, ox,
		0, ky, oy,
		0,  0,  1;
	Tracking::A << fx,  0, cx, 0,
		0, fy, cy, 0,
		0,  0,  1, 0;

	Tracking::A3x3 <<fx,  0, cx,
		0, fy, cy,
		0,  0,  1;

	Tracking::dist << k1, k2, p1, p2, k3;

	//	Tracking::f = Tracking::K.inverse() * Tracking::A;
}

cv::Mat ImageConverter::tool(cv::Mat image){
	if(!image.data) {
		std::cout << "Error: the image wasn't correctly loaded." << std::endl;
		exit(1);
	}
	// We iterate over all pixels of the image
	for(int r = 0; r < image.rows; r++) {
		// We obtain a pointer to the beginning of row r
		cv::Vec3b* ptr = image.ptr<cv::Vec3b>(r);
		for(int c = 0; c < image.cols; c++) {
			// We invert the blue and red values of the pixel
			//            ptr[c] = cv::Vec3b(ptr[c][2],ptr[c][1],ptr[c][0]);
		}
	}
	return image;
}


void ImageConverter::imgconvCallback(const sensor_msgs::ImageConstPtr& msg  , const sensor_msgs::CameraInfo::ConstPtr &info){

	InfoCallback(info);

	try {
		image_ori = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	if(detect_flag){
		cv::Mat image_conved = cv::Mat::zeros(image_ori.rows,image_ori.cols,CV_8UC3);
		//ROS_INFO_STREAM(image_ori.cols );
		//cv::rectangle(image_conved, lefttop, rightbottom, cv::Scalar(0,0,255), -1, CV_AA);
		cv::Rect roi(lefttop, cv::Size(rightbottom.x-lefttop.x, rightbottom.y-lefttop.y));
		image_trim = image_ori(roi); // 切り出し画像
		//paste(image_conved,image_trim,pv_cx,pv_cy);
		paste(image_conved,image_trim,lefttop.x,lefttop.y);


//		cv::imshow("(published) image_conved", image_conved);
//		cv::waitKey(1);

		img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", image_conved).toImageMsg();


/*
		if(drawpoint_flag){ 
			
			cv::circle(image_conved, cv::Point(p1(0), p1(1)), 5, cv::Scalar(0,0,200), 1);
			cv::circle(image_conved, cv::Point(p2(0), p2(1)), 5, cv::Scalar(0,0,200), 1);
			cv::circle(image_conved, cv::Point(p3(0), p3(1)), 5, cv::Scalar(0,0,200), 1);
			cv::circle(image_conved, cv::Point(p4(0), p4(1)), 5, cv::Scalar(0,0,200), 1);
			stampText(image_conved,track); 
		}
		cv::imshow("image_conved", image_conved);
		cv::waitKey(1);
		*/
	}
	else{
		//image_conved = image_ori.clone();
		image_ori.copyTo(image_conved);
//		cv::imshow("(published) image_conved", image_conved);
//		cv::waitKey(1);
		img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", image_conved).toImageMsg();
	}

	camera_info_ptr = boost::make_shared<sensor_msgs::CameraInfo>(camera_info);

	img_msg->header.stamp = ros::Time::now();
	camera_info_ptr->header.stamp = img_msg->header.stamp;

	image_pub.publish(img_msg);
	camera_info_pub.publish(camera_info_ptr);

}







void ImageConverter::stampText(cv::Mat dst,Tracking track){
	//	std::stringstream ss;
	//	ss << track.id;
	//	cv::String text = ss.str();
	const int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	const double fontscale = 0.5;
	//	int baseline;
	//	cv::Size textsize = cv::getTextSize(text, fontface,
	//																			fontscale, 2, &baseline);
	cv::putText(dst, "p1",
			cv::Point((int)(track.p1(0)),
				(int)(track.p1(1))),
			fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);

	cv::putText(dst, "p2",
			cv::Point((int)(track.p2(0)),
				(int)(track.p2(1))),
			fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);

	cv::putText(dst, "p3",
			cv::Point((int)(track.p3(0)),
				(int)(track.p3(1))),
			fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);
	cv::putText(dst, "p4",
			cv::Point((int)(track.p4(0)),
				(int)(track.p4(1))),
			fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);
}


void ImageConverter::paste(cv::Mat dst, cv::Mat src, int x, int y) {
	cv::Mat resized_img = src;
	int width = src.rows;
	int height = src.cols;
	//cv::resize(src, resized_img, cv::Size(width, height));

	if (x >= dst.cols || y >= dst.rows) return;
	int w = (x >= 0) ? std::min(dst.cols - x, resized_img.cols) : std::min(std::max(resized_img.cols + x, 0), dst.cols);
	int h = (y >= 0) ? std::min(dst.rows - y, resized_img.rows) : std::min(std::max(resized_img.rows + y, 0), dst.rows);
	int u = (x >= 0) ? 0 : std::min(-x, resized_img.cols - 1);
	int v = (y >= 0) ? 0 : std::min(-y, resized_img.rows - 1);
	int px = std::max(x, 0);
	int py = std::max(y, 0);

	cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
	cv::Mat roi_resized = resized_img(cv::Rect(u, v, w, h));
	roi_resized.copyTo(roi_dst);
}


void ImageConverter::RectanglePoint(const apriltag_ros::AprilTagDetection &detect){

	pv_w = std::max({detect.pxdata[0],detect.pxdata[1],detect.pxdata[2],detect.pxdata[3]}) - std::min({detect.pxdata[0],detect.pxdata[1],detect.pxdata[2],detect.pxdata[3]});

	pv_h = std::max({detect.pydata[0],detect.pydata[1],detect.pydata[2],detect.pydata[3]}) - std::min({detect.pydata[0],detect.pydata[1],detect.pydata[2],detect.pydata[3]});

	pv_cx = (detect.pxdata[0]+detect.pxdata[1]+detect.pxdata[2]+detect.pxdata[3])/4;

	pv_cy = (detect.pydata[0]+detect.pydata[1]+detect.pydata[2]+detect.pydata[3])/4;

}


void ImageConverter::TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){

	XmlRpc::XmlRpcValue params;
	nh.getParam("/apriltags3_param", params);


	if(msg->detections.size()){
		detect_flag = true;
		Tracking::t = ros::Time::now().toSec();
		int i,t;

		for(i=0;i<msg->detections.size();i++){
			//std::vector<apriltag_ros::AprilTagDetectionArray> it;

			track.measurePose(msg->detections[i]);
			p1 = track.p1; 
			p2 = track.p2; 
			p3 = track.p3; 
			p4 = track.p4; 


			pv_cx = track.uv_x;
			pv_cy = track.uv_y;

			pv_w = track.size_w;
			pv_h = track.size_h;
			//RectanglePoint(msg->detections[i]);




			/*
				 lefttop.x = pv_cx - 0.5*pv_w;
				 lefttop.y = pv_cy - 0.5*pv_h;
				 rightbottom.x = pv_cx + 0.5*pv_w;
				 rightbottom.y = pv_cy + 0.5*pv_h;
				 */
			drawpoint_flag = true;
			if(p1(0)<0 || p2(0)<0 || p3(0)<0 || p4(0)<0){drawpoint_flag = false;}
			else if(p1(0)<img_size[0] || p2(0)<img_size[0] || p3(0)<img_size[0] || p4(0)<img_size[0] ){drawpoint_flag = false;}

			if(p1(1)<0 || p2(1)<0 || p3(1)<0 || p4(1)<0){drawpoint_flag = false;}
			else if(p1(1)<img_size[1] || p2(1)<img_size[1] || p3(1)<img_size[1] || p4(1)<img_size[1] ){drawpoint_flag = false;}




			lefttop.x = pv_cx - 0.5*pv_w;
			lefttop.y = pv_cy - 0.5*pv_h;
			rightbottom.x = pv_cx + 0.5*pv_w;
			rightbottom.y = pv_cy + 0.5*pv_h;
			if(lefttop.x<0)lefttop.x = 0;
			//else if(lefttop.x>img_size[0])lefttop.x = img_size[0];
			if(lefttop.y<0)lefttop.y = 0;
			//else if(lefttop.y>img_size[1])lefttop.y = img_size[1];
			//if(rightbottom.x<0)rightbottom.x = 0;
			if(rightbottom.x>img_size[0])rightbottom.x = img_size[0];
			//if(rightbottom.y<0)rightbottom.y = 0;
			if(rightbottom.y>img_size[1])rightbottom.y = img_size[1];

			//ROS_INFO("w:%d h:%d cx:%d cy:%d",pv_w,pv_h,pv_cx,pv_cy);
			if(lefttop.x > rightbottom.x){
				lefttop.x = 0;
				rightbottom.x=img_size[0];
			}
			if(lefttop.y > rightbottom.y){
				lefttop.y = 0;
				rightbottom.y=img_size[1];
			}
			//std::cout << rightbottom <<"  "<< lefttop <<std::endl; 

		}
		Tracking::pre_t = Tracking::t;
	}
	else{
		detect_flag = false;
		lefttop.x = 0;
		lefttop.y = 0;
		rightbottom.x = img_size[0];
		rightbottom.y = img_size[1];

	}
}
}
PLUGINLIB_EXPORT_CLASS(nodelet_image_trim::ImageConverter , nodelet::Nodelet)
