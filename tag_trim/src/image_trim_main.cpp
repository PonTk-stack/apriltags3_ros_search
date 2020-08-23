#include "image_trim.h"


int main(int argc, char** argv)
{
	const char *nodename = "image_trim_node";
	ros::init(argc, argv,"image_trim_node");
	ImageConverter imgconverter(argc,argv,nodename);
//	ros::init (argc, argv, "img_subscriber");
//	ros::Rate loop(10);
//	cv::namedWindow("image_subscriber");
//	cv::startWindowThread();
	//ros::spin();
	/*
	ros::AsyncSpinner spinner(1);
	spinner.start();
	while(ros::ok()){
	}
	spinner.stop();
	cv::destroyWindow("image_ori");
	//	ros::spinOnce();
	//	loop.sleep();
	return 0;
	*/
	ros::spin();
	return 0;

	
}

