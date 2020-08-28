#ifndef NODELET_IMAGE_TRIM_H
#define NODELET_IMAGE_TRIM_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <eigen3/Eigen/Dense>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include <message_filters/sync_policies/approximate_time.h>

#include <nodelet/nodelet.h>

#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <stdlib.h>
#include <math.h>

#include "apriltag_detector.h"
#include "tracking.h"
#include "image_grabber.h"


typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Quaterniond Quat;
typedef Eigen::AngleAxisd AngleAxisd;

const double PI = 3.14159265358979323846;
inline Vector3 rad2deg(Vector3 rad) {return 180.0 / PI * rad;}
inline double rad2deg(double rad) {return 180.0 / PI * rad;}
inline Vector3 deg2rad(Vector3 deg) {return PI / 180.0 * deg;}
inline Vector3 q2rpy(Quat q) {return q.matrix().eulerAngles(0, 1, 2);}
inline Quat rpy2q(Vector3 rpy) {
	Quat q = AngleAxisd(rpy[0], Vector3::UnitX())
		* AngleAxisd(rpy[1], Vector3::UnitY())
		* AngleAxisd(rpy[2],   Vector3::UnitZ());
	return q;
}
inline Vector3 q2rpy2deg(Quat q) {return rad2deg(q2rpy(q));}





class ROSCommonNode
{
	protected:
		ROSCommonNode(int argc, char** argv, const char* node_name)
		{
			ros::init(argc, argv, node_name);
		}
};



typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy; 

namespace nodelet_image_trim{

class ImageConverter :  public nodelet::Nodelet   {

	private:
		ros::NodeHandle nh;
		ros::NodeHandle pnh;

		image_transport::Subscriber image_sub ;
		image_transport::Publisher image_pub;

		ros::Subscriber camera_info_sub;
		ros::Publisher camera_info_pub;

		sensor_msgs::ImagePtr img_msg;
		sensor_msgs::CameraInfoPtr camera_info_ptr;
		sensor_msgs::CameraInfo camera_info;

		ros::Subscriber tag_detection_sub;

		message_filters::Subscriber<sensor_msgs::Image> image_sub_mf;
		message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_mf;


		message_filters::Synchronizer<MySyncPolicy> *sync;//(MySyncPolicy(10), image_sub_mf, info_sub_mf);

		const int img_size[2] = {1280,720};

		cv::Mat image_ori;
		cv::Mat image_trim;
		cv::Mat image_conved = cv::Mat::zeros(img_size[0],img_size[1],CV_8UC3);

		cv::Point lefttop,rightbottom;
		int pv_cx,pv_cy,pv_w,pv_h;

		bool detect_flag = false;
		unsigned int  detect_count = 0;

		Eigen::Matrix<int,2,1> p1,p2,p3,p4;
		bool drawpoint_flag =true;

		Tracking2 track2;

		ApriltagDetector apriltag_detector;

		ImageGrabber ig;
	public: 
		virtual void onInit();
		ImageConverter();
		inline void imgconvCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfo::ConstPtr &info);
		inline void TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
		inline void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info);
		inline void paste(cv::Mat dst, cv::Mat src, int x, int y);
		inline void stampText(cv::Mat dst,Tracking2 track);
		cv::Mat tool(cv::Mat image_ori);
		
};
}

#endif //NODELET_IMAGE_TRIM_H
