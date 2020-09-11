#define avcodec_alloc_frame av_frame_alloc
#define PIXEL_FORMAT_RGB24 AV_PIXEL_FORMAT_RGB24
#define PIXEL_FORMAT_MJPEG AV_PIXEL_FORMAT_MJPEG

#include "image_trim.h"
#include <nodelet/loader.h>


int main(int argc,char **argv){
	const char *nodename = "tag_trim_plugin_nodelet_image_trim_node";

	ros::init(argc, argv,"tag_trim_plugin_nodelet_image_trim_node");
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	nodelet.load(ros::this_node::getName(),
					"ImageConverter",
					remap, nargv);
	ros::spin();
	return 0;
}

