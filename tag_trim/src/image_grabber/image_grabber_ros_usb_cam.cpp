#include "image_grabber_ros_usb_cam.h"


std::mutex ImageGrabberRosUsbCam::lock_obj;

ImageGrabberRosUsbCam::ImageGrabberRosUsbCam(int iid, const int w,const int h, const int c)
    :ImageGrabber(iid, w, h, c),camera_device_number(0)
{
}


sensor_msgs::ImagePtr ImageGrabberRosUsbCam::getImageMsg(cv::Mat &src){
    if(channel == 1){
        img_msg_ptr = cv_bridge::CvImage(std_msgs::Header(),"mono8", src).toImageMsg();
    }else if (channel == 3){
        img_msg_ptr = cv_bridge::CvImage(std_msgs::Header(),"bgr8", src).toImageMsg();
    }else{
        std::cerr << "ImageGrabber open error" << std::endl;
        exit(0);
    }
    return img_msg_ptr;
}


sensor_msgs::CameraInfoPtr ImageGrabberRosUsbCam::getInfoMsg(){
    cam_info_ptr = boost::make_shared<sensor_msgs::CameraInfo>(cam_info);
    return cam_info_ptr;
}


bool ImageGrabberRosUsbCam::grabFrame(const sensor_msgs::ImageConstPtr &msg,const sensor_msgs::CameraInfo::ConstPtr &info){
    bool imgflag,infoflag;

    imgflag = catch_msg2BGR(msg);
    infoflag = usbCamSetICP(info);
    if(imgflag &&infoflag){return true;}
    else {
        std::cerr << "subscribe msg to convert it in image_grabber from ImageGrabberRosUsbCam::grabFrame() " << std::endl;
        return -1;
    }
}


bool ImageGrabberRosUsbCam::getImage(cv::Mat &dst_img){
    if(img.empty() || img.rows != dst_img.rows || img.cols != dst_img.cols ){
        std::cerr << "invaild image in image_grabber from ImageGrabberRosUsbCam::getImage() " << std::endl;
        return false;
    }else {
        std::lock_guard<std::mutex> lock_img(lock_obj);
        img.copyTo(dst_img);
        return true;
    }
}

bool ImageGrabberRosUsbCam::getSmoothedImage(cv::Mat &dst_img){
    if(img.empty() || img.rows != dst_img.rows || img.cols != dst_img.cols ){
        std::cerr << "invaild image in image_grabber from ImageGrabberRosUsbCam::getSmoothedImage() " << std::endl;
        return false;
    }else {
        //cv::imshow("ImageGrabberRosUsbCam::getSmoothedImage",img);
        //cv::waitKey(1);
        std::lock_guard<std::mutex> lock_img(lock_obj);
        image_stabilizer.getSmoothedFrame(img,dst_img);
        //img.copyTo(dst_img);
        return true;
    }
}
bool ImageGrabberRosUsbCam::getSmoothedImageAndMat(cv::Mat &dst_img,cv::Mat &mat){
    if(img.empty() || img.rows != dst_img.rows || img.cols != dst_img.cols ){
        std::cerr << "invaild image in image_grabber from ImageGrabberRosUsbCam::getSmoothedImage() " << std::endl;
        return false;
    }else {
        //cv::imshow("ImageGrabberRosUsbCam::getSmoothedImageAndMat",img);
        //cv::waitKey(1);
        std::lock_guard<std::mutex> lock_img(lock_obj);
        image_stabilizer.getSmoothedFrameAndMat(img,dst_img,mat);
        //img.copyTo(dst_img);
        return true;
    }
}

/**************************private***************************/
bool ImageGrabberRosUsbCam::catch_msg2BGR(const sensor_msgs::ImageConstPtr& msg)
{
     try {
        std::lock_guard<std::mutex> lock_img(lock_obj);
        if(channel == 1){
         //cv_img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         //cv::cvtColor(cv_img_ptr->image, img, CV_BGR2GRAY);
         cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
         cv_img_ptr->image.copyTo(img);
        }
        else if (channel == 3){
         img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        else{
            std::cerr << "ImageGrabber open error" << std::endl;
            exit(0);
        }
     }
     catch (cv_bridge::Exception& e) {
         ROS_ERROR("cv_bridge exception: %s", e.what());
     }
     return true;
}
bool ImageGrabberRosUsbCam::usbCamSetICP(const sensor_msgs::CameraInfo::ConstPtr &info){
    try{
        cam_info = *info;
        setICP(info);
    }
     catch (...) {
        std::cerr << "erorr in  ImageGrabberRosUsbCam::usbCamSetICP() " << std::endl;
     }
     return true;
}
