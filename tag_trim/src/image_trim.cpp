#include "image_trim.h"



void callback(const sensor_msgs::ImageConstPtr& image,const sensor_msgs::CameraInfoConstPtr& cam_info){
}



ImageConverter::ImageConverter(int argc,char** argv,const char* node_name): ROSCommonNode(argc, argv,node_name){

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
    //track2.setICP(info);
    //ig.setICP(info);
    camera_info = *info;
}

cv::Mat ImageConverter::tool(cv::Mat image){
    if(!image.data) {
        std::cerr << "Error: the image wasn't correctly loaded." << std::endl;
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
    //igruc.setMsg(msg ,info);
    bool f = igruc.grabFrame(msg ,info);
    try {
        image_ori = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    //igruc.getImage(image_ori);
    //igruc.getSmoothedImage(image_ori);
    cv::Mat stabedMat;
    igruc.getSmoothedImageAndMat(image_ori,stabedMat);
    std::cout << "stabedMat" << std::endl;
    std::cout << stabedMat << std::endl;
    if(detect_flag){
        cv::Mat image_conved = cv::Mat::zeros(image_ori.rows,image_ori.cols,CV_8UC1);
//        cv::Mat image_conved = cv::Mat::zeros(image_ori.rows,image_ori.cols,CV_8UC3);

        cv::Rect roi(lefttop, cv::Size(rightbottom.x-lefttop.x, rightbottom.y-lefttop.y));
        image_trim = image_ori(roi); // 切り出し画像
        //paste(image_conved,image_trim,pv_cx,pv_cy);
        paste(image_conved,image_trim,lefttop.x,lefttop.y);

        img_msg = igruc.getImageMsg(image_conved);
           if(drawpoint_flag){
           stampText(image_conved,track2);
           }
           cv::imshow("image_conved", image_conved);
           cv::waitKey(1);
    }
    else{
        img_msg = igruc.getImageMsg(image_ori);
    }

    camera_info_ptr = igruc.getInfoMsg();

    img_msg->header.stamp = ros::Time::now();
    camera_info_ptr->header.stamp = img_msg->header.stamp;

    image_pub.publish(img_msg);
    camera_info_pub.publish(camera_info_ptr);

}
void ImageConverter::stampText(cv::Mat dst,Tracking2 track){
    //	std::stringstream ss;
    //	ss << track.id;
    //	cv::String text = ss.str();
    const int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    const double fontscale = 0.5;
    //	int baseline;
    //	cv::Size textsize = cv::getTextSize(text, fontface,
    //																			fontscale, 2, &baseline);
    //
   cv::circle(dst, track.getP1(), 5, cv::Scalar(0,0,200), 3);
   cv::circle(dst, track.getP2(), 5, cv::Scalar(0,0,200), 3);
   cv::circle(dst, track.getP3(), 5, cv::Scalar(0,0,200), 3);
   cv::circle(dst, track.getP4(), 5, cv::Scalar(0,0,200), 3);
    cv::putText(dst, "p1",track.getP1(),
            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);
    cv::putText(dst, "p2",track.getP2(),
            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);
    cv::putText(dst, "p3",track.getP3(),
            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 1);
    cv::putText(dst, "p4",track.getP4(),
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

//    pv_w = std::max({detect.pxdata[0],detect.pxdata[1],detect.pxdata[2],detect.pxdata[3]}) - std::min({detect.pxdata[0],detect.pxdata[1],detect.pxdata[2],detect.pxdata[3]});

//    pv_h = std::max({detect.pydata[0],detect.pydata[1],detect.pydata[2],detect.pydata[3]}) - std::min({detect.pydata[0],detect.pydata[1],detect.pydata[2],detect.pydata[3]});

//    pv_cx = (detect.pxdata[0]+detect.pxdata[1]+detect.pxdata[2]+detect.pxdata[3])/4;

//    pv_cy = (detect.pydata[0]+detect.pydata[1]+detect.pydata[2]+detect.pydata[3])/4;

}


void ImageConverter::TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){

    XmlRpc::XmlRpcValue params;
    nh.getParam("/apriltags3_param", params);

    if(msg->detections.size()){
        detect_flag = true;
        int i,t;

        for(i=0;i<msg->detections.size();i++){
            apriltag_detector.setApriltag(msg->detections[i]);
            drawpoint_flag = true;
        }
        track2.setWindowParam(apriltag_detector,msg);
        std::vector<cv::Point> ltrb = track2.getltrb();
        lefttop = ltrb[0];
        rightbottom = ltrb[1];
        std::cout << lefttop<< rightbottom<< std::endl;


    }
    else{
        detect_flag = false;
        drawpoint_flag = false;
        lefttop.x = 0;
        lefttop.y = 0;
        rightbottom.x = img_size[0];
        rightbottom.y = img_size[1];
    }
}


