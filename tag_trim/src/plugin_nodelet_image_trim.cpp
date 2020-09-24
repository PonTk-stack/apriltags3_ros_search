#include "plugin_nodelet_image_trim.h"

#include <pluginlib/class_list_macros.h>


namespace nodelet_image_trim{

    ImageConverter::ImageConverter(){
        //csvm.newFile();
        csvm.newFile();
        csvm.csv.newRow() << "detect" << "K_safe" << "K_uv_vel" << "K_tag_vel"
            << "pixel";

        
    }
    ImageConverter::~ImageConverter(){
        csvm.write();
    };

    void ImageConverter::onInit(){

        nh = getMTNodeHandle();
        pnh = getPrivateNodeHandle();

        //ros::init(argc, argv,node_name);

        image_transport::ImageTransport it(nh);


        image_pub = it.advertise("image_trim_node/image_trimmed", 10);
        camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("image_trim_node/camera_info", 10);

        image_sub_mf.subscribe(nh,"image_topic", 1);
        info_sub_mf.subscribe(nh,"camera_info_topic", 1);


        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10) , image_sub_mf,info_sub_mf); 
        sync->registerCallback(boost::bind(&ImageConverter::imgconvCallback ,this, _1, _2));

        //tag_detection_sub = nh.subscribe("tag_topic", 10, &ImageConverter::TagDetectCallback,this);
        tag_detection_sub = nh.subscribe("tag_topic", 10, &ImageConverter::TagDetectCallback,this);


        lefttop.x = 0;
        lefttop.y = 0;
        rightbottom.x = igruc.getWindowWidth();//img_size[0];
        rightbottom.y = igruc.getWindowHeight();//img_size[1];
ready_exc = true;
    }

    void ImageConverter::imgconvCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfo::ConstPtr &info){

        if(ready_exc)
        {
            start = std::chrono::system_clock::now();
        ready_exc = false;
        bool f = igruc.grabFrame(msg ,info);
        igruc.getImage(image_ori);

        //cv::Mat stabedMat;
        //igruc.getSmoothedImageAndMat(image_ori,stabedMat);

            if(detect_flag){
                image_conved = cv::Mat::zeros(image_ori.rows,image_ori.cols,CV_8UC1);
        //        cv::Mat image_conved = cv::Mat::zeros(image_ori.rows,image_ori.cols,CV_8UC3);

                cv::Rect roi(lefttop, cv::Size(rightbottom.x-lefttop.x, rightbottom.y-lefttop.y));
                image_trim = image_ori(roi); // 切り出し画像
                //paste(image_conved,image_trim,lefttop.x,lefttop.y);
                image_trim.copyTo(image_conved(cv::Range(lefttop.y,rightbottom.y),
                                                cv::Range(lefttop.x,rightbottom.x ) ));

                img_msg = igruc.getImageMsg(image_conved);
                   if(drawpoint_flag){
                       stampText(image_conved,track2);
                   }
                   //cv::imshow("image_conved", image_conved);
                   //cv::waitKey(1);
            }
            else{
                img_msg = igruc.getImageMsg(image_ori);
            }
            camera_info_ptr = igruc.getInfoMsg();

            img_msg->header.stamp = ros::Time::now();
            camera_info_ptr->header.stamp = img_msg->header.stamp;

            image_pub.publish(img_msg);
            camera_info_pub.publish(camera_info_ptr);

            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            timer += (end - start);
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(timer).count()
                << "ms: "<< count << " , " << count_err  << std::endl;
        }
    }


    void ImageConverter::stampText(cv::Mat dst,Tracking2 track){
        const int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        const double fontscale = 0.5;

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

    void ImageConverter::TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){

        {
            //XmlRpc::XmlRpcValue params;
            //nh.getParam("/apriltags3_param", params);
            if(msg->detections.size()){
                detect_flag = true;
                int i,t;
                count++;
                for(i=0;i<msg->detections.size();i++){
                    apriltag_detector.setApriltag(msg->detections[i]);
                    //drawpoint_flag = true;
                }
                track2.setWindowParam(apriltag_detector,msg);
                ltrb = track2.getltrb();
                lefttop = ltrb[0];
                rightbottom = ltrb[1];
            }
            else{
                apriltag_detector.resetApriltagVel();
                if(count > 0) count_err++;
                detect_flag = false;
                drawpoint_flag = false;
                lefttop.x = 0;
                lefttop.y = 0;
                rightbottom.x = igruc.getWindowWidth();//img_size[0];
                rightbottom.y = igruc.getWindowHeight();//img_size[1];
            }
            csvm.csv.newRow() << detect_flag << track2.getK_safe()
                    << track2.getK_uv_vel() << track2.getK_tag_vel()
                    <<(rightbottom.x-lefttop.x)*(rightbottom.y-lefttop.y) ;
        }
        ready_exc = true;
    }
}
PLUGINLIB_EXPORT_CLASS(nodelet_image_trim::ImageConverter , nodelet::Nodelet)
