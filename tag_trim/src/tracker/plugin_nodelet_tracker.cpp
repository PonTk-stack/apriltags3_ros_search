#include "plugin_nodelet_tracker.h"

#include <pluginlib/class_list_macros.h>




namespace nodelet_tracker{
    Tracker::Tracker(){
    }

    Tracker::~Tracker(){
    }

    void Tracker::onInit(){
        nh = getMTNodeHandle();
        pnh = getPrivateNodeHandle();


        tag_detection_sub = nh.subscribe("tag_topic", 10, &Tracker::TagDetectCallback,this);
        //tag_detection_sub = nh.subscribe("info_topic", 10, &Tracker::InfoCallback,this);
        camera_info_sub = nh.subscribe("info_topic", 10, &Tracker::InfoCallback,this);
    }

    void Tracker::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info){
        cam.setICP(info);
    }

    void Tracker::TagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){
        std::cout << "1: start tracker tag callback" << std::endl;
        //g_share.prepare_for_ltrbs_process();
        share->ltrbs_ready=false;
        ros::param::set("ltrbs_ready",share->ltrbs_ready );
        TagDetectProcess(msg);
        share->ltrbs_ready=true;
        //g_share.prepared_for_ltrbs_process();
        ros::param::set("ltrbs_ready",share->ltrbs_ready );
        std::cout << "2: end tracker tag callback" << std::endl;
    }

    void Tracker::TagDetectProcess(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){
        //std::cout << msg->detections.size() << std::endl;
        Camera cam;
        std::vector<double> infoA;
        ros::param::get("/info_param/A", infoA);
        Eigen::Matrix<double,3,3> A;
        if(!continuation_flag){
            count = 0;
            count_detected = 0;
            count_err = 0;
            apriltag_detector.resetApriltagVel();
        }
        //XmlRpc::XmlRpcValue params;
        //nh.getParam("/apriltags3_param", params);


        ltrbs.clear();
        if(msg->detections.size()){
            track2.onMsgProcessing();
            int i,t;
            for(i=0;i<msg->detections.size();i++){
                apriltag_detector.setApriltag(msg->detections[i]);
                //drawpoint_flag = true;
            }
            track2.setWindowParam(apriltag_detector,msg);
            ltrb = track2.getltrb();
            //ltrbs.push_back(ltrb);
            //share.ltrbs = ltrbs;
            share->ltrbs.push_back(ltrb);
            //lefttop = ltrb[0];
            //rightbottom = ltrb[1];
            //
            box.top_left.x = ltrb[0].x;
            box.top_left.y = ltrb[0].y;
            box.right_bottom.x = ltrb[1].x;
            box.right_bottom.y = ltrb[1].y;

            boxes.ltrbs.push_back(box);
            //
            continuation_flag = true;
            //計測
            count_pure_pixel = track2.uv_apriltag.getPurePixelSize();
            if(continuation_flag){
                count++;
                count_detected++;
            }
        }
        else{
            track2.noMsgProcessing();
            apriltag_detector.resetApriltagVel();
            //lefttop.x = 0;
            //lefttop.y = 0;
            //rightbottom.x = igruc.getWindowWidth();//img_size[0];
            //rightbottom.y = igruc.getWindowHeight();//img_size[1];
            //計測
            count_pure_pixel = 0;
            if(continuation_flag){
                count++;
                count_err++;
            }
        }
#if 0 //loop cam data
        if(bcc.need_switch_fase() && count>0){
            float no_detected_rate = (float)count_err / count ;
            csvm.csv.newRow() << count <<  count_detected<< ros_duration <<  no_detected_rate << track2.getK_safe()
                << track2.getK_uv_vel() << track2.getK_tag_vel()
                <<(rightbottom.x-lefttop.x)*(rightbottom.y-lefttop.y)
                << count_pure_pixel;
            //計測パラメータ　リセット
            continuation_flag = false;
            measure_param_update();
        }
#endif
#if 0 //not loop cam data
        if(count>0){
            float no_detected_rate = (float)count_err / count ;
            csvm.csv.newRow() << count << count_detected<< ros_duration <<  no_detected_rate << track2.getK_safe()
                << track2.getK_uv_vel() << track2.getK_tag_vel()
                <<(rightbottom.x-lefttop.x)*(rightbottom.y-lefttop.y)
                << count_pure_pixel;
            if(bcc.need_switch_fase()){
                //計測パラメータ　リセット
                continuation_flag = false;
                measure_param_update();
            }
        }
#endif
    }

    void Tracker::TagDetectedInit(){



    }



    void MeasuringInstrument::detected(){
            if(run){
                count++;
                count_detected++;
            }
    }
    void MeasuringInstrument::nondetected(){
            if(run){
                count++;
                count_err++;
            }
    }
    void MeasuringInstrument::reset(){
            run = false;
            count = 0;
            count_detected = 0;
            count_err = 0;
            count_pure_pixel = 0;
    }

}
PLUGINLIB_EXPORT_CLASS(nodelet_tracker::Tracker , nodelet::Nodelet)
