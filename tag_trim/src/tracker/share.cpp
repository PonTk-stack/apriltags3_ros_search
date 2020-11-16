#include "share.h"
std::vector<std::vector<cv::Point>> Share::ltrbs;
std::mutex Share::ltrbs_mtx;
bool Share::ltrbs_ready;
        bool Share::ltrbs_locking_now;
        std::condition_variable Share::ltrbs_cond;

Share::Share(bool Im_client){

    if(Im_client){
        ros::XMLRPCManager::instance()->unbind("paramUpdate");
        ros::XMLRPCManager::instance()->bind("paramUpdate", this->callback);
        params[0] = ros::this_node::getName();
        params[1] = ros::XMLRPCManager::instance()->getServerURI();
        params[2] = ros::names::resolve(std::string("ltrbs_ready"));

        if (ros::master::execute("subscribeParam", params, result, payload, false)) {
            ROS_INFO("Subscribed to parameter.");
        }
        else {
            ROS_ERROR("Failed to subscribe to the parameter.");
        }
    }

    ltrbs_ready = false;
    ros::param::set("ltrbs_ready", ltrbs_ready  );
}
Share::Share(Share_Obj& obj){
        share_obj = obj;
        ros::XMLRPCManager::instance()->unbind("paramUpdate");
        ros::XMLRPCManager::instance()->bind("paramUpdate", this->callback);
        params[0] = ros::this_node::getName();
        params[1] = ros::XMLRPCManager::instance()->getServerURI();
        params[2] = ros::names::resolve(std::string("ltrbs_ready"));
        if (ros::master::execute("subscribeParam", params, result, payload, false)) {
            ROS_INFO("Subscribed to parameter.");
        }
        else {
            ROS_ERROR("Failed to subscribe to the parameter.");
        }
    ltrbs_ready = false;
    ros::param::set("ltrbs_ready", ltrbs_ready  );
}

Share::~Share(){
}

void Share::wait_for_ltrbs_process(){
    std::unique_lock<std::mutex> lk(ltrbs_mtx);
    ltrbs_cond.wait(lk, [this] {return ltrbs_ready;});
    ltrbs_ready = false;

    ros::param::set("ltrbs_ready", ltrbs_ready  );
}
void Share::prepare_for_ltrbs_process(){
    std::lock_guard<std::mutex> lk(ltrbs_mtx);
    ltrbs_ready = true;
    ros::param::set("ltrbs_ready", ltrbs_ready  );
}
void Share::prepared_for_ltrbs_process(){
    ltrbs_cond.notify_one();
}
void Share::setParam(){
   // ros::param::set("info_param",   );
}
void Share::callback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    if(ltrbs_locking_now){
        ltrbs_mtx.unlock();
        ros::param::set("/ltrbs_locking_now",false );
    }
    //ROS_ERROR("Updated parameters!");
}

