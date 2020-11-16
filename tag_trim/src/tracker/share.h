#ifndef SHARE_H
#define SHARE_H



#include <iostream>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "ros/ros.h"

#include <ros/xmlrpc_manager.h>

class Share_Obj{
};
class Share{
    public:
        static std::vector<std::vector<cv::Point>> ltrbs;
        static std::mutex ltrbs_mtx;
        static std::condition_variable ltrbs_cond;
        void wait_for_ltrbs_process();
        void prepare_for_ltrbs_process();
        void prepared_for_ltrbs_process();
        Share(bool Im_client= false);
        Share(Share_Obj &obj);
        ~Share();
        static bool ltrbs_ready;
        static bool ltrbs_locking_now;
        void setParam();
        void getParam();
        static void callback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
    private:
        XmlRpc::XmlRpcValue params, result, payload;
        static Share_Obj share_obj;
};

static Share g_share;
#endif // SHARE_H
