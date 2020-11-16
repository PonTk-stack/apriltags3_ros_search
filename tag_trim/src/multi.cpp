#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <boost/thread/thread.hpp>
void do_stuff(int* publish_rate){
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);
  ros::Rate loop_rate(*publish_rate);
  while (ros::ok()){
      std::cout << "************" << std::endl;
    std_msgs::Empty msg;
    pub_b.publish(msg);
    loop_rate.sleep();
  }
}
void do_stuff_a(int* publish_rate){
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);
  ros::Rate loop_rate(*publish_rate);
  while (ros::ok()){
      std::cout << "aaaaaaaaaaaa" << std::endl;
    std_msgs::Empty msg;
    pub_b.publish(msg);
    loop_rate.sleep();
  }
}
int main(int argc, char** argv){
  int rate_b = 1; // 1 Hz
  ros::init(argc, argv, "mt_node");
  // spawn another thread
  boost::thread thread_a(do_stuff_a, &rate_b);
  boost::thread thread_b(do_stuff, &rate_b);
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher pub_a = node->advertise<std_msgs::Empty>("topic_a", 10);
  ros::Rate loop_rate(10); // 10 Hz
  while (ros::ok())
  {
    std_msgs::Empty msg;
    pub_a.publish(msg);
    loop_rate.sleep();
    // process any incoming messages in this thread
    ros::spinOnce();
  }
  // wait the second thread to finish
  thread_a.join();
  thread_b.join();
  return 0;
}
