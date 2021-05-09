#include <ros/ros.h>
#include <std_msgs/String.h>

void chatter_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc , argv , "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter",1000, chatter_callback);

    ros::spin();
    /*
    ros::ok() return false, which means ros::shutdown() has been called either by the default cntl-c handler,
    the master telling us to shutdown, or it being called manually.

    */
    return 0;

}