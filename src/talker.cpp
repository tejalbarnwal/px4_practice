#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>


/*
we can command line argument in cpp
COmmand line arguments are given after the name of the program in command-line shell of operating systems
to pass command line arguments, we typically define main() with two arguments:
first argument- the no. of command line arguments
second - list of command-line arguments

argc -> (argument count) is int and stores no.of command line arguments
passed by the user including the name of the program
so if we pass a value to a program, value of argc would be 2 (one for argument and one for program name)

argv-> (argument vector) array of character pointers listing all the arguments
https://stackoverflow.com/questions/5580761/why-use-double-indirection-or-why-use-pointers-to-pointers
https://www.geeksforgeeks.org/command-line-arguments-in-c-cpp/

*/

int main(int argc, char **argv)
{

    ros::init(argc,argv,"talker");
    /*
    ros::init() func needs to see argc and argv so that it can perform
    any ROS arguments and name remapping that were provided at the command line
    the third argumet is the name of the node
    */

    ros::NodeHandle n;
    /*
    nodehandle is the main access point to communications with the ROS system
    the first nodehandle constructed will fully intialize this node, and the last
    nodehandle destructed will close down the node.

    */

    ros::Publisher chatter_pub;
    chatter_pub = n.advertise<std_msgs::String>("chatter",1000);

    /*
    advertise() returns a publisher object which allows you to
    publish messgaes on that topic through a call to publish()
    once all copies of the returned publisher object are destroyed, the topic
    will be automatically unadvertised

    the second parameter to advertise() is the size of the msg queue
    used for publishing messages. If messages are published more quickly than
    we can send them, the number here specifies how many messages to buffer up before throwing some away

    */

    ros::Rate loop_rate(10);

    int count = 0;
    /*
    a count of how many msgs hv been sent 
    */
    while(ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();


        ROS_INFO("%s",msg.data.c_str());

        chatter_pub.publish(msg);
        /*
        publish() function is how you send the msgs
        the parameter in the msg object
        the type of this object must agree with the 
        type given as a template parameter to the advertise<>() call, as was done in the constructor above.
        */

        ros::spinOnce();
        loop_rate.sleep();
        count++;

    }

    return 0;



}