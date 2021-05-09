#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;

void state_sub_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_sub_callback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_1;
    pose_1.pose.position.x = 0;
    pose_1.pose.position.y = 0;
    pose_1.pose.position.z = 2;

    for(int i = 100; ros::ok() && i>0; --i){
        local_pos_pub.publish(pose_1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                        ROS_INFO("OFFBOARD ENABLED");
                    }

                    last_request = ros::Time::now();
            }
        else{
            if(!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if(arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                            ROS_INFO("vehicle armed");
                        }
                        last_request = ros::Time::now();
                }
                local_pos_pub.publish(pose_1);
                ros::spin();
                rate.sleep();
        }    
        return 0;
    }




}


