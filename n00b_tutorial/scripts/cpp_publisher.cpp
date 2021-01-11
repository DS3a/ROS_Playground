#include "ros/ros.h" 
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sender");
    ros::NodeHandle n;
    ros::Publisher publisher =  n.advertise<std_msgs::String>("Chat_topic", 1000);
    ros::Rate rate_mod(0.25);

    int count = 0;
    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello From CPP to python : " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        publisher.publish(msg);

        ros::spinOnce();

        rate_mod.sleep();
        ++count;
    }

    return 0;
}
