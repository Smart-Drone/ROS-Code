#include "ros/ros.h"

#include "UDPClient.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    UDPClient udp("192.168.1.2", 4210);

    while (ros::ok())
    {
        std::string sensor_data = udp.requestData();
        ROS_INFO("%s", sensor_data.c_str());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
