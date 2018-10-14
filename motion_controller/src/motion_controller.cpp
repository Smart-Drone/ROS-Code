#include "ros/ros.h"

#include "UDPClient.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    UDPClient udp_client("192.168.1.2", "192.168.1.3", 4210);
    udp_client.requestData();

    while (ros::ok())
    {
        std::vector<char> mdr = udp_client.readData(MOTION_DATA_RIGHT);
        ROS_INFO("RIGHT:%d#%d", mdr[1], mdr[2]);

        std::vector<char> mdl = udp_client.readData(MOTION_DATA_LEFT);
        ROS_INFO("LEFT:%d#%d", mdl[1], mdl[2]);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
