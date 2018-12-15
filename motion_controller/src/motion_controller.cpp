#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

#include "UDPClient.h"


const std::string IP_MCR = "10.0.0.2";   // IP address for right motion controller
const std::string IP_MCL = "10.0.0.3";   // IP address for lef motion controller
constexpr unsigned int UDP_PORT_MCL = 4220;  // UDP port for left motion controller
constexpr unsigned int UDP_PORT_MCR = 4210;  // UDP port for right motion controller


void sendFlightcommand(std::vector<char> mdr, std::vector<char> mdl, ros::Publisher* cmd_vel_pub);
void sendEmptyMessage(ros::Publisher* ardrone_pub);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Publisher ardrone_takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 100);
	ros::Publisher ardrone_land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 100);
	ros::Publisher ardrone_reset_pub = n.advertise<std_msgs::Empty>("ardrone/reset", 100);
    ros::Rate loop_rate(10);

    boost::asio::io_service io_service;
    UDPClient udp_client_mcr(IP_MCR, UDP_PORT_MCR, io_service);
    UDPClient udp_client_mcl(IP_MCL, UDP_PORT_MCL, io_service);
    udp_client_mcr.requestData();
    udp_client_mcl.requestData();

    while (ros::ok())
    {
        io_service.poll();

        std::vector<char> data_right = udp_client_mcr.readData();
        std::vector<char> data_left = udp_client_mcl.readData();

        if (data_right[0] == RESET_CMD) {
            sendEmptyMessage(&ardrone_reset_pub);
            udp_client_mcr.requestData();
            ROS_INFO("SENDING RESET MSG");
        }
        if (data_left[0] == TAKEOFF_CMD) {
            sendEmptyMessage(&ardrone_takeoff_pub);
            udp_client_mcl.requestData();
            ROS_INFO("SENDING TAKEOFF MSG");
        }
        else if (data_left[0] == LAND_CMD) {
            sendEmptyMessage(&ardrone_land_pub);
            udp_client_mcl.requestData();
            ROS_INFO("SENDING LAND MSG");
        }
        else if (data_right[0] == MOTION_DATA && data_left[0] == MOTION_DATA) {
            sendFlightcommand(data_right, data_left, &cmd_vel_pub);
            ROS_INFO("MOTION DATA LEFT: %d %d", data_left[1], data_left[2]);
            ROS_INFO("MOTION DATA RIGHT: %d %d", data_right[1], data_right[2]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void sendFlightcommand(std::vector<char> mdr, std::vector<char> mdl, ros::Publisher* cmd_vel_pub) {
    geometry_msgs::Twist msg;

    // Mapping pitch_left to linear_z
    int pitch_left = mdl[1];
    if (pitch_left >= -90 && pitch_left < -25) msg.linear.z = -1.0;
    else if(pitch_left >= -25 && pitch_left <= 25) msg.linear.z = 0.0;
    else if(pitch_left > 25 && pitch_left <= 90) msg.linear.z = 1.0;

    // Mapping roll_left to angular_z
    int roll_left = mdl[2];
    if (roll_left >= -90 && roll_left < -10) msg.angular.z = 1.0;
    else if(roll_left >= -10 && roll_left <= 10) msg.angular.z = 0.0;
    else if(roll_left > 10 && roll_left <= 90) msg.angular.z = -1.0;

    // Mapping pitch_right to linear_x
    int pitch_right = mdr[1];
    if (pitch_right >= -90 && pitch_right < -25) msg.linear.x = 1.0;
    else if(pitch_right >= -25 && pitch_right <= 25) msg.linear.x = 0.0;
    else if(pitch_right > 25 && pitch_right <= 90) msg.linear.x = -1.0;

    // Mapping roll_right to linear_Y
    int roll_right = mdr[2];
    if (roll_right >= -90 && roll_right < -25) msg.linear.y = 1.0;
    else if(roll_right >= -25 && roll_right <= 25) msg.linear.y = 0.0;
    else if(roll_right > 25 && roll_right <= 90) msg.linear.y = -1.0;

    cmd_vel_pub->publish(msg);
}

void sendEmptyMessage(ros::Publisher* ardrone_pub) {
    std_msgs::Empty msg;
    ardrone_pub->publish(msg);
    //ros::Duration(3).sleep();
}
