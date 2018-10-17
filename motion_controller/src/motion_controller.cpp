#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "UDPClient.h"


void sendFlightcommand(std::vector<char> mdr, std::vector<char> mdl, ros::Publisher* cmd_vel_pub);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Rate loop_rate(10);

    UDPClient udp_client("192.168.1.2", "192.168.1.3", 4210);
    udp_client.requestData();

    while (ros::ok())
    {
        std::vector<char> mdr = udp_client.readData(MOTION_DATA_RIGHT);
        ROS_INFO("RIGHT:%d#%d", mdr[1], mdr[2]);

        std::vector<char> mdl = udp_client.readData(MOTION_DATA_LEFT);
        ROS_INFO("LEFT:%d#%d", mdl[1], mdl[2]);

        sendFlightcommand(mdr, mdl, &cmd_vel_pub);

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
