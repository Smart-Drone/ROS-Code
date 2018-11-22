#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>
#include <stdio_ext.h>

int getch();
void sendFlightCommand(int pressed_key, ros::Publisher* cmd_vel_pub);
void sendEmptyMessage(ros::Publisher* ardrone_pub, ros::Publisher* cmd_vel_pub, float duration);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "key_input_node");
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	ros::Publisher ardrone_takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 100);
	ros::Publisher ardrone_land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 100);
	ros::Publisher ardrone_reset_pub = n.advertise<std_msgs::Empty>("ardrone/reset", 100);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		int c = getch();
		switch(c)
		{
			case 'w':
			ROS_INFO("Ascent");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 's':
			ROS_INFO("Descent");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 'a':
			ROS_INFO("Rotate left");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 'd':
			ROS_INFO("Rotate right");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 'i':
			ROS_INFO("Fly forwards");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 'k':
			ROS_INFO("Fly backwards");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 'j':
			ROS_INFO("Fly left");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case 'l':
			ROS_INFO("Fly right");
			sendFlightCommand(c, &cmd_vel_pub);
			break;

			case '1':
			ROS_INFO("Take off");
			sendEmptyMessage(&ardrone_takeoff_pub, &cmd_vel_pub, 3);
			break;

			case '2':
			ROS_INFO("Land");
			sendEmptyMessage(&ardrone_land_pub, &cmd_vel_pub, 10);
			break;

			case '3':
			ROS_INFO("Emergency shutdown");
			sendEmptyMessage(&ardrone_reset_pub, &cmd_vel_pub, 0);
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);
	int c = getchar();
	__fpurge(stdin);
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
	return c;
}

void sendFlightCommand(int pressed_key, ros::Publisher* cmd_vel_pub){
	geometry_msgs::Twist msg;

	if(pressed_key == 'w')msg.linear.z = 1.0;
	else if (pressed_key == 's')msg.linear.z = -1.0;
	else if (pressed_key == 'a')msg.angular.z = 1.0;
	else if (pressed_key == 'd')msg.angular.z = -1.0;
	else if (pressed_key == 'i')msg.linear.x = 1.0;
	else if (pressed_key == 'k')msg.linear.x = -1.0;
	else if (pressed_key == 'j')msg.linear.y = 1.0;
	else if (pressed_key == 'l')msg.linear.y = -1.0;

	cmd_vel_pub->publish(msg);
	ros::Duration(0.1).sleep();

	geometry_msgs::Twist hover_msg;
	cmd_vel_pub->publish(hover_msg);
}

void sendEmptyMessage(ros::Publisher* ardrone_pub, ros::Publisher* cmd_vel_pub, float duration){
	std_msgs::Empty msg;
	ardrone_pub->publish(msg);
	ros::Duration(duration).sleep();
	geometry_msgs::Twist hover_msg;
	cmd_vel_pub->publish(hover_msg);
}
