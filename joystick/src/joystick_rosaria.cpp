#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <priorityhandler/PrioMsg.h>

using namespace std;

ros::Publisher pioneer_vel;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //cout << joy->axes[0] << endl; // 1 for left, -1 for right
    //cout << joy->axes[1] << endl; // 1 for top, -1 for bot
    //cout << "-------" << endl;
		//ROS_INFO("I heard: linear_:[%f]   angular_[%d]", joy->axes[1], joy->axes[0]);
    float lr_val = joy->axes[0];
    float tb_val = joy->axes[1];
    geometry_msgs::Twist twist;
    cout << "Received" << endl;
    if (lr_val != 0 || tb_val != 0) {
        twist.angular.z = lr_val/2;
        twist.linear.x = tb_val;

        priorityhandler::PrioMsg prio_msg;
        prio_msg.priority = 5;
        prio_msg.cmd = twist;
        pioneer_vel.publish(prio_msg);
        cout << "Sent" << endl;
    }
}

int main(int argc, char **argv)
{

    // initialize ros
    ros::init(argc, argv, "Joystick_controller");
    ros::NodeHandle n;

    // create publisher

    ros::Publisher cmd_vel_topic = n.advertise<priorityhandler::PrioMsg>("Prio/cmd_vel", 10);

    pioneer_vel = cmd_vel_topic;

    ros::Subscriber joy_command = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    // our loop will publish at 10Hz
    ros::Rate loop_rate(5);

	  ros::spin();
}
