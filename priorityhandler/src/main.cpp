#include "priorityhandler/includes.h"

void cmd_vel_cb(const priorityhandler::PrioMsg msg) {
    cout << "Message received in cmd_vel_cb" << endl;
}

int main(int argc, char **argv) {
    cout << "Running priorityhandler package" << endl;

    init(argc, argv, "priority_handler");
    NodeHandle nh;

    Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
    Subscriber sub = nh.subscribe("Prio/cmd_vel", 100, cmd_vel_cb);

    Rate rate(10);
    cout << "Starting loop" << endl;
    while(ok()) {
        //geometry_msgs::Twist msg;
        //msg.linear.x = 4;
        //msg.angular.z = 4;
        //pub.publish(msg);
        spinOnce();
        rate.sleep();
    }
    return 0;
}
