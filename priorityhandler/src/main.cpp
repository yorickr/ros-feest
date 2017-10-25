#include "priorityhandler/includes.h"

vector<priorityhandler::PrioMsg> messageQueue;

void cmd_vel_cb(const priorityhandler::PrioMsg msg) {
    //cout << "Message received with priority " << msg.priority << endl;
    messageQueue.push_back(msg);
}

bool sortQueue(const priorityhandler::PrioMsg first, const priorityhandler::PrioMsg second) {
    return first.priority > second.priority;
}

int main(int argc, char **argv) {
    cout << "Running priorityhandler package" << endl;

    init(argc, argv, "priority_handler");
    NodeHandle nh;

    Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    Subscriber sub = nh.subscribe("Prio/cmd_vel", 10, cmd_vel_cb);

    Rate rate(30);
    cout << "Starting loop" << endl;

    Time last_msg = Time::now();
    int last_prio = -1;

    while(ok()) {
        // check message queue and publish the highest priority one
        if (messageQueue.size() > 0) {
            sort (messageQueue.begin(), messageQueue.end(), sortQueue);
            priorityhandler::PrioMsg msg = messageQueue.front();

            // If higher priority message
            if (msg.priority >= last_prio) {
                if (msg.priority > last_prio)ROS_INFO("Message with higher priority taking over.");

                last_msg = Time::now();
                last_prio = msg.priority;

                // send it
                cout << "Sending msg with prio " << msg.priority << endl;
                pub.publish(msg.cmd);
            } else if ((Time::now() - last_msg).toSec() >= 3.0) { 
                // Priority is lower, so wait for a 3s timeout in case higher priority messages come in.
                // send it
                last_prio = msg.priority;
                last_msg = Time::now();
                ROS_INFO("Message with lower priority taking over due to 3s time-out.");
                cout << "Sending msg with prio " << msg.priority << endl;
                pub.publish(msg.cmd);

            }
            messageQueue.clear();
        }
        spinOnce();
        rate.sleep();
    }
    return 0;
}
