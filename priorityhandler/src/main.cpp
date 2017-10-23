#include "priorityhandler/includes.h"

vector<priorityhandler::PrioMsg> messageQueue;

void cmd_vel_cb(const priorityhandler::PrioMsg msg) {
    cout << "Message received with priority " << msg.priority << endl;
    messageQueue.push_back(msg);
}

bool sortQueue(const priorityhandler::PrioMsg first, const priorityhandler::PrioMsg second) {
    return first.priority > second.priority;
}

int main(int argc, char **argv) {
    cout << "Running priorityhandler package" << endl;

    init(argc, argv, "priority_handler");
    NodeHandle nh;

    Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
    Subscriber sub = nh.subscribe("Prio/cmd_vel", 100, cmd_vel_cb);

    Rate rate(20);
    cout << "Starting loop" << endl;
    while(ok()) {
        //geometry_msgs::Twist msg;
        //msg.linear.x = 4;
        //msg.angular.z = 4;
        //pub.publish(msg);
        // check message queue every 50ms and publish the highest priority one
        if (messageQueue.size() > 0) {
            sort (messageQueue.begin(), messageQueue.end(), sortQueue);
            for (int i = 0; i < messageQueue.size(); i++) {
                auto msg = messageQueue[i];
                cout << "Prio " << msg.priority << endl;
            }
            // send it
            priorityhandler::PrioMsg msg = messageQueue.front();
            pub.publish(msg.cmd);

            messageQueue.clear();
        }
        spinOnce();
        rate.sleep();
    }
    return 0;
}
