#include "planner.h"
#include "backward.hpp"


namespace backward {
    backward::SignalHandling sh;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh("~");
    Planner planner;
    planner.init(nh);
    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}