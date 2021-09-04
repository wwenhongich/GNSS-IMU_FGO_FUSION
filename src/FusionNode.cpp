#include "ros/ros.h"
#include "TransformFusion.h"
#include <chrono>

using namespace std;

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "FusionNode");
    ros::NodeHandle n;    
    ROS_INFO("\033[1;32m----> IMU GPS Fusion Started.\033[0m");
    // Initialize  ROS wrapper
    TransformFusion TF(n);
    TF.init();
    TF.run();

    return 0;
}
