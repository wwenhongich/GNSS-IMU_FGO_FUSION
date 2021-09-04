#include "Measurement.h"
using namespace std;

// Construct Empty Measurement
Measurement::Measurement() {
    mytime= 0;
    mytype = EMPTY;
}
double Measurement::getTime() { return mytime; }
MeasurementType Measurement::getType() { return mytype; }

// Construct IMU measurement
ImuMeasurement::ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg) {
    mytime= msg->header.stamp.toSec();
    mydata << msg->angular_velocity.x, 
             msg->angular_velocity.y, 
             msg->angular_velocity.z,
             msg->linear_acceleration.x, 
             msg->linear_acceleration.y, 
             msg->linear_acceleration.z;
    myori << msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w;
    mytype = IMU;
}
Eigen::VectorXd ImuMeasurement::getData() { return mydata; }
Eigen::VectorXd ImuMeasurement::getOri() { return myori; }

// Construct GPS measurement
GpsMeasurement::GpsMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    mytime= msg->header.stamp.toSec();
    mydata << msg->latitude, 
             msg->longitude, 
             msg->altitude;
    mytype = GPS;
}
Eigen::VectorXd GpsMeasurement::getData() { return mydata; }

// Construct Pose measurement
PoseMeasurement::PoseMeasurement(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    mytime= msg->header.stamp.toSec();
    mydata << msg->pose.pose.position.x, 
             msg->pose.pose.position.y, 
             msg->pose.pose.position.z;
    myori << msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w;
    mytype = POSE;
}


Eigen::VectorXd PoseMeasurement::getData() { return mydata; }




