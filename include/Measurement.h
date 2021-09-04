
#ifndef MEASUREMENmytimeH
#define MEASUREMENmytimeH 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_listener.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

enum MeasurementType {EMPTY, IMU, GPS, POSE};


class Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        double getTime();
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        double mytime;
        MeasurementType mytype;
};

struct MeasurementCompare {
  bool operator()(const std::shared_ptr<Measurement> lhs, const std::shared_ptr<Measurement> rhs) const {
    return lhs->getTime() > rhs->getTime();
  }
};

//IMU
class ImuMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
        Eigen::VectorXd getData();
        Eigen::VectorXd getOri();

    private: 
        Eigen::Matrix<double,6,1> mydata;    // w a
        Eigen::Matrix<double,4,1> myori; // x y z w
};

//GPS
class GpsMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GpsMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);
        Eigen::VectorXd getData();
        Eigen::VectorXd getOri();

    private: 
        Eigen::Matrix<double,3,1> mydata;
};
//Pose
class PoseMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PoseMeasurement(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        Eigen::VectorXd getData();

    private: 
        Eigen::Matrix<double,3,1> mydata;    // x y z
        Eigen::Matrix<double,4,1> myori;     // x y z w
};


#endif 
