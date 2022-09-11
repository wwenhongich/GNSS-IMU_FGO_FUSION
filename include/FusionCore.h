#ifndef FUSION_CORE_H
#define FUSION_CORE_H
#include <boost/lockfree/queue.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#if GTSAM_USE_MUTEX
#include <mutex>
#endif
#include <algorithm>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "Measurement.h"

using namespace gtsam;
using namespace std;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace GTSAM {

class CalibrationParams {
public:
    double Position_tx; // tx ty tz
    double Position_ty;
    double Position_tz;
    double Position_rx; // rx ry rz
    double Position_ry;
    double Position_rz;
    // NoiseParams
    double accelerometer_sigma;
    double gyroscope_sigma;
    double integration_sigma;
    double accelerometer_bias_sigma;
    double gyroscope_bias_sigma;
    double average_delta_t;

    CalibrationParams();
    CalibrationParams(const Vector12& calibration);
};

class FUSION_CORE {
public:
    void initialize(Vector12& calibration, Vector3& position);
    Pose3 getCurPose();
    Values getResult();
    Matrix getMarginalPoseCov();

    // x, y, z
    void addGPS(shared_ptr<PoseMeasurement> ptr);
    // quaternion(qx,qy,qz,qw), anglular velocity(omegax, omegay, omegaz), linear acceleration(accx, accy, accz)
    void addIMU(shared_ptr<ImuMeasurement> ptr);
    void addVins(shared_ptr<PoseMeasurement> ptr);

private:
    double t1;
    size_t included_imu_measurement_count;
    double g_;
    uint64_t key;
    uint64_t key_store;
    int gps_skip;
    uint64_t estimationPose_count;

    boost::shared_ptr<PreintegrationParams> imuPreintegrationParams;
    boost::shared_ptr<PreintegratedImuMeasurements> ImuPreIntegrated;
    noiseModel::Diagonal::shared_ptr noise_model_gps;
    imuBias::ConstantBias current_bias;
    Pose3 current_pose_global;
    Matrix current_pose_cov;
    Vector3 current_velocity_global;
    CalibrationParams Calibration_Params;
    Vector4 orientation;

    ISAM2 isam2;
    Values result;
    // Create the factor graph and values object that will store new factors and values to add to the incremental graph
    NonlinearFactorGraph NewFactorGraph;
    Values new_values;
};

} // end of namespace GTSAM

#endif
