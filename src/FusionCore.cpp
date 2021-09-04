//time: 2021.1.11  
//author: Wu Wenhong
#include "FusionCore.h"
using namespace gtsam;
using namespace std;

namespace GTSAM {
CalibrationParams::CalibrationParams() : Position_tx(0),
                                         Position_ty(0),
                                         Position_tz(0),
                                         Position_rx(0),
                                         Position_ry(0),
                                         Position_rz(0),
                                         accelerometer_sigma(0.001),//0.01
                                         gyroscope_sigma(0.000175),
                                         integration_sigma(0),
                                         accelerometer_bias_sigma(0.000167),
                                         gyroscope_bias_sigma(2.91e-006),
                                         average_delta_t(0.0100395199348279) 
                                         {}

CalibrationParams::CalibrationParams(const Vector12& calibration) : Position_tx(calibration(0)),
                                                                    Position_ty(calibration(1)),
                                                                    Position_tz(calibration(2)),
                                                                    Position_rx(calibration(3)),
                                                                    Position_ry(calibration(4)),
                                                                    Position_rz(calibration(5)),
                                                                    accelerometer_sigma(calibration(6)),
                                                                    gyroscope_sigma(calibration(7)),
                                                                    integration_sigma(calibration(8)),
                                                                    accelerometer_bias_sigma(calibration(9)),
                                                                    gyroscope_bias_sigma(calibration(10)),
                                                                    average_delta_t(calibration(11)) 
                                                                    {}
//初始化核心
void FUSION_CORE::initialize(Vector12& calibration, Vector3& position) {
//setting CalibrationParams
	Calibration_Params = CalibrationParams(calibration);
//BasicPosition
    Vector6 Basic_Position = (Vector6() << Calibration_Params.Position_tx, Calibration_Params.Position_ty, Calibration_Params.Position_tz,
                                  Calibration_Params.Position_rx, Calibration_Params.Position_ry, Calibration_Params.Position_rz)
                    .finished();


    // Configure different variables
    t1 = 0;
    key = 1;
    key_store = 1;
  	estimationPose_count = 1;
// Skip  GPS measurements each time
    gps_skip = 1;  
    g_ = 9.81; // 0 for reorded rosbag data, 9.81 for vrx simulation
    auto w_coriolis = Vector3::Zero();  // zero vector
    orientation = Vector4::Zero();

  //先验观测协方差(GPS) Pose Xi
    noise_model_gps = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                     Vector3::Constant(0.3))//1/0.0001 1/0.01 0.2 tx ty tz
                                                            .finished());//0.65 0.01

//intialization
    current_pose_global = Pose3(Rot3(), position);// 0 0 0
    current_velocity_global = Vector3::Zero();//0 0 0
    current_bias = imuBias::ConstantBias();  // 0

 //先验运动协方差(IMU)                       
    auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                       Vector3::Constant(1.0e-6))  
                                                         .finished());
    auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1));// 10m/s 0.9^2 1^2
    auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100),
                                                                   Vector3::Constant(5.00e-05))
                                                     .finished());
// Set IMU preintegration parameters
    Matrix33 measured_acc_cov = I_3x3 * pow(Calibration_Params.accelerometer_sigma, 5);
    Matrix33 measured_omega_cov = I_3x3 * pow(Calibration_Params.gyroscope_sigma, 5);
// error committed in integrating position from velocities  
    Matrix33 integration_error_cov = I_3x3 * pow(Calibration_Params.integration_sigma, 5);
//setting imuPreintegrationParams
    imuPreintegrationParams = boost::shared_ptr<PreintegrationParams>(PreintegrationParams::MakeSharedU(g_));

    imuPreintegrationParams->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    imuPreintegrationParams->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    imuPreintegrationParams->gyroscopeCovariance = measured_omega_cov;       // gyro white noise in continuous
    imuPreintegrationParams->omegaCoriolis = w_coriolis;

    ImuPreIntegrated = boost::shared_ptr<PreintegratedImuMeasurements>(
                                        new PreintegratedImuMeasurements(imuPreintegrationParams, current_bias));

    //iSAM2 参数设置
    ISAM2Params isam_params;
    //因子图优化方式 高斯牛顿法
    isam_params.optimizationParams=ISAM2GaussNewtonParams();
    //因子分解方式QR
    isam_params.factorization = ISAM2Params::QR;    //ISAM2Params::CHOLESKY;
    //再线性化间隔 
    isam_params.relinearizeSkip = 2;
    //再线性化阈值
    isam_params.relinearizeThreshold = 0.01;
    //将参数写入isam2
    isam2 = ISAM2(isam_params);
}

//主节点
void FUSION_CORE::addGPS(shared_ptr<PoseMeasurement> ptr) {
//No IMU update
    if (fabs(orientation[0]) < 1e-5 && fabs(orientation[1]) < 1e-5 && fabs(orientation[2]) < 1e-5 && fabs(orientation[3]) < 1e-5) {
        return;
    }   

    double t = ptr -> getTime();
    Vector3 GpsData = ptr -> getData();

//定义Xi Xi-1
    auto current_pose_key = X(key);
    auto current_vel_key = V(key);
    auto current_bias_key = B(key);
    auto previous_pose_key = X(key - 1);
    auto previous_vel_key = V(key - 1);
    auto previous_bias_key = B(key - 1);

//0.初始迭代   

    if (key <= 1) {
//IMU协方差        
        auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0), Vector3::Constant(1.0e-6)) .finished());
        auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1.96*1.96));// 1.96m/s
        auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100), Vector3::Constant(1.00e-05)).finished());

    	current_pose_global = Pose3(Rot3(Quaternion(orientation[3], orientation[0], orientation[1], orientation[2])), GpsData);
    //Value insert
        new_values.insert(current_pose_key, current_pose_global);
        new_values.insert(current_vel_key, current_velocity_global);
        new_values.insert(current_bias_key, current_bias);

    //NewFactorGraph insert
        NewFactorGraph.emplace_shared<PriorFactor<Pose3>>(current_pose_key, current_pose_global, sigma_init_x);
        NewFactorGraph.emplace_shared<PriorFactor<Vector3>>(current_vel_key, current_velocity_global, sigma_init_v);
        NewFactorGraph.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key, current_bias, sigma_init_b);


    //ImuPreIntegrated
        ImuPreIntegrated = boost::shared_ptr<PreintegratedImuMeasurements>(new PreintegratedImuMeasurements(imuPreintegrationParams, current_bias));
         included_imu_measurement_count = 0;

        ++key;
        return;
    }
  

    //1.观测迭代
    // Create IMU factor
    NewFactorGraph.emplace_shared<ImuFactor>(previous_pose_key, previous_vel_key,
                                          current_pose_key, current_vel_key,
                                          previous_bias_key, *ImuPreIntegrated);
    ImuPreIntegrated = boost::shared_ptr<PreintegratedImuMeasurements>(
                                        new PreintegratedImuMeasurements(imuPreintegrationParams, current_bias));
    
    // Bias evolution as given in the IMU metadata
    auto sigma_between_b = noiseModel::Diagonal::Sigmas((Vector6() <<
            Vector3::Constant(sqrt(included_imu_measurement_count) * Calibration_Params.accelerometer_bias_sigma),
            Vector3::Constant(sqrt(included_imu_measurement_count) * Calibration_Params.gyroscope_bias_sigma))
            .finished());


    NewFactorGraph.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(previous_bias_key,
                                                                     current_bias_key,
                                                                     imuBias::ConstantBias(),
                                                                     sigma_between_b);
    included_imu_measurement_count = 0; // set the counter to 0

 // Create GPS factor(观测)
    auto gps_pose = Pose3(current_pose_global.rotation(), GpsData);

    	NewFactorGraph.emplace_shared<PriorFactor<Pose3>>(current_pose_key, gps_pose, noise_model_gps);
        new_values.insert(current_pose_key, gps_pose);

    new_values.insert(current_vel_key, current_velocity_global);
    new_values.insert(current_bias_key, current_bias);

//3.Optimize
     if (key > 1 + 2 * gps_skip){
   
         //  isam2 Optimize
	    isam2.update(NewFactorGraph, new_values);

        isam2.update();

       // printf("################ UPDATED ################\n");
       // printf("\n\n");

        NewFactorGraph.resize(0);
        new_values.clear();

        //printf("################ CLEARED ################\n");
        //printf("\n\n");
    	result = isam2.calculateEstimate(); 


      	current_pose_global = result.at<Pose3>(current_pose_key);
		current_velocity_global = result.at<Vector3>(current_vel_key);
		current_bias = result.at<imuBias::ConstantBias>(current_bias_key);

       //printf("################ CALCULATED Estimate ################\n");
      // printf("\n\n");

		current_pose_global = result.at<Pose3>(current_pose_key);
		current_velocity_global = result.at<Vector3>(current_vel_key);
		current_bias = result.at<imuBias::ConstantBias>(current_bias_key);
    }
    ++key;


}
//预积分
void FUSION_CORE::addIMU(shared_ptr<ImuMeasurement> ptr) {
    if (t1 == 0) {
        t1 = ptr -> getTime();
        return;
    }
  	double t2 = ptr -> getTime();
  	double dt = t2 - t1;
    t1 = t2;
  	Vector3 gyroscope = ptr -> getData().head(3);
  	Vector3 accelerometer = ptr -> getData().tail(3);
 	orientation = ptr -> getOri();

	ImuPreIntegrated -> integrateMeasurement(accelerometer, gyroscope, dt);

    ++included_imu_measurement_count;
	
}
/*
void FUSION_CORE::addOdom(shared_ptr<PoseMeasurement>ptr)
{
    
}
*/

void FUSION_CORE::addVins(shared_ptr<PoseMeasurement>ptr)
{
}

Values FUSION_CORE::getResult() {
  	return result;
}  

Pose3 FUSION_CORE::getCurPose() {  
	return current_pose_global; 
}

Matrix FUSION_CORE::getMarginalPoseCov() {
    return current_pose_cov;
}

}   
