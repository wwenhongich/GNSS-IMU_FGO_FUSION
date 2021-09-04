#include "NoiseParams.h"

using namespace std;

// ------------ NoiseParams -------------
// Default Constructor
NoiseParams::NoiseParams() {
    //设置陀螺仪噪声参数为 
    setGyroscopeNoise(0.01);
    //设置加速度计噪声参数为   
    setAccelerometerNoise(0.1);
    //设置迭代噪声参数为
    setIntegrationNoise(0);
    //设置陀螺仪偏置噪声参数为 
    setGyroscopeBiasNoise(0.00001);
    //设置加速度计偏置噪声参数为  
    setAccelerometerBiasNoise(0.0001);
    //设置Gps噪声参数为
    setGpsNoise(0.1);
    //设置平均时间差为
    setAverageDeltaT(0.0100395199348279);
}

void NoiseParams::setGyroscopeNoise(double std) { Qg_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setGyroscopeNoise(const Eigen::Vector3d& std) { Qg_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setGyroscopeNoise(const Eigen::Matrix3d& cov) { Qg_ = cov; }

void NoiseParams::setAccelerometerNoise(double std) { Qa_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setAccelerometerNoise(const Eigen::Vector3d& std) { Qa_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setAccelerometerNoise(const Eigen::Matrix3d& cov) { Qa_ = cov; } 

void NoiseParams::setIntegrationNoise(double std) { Qi_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setIntegrationNoise(const Eigen::Vector3d& std) { Qi_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setIntegrationNoise(const Eigen::Matrix3d& cov) { Qi_ = cov; }   

void NoiseParams::setGyroscopeBiasNoise(double std) { Qbg_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setGyroscopeBiasNoise(const Eigen::Vector3d& std) { Qbg_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setGyroscopeBiasNoise(const Eigen::Matrix3d& cov) { Qbg_ = cov; }

void NoiseParams::setAccelerometerBiasNoise(double std) { Qba_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setAccelerometerBiasNoise(const Eigen::Vector3d& std) { Qba_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setAccelerometerBiasNoise(const Eigen::Matrix3d& cov) { Qba_ = cov; }

void NoiseParams::setGpsNoise(double std) { Qgps_ = std*std*Eigen::Matrix3d::Identity(); }
void NoiseParams::setGpsNoise(const Eigen::Vector3d& std) { Qgps_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); }
void NoiseParams::setGpsNoise(const Eigen::Matrix3d& cov) { Qgps_ = cov; }

void NoiseParams::setAverageDeltaT(double dt) { dt_ = dt; }

Eigen::Matrix3d NoiseParams::getGyroscopeCov() { return Qg_; }
Eigen::Matrix3d NoiseParams::getAccelerometerCov() { return Qa_; }
Eigen::Matrix3d NoiseParams::getIntegrationCov() { return Qi_; }
Eigen::Matrix3d NoiseParams::getGyroscopeBiasCov() { return Qbg_; }
Eigen::Matrix3d NoiseParams::getAccelerometerBiasCov() { return Qba_; }
Eigen::Matrix3d NoiseParams::getGpsCov() { return Qgps_; }
double NoiseParams::getAverageDeltaT() { return dt_; }


