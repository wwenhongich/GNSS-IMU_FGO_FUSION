
#include "TransformFusion.h"
#include <nav_msgs/Odometry.h>
using namespace std;

// Constructor
TransformFusion::TransformFusion(ros::NodeHandle n) : n_(n) {}

//init
void TransformFusion::init() {
    // Create private node handle
    ros::NodeHandle nh("~");

    // Set noise parameters
    gtsam::Vector12 calibration = gtsam::Vector12::Zero();
    double std, cov;
    nh.param<double>("noise/accelerometer_std", std, 0.01);
    init_params_.setAccelerometerNoise(std);
    calibration(6) = std;
    nh.param<double>("noise/gyroscope_std", std, 0.000175);
    init_params_.setGyroscopeNoise(std);
    calibration(7) = std;
    nh.param<double>("noise/integration_std", std, 0);
    init_params_.setIntegrationNoise(std);
    calibration(8) = std;
    nh.param<double>("noise/accelerometer_bias_std", std, 0.000167);
    init_params_.setAccelerometerBiasNoise(std);
    calibration(9) = std;
    nh.param<double>("noise/gyroscope_bias_std", std, 2.91e-006);
    init_params_.setGyroscopeBiasNoise(std);
    calibration(10) = std;
    nh.param<double>("noise/average_delta_t", std, 0.00100395199348279);
    init_params_.setAverageDeltaT(std);
    calibration(11) = std;

    nh.param<double>("noise/gps_std", std,0.3);//0.3
    init_params_.setGpsNoise(std);

    gtsam::Vector3 position = gtsam::Vector3::Zero();
    Fusion_Core.initialize(calibration, position);


    // ----- Settings --------
    nh.param<string>("settings/map_frame_id", map_frame_id_, "/map");
 

}

// Process Data
void TransformFusion::run() {
// Subscribe Topics
    this->subscribe();


    isam2_thread_ = std::thread([this]{this->mainIsam2Thread();});

    output_thread_ = std::thread([this]{this->outputPublishingThread();});

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}    

// Subscribe to all publishers
void TransformFusion::subscribe() {
    double start_sub_time = ros::Time::now().toSec();

    // Create private node handle to get topic names
    ros::NodeHandle nh("~");
    string imu_topic;
    nh.param<string>("settings/imu_topic", imu_topic, "/imu/data");

    // Retrieve imu frame_id 
    ROS_INFO("Waiting for IMU message...");
    sensor_msgs::Imu::ConstPtr imu_msg = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic);
    imu_frame_id_ = imu_msg->header.frame_id;

    Eigen::Quaterniond quat(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    quat.normalize();
    initial_R_ = quat.toRotationMatrix();
    Eigen::Vector3d euler = initial_R_.eulerAngles(0, 1, 2);
    initial_yaw_ = euler(2);
    ROS_INFO("IMU message received. IMU frame is set to %s.", imu_frame_id_.c_str());
    

    // Retrieve gps frame_id 
    string gps_topic;
    nh.param<string>("settings/gps_topic", gps_topic, "/fix");
    ROS_INFO("Waiting for GPS message...");
    sensor_msgs::NavSatFix::ConstPtr gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gps_topic);
    gps_frame_id_ = gps_msg->header.frame_id;
    // TODO: Convert output from IMU frame to base frame 
    nh.param<string>("settings/base_frame_id", base_frame_id_, "base_link");
    ROS_INFO("Waiting for tf lookup between frames %s and %s...", gps_frame_id_.c_str(), base_frame_id_.c_str());
    tf::TransformListener listener;
    try {
        listener.waitForTransform(base_frame_id_, gps_frame_id_, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(base_frame_id_, gps_frame_id_, ros::Time(0), base_to_gps_transform_);
        ROS_INFO("Tranform between frames %s and %s was found.", gps_frame_id_.c_str(), base_frame_id_.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        base_to_gps_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), gps_frame_id_, base_frame_id_);
    }

    double x = base_to_gps_transform_.getOrigin().getX(), 
           y = base_to_gps_transform_.getOrigin().getY(), 
           z = base_to_gps_transform_.getOrigin().getZ();
    Og0_to_Ob0_ << x, y, z;
    
    initial_lla_ << gps_msg->latitude, 
                    gps_msg->longitude, 
                    gps_msg->altitude;                   
    initial_ecef_ = lla_to_ecef(initial_lla_);
    ROS_INFO("GPS message received. GPS frame is set to %s.", gps_frame_id_.c_str());




    // Subscribe to IMU publisher
    ROS_INFO("Subscribing to %s.", imu_topic.c_str());
    imu_sub_ = n_.subscribe(imu_topic, 100, &TransformFusion::imuCallback, this);

    // Subscribe to GPS publisher
    ROS_INFO("Subscribing to %s.", gps_topic.c_str());
    gps_sub_ = n_.subscribe(gps_topic, 1, &TransformFusion::gpsCallback, this);

  
   double end_sub_time = ros::Time::now().toSec();
   ROS_INFO("subscribe start time: %f", start_sub_time);
   ROS_INFO("subscribe end time: %f", end_sub_time);
}

// IMU Callback function
void TransformFusion::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new ImuMeasurement(msg));
    timeLaserInfoStamp = msg->header.stamp;
    m_queue_.push(ptr);
}

// GPS Callback function
void TransformFusion::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  
    shared_ptr<GpsMeasurement> ptr(new GpsMeasurement(msg));
    geometry_msgs::PoseWithCovarianceStamped::Ptr pose_msg(new geometry_msgs::PoseWithCovarianceStamped());
    pose_msg->header.seq = msg->header.seq;
    pose_msg->header.stamp = msg->header.stamp;
    pose_msg->header.frame_id = base_frame_id_; 
    gtsam::Vector3 position = lla_to_enu(ptr->getData());
    pose_msg->pose.pose.position.x = position(0); 
    pose_msg->pose.pose.position.y = position(1); 
    pose_msg->pose.pose.position.z = position(2); 
    shared_ptr<PoseMeasurement> pose_ptr(new PoseMeasurement(pose_msg));
    m_queue_.push(pose_ptr);

}



gtsam::Vector3 TransformFusion::lla_to_ecef(const gtsam::Vector3& lla) {
    const double equatorial_radius = 6378137.0;
    const double polar_radius = 6356752.31424518;
    const double square_ratio = pow(polar_radius,2) / pow(equatorial_radius,2);

    const double lat = lla(0) * M_PI / 180;
    const double lon = lla(1) * M_PI / 180;
    const double alt = lla(2);
    const double N = equatorial_radius / sqrt(1 - (1-square_ratio) * pow(sin(lat),2));

    const double z = (square_ratio * N + alt) * sin(lat);
    const double q = (N + alt) * cos(lat);
    const double x = q * cos(lon);
    const double y = q * sin(lon);

    return (gtsam::Vector3() << x, y, z).finished();
}

gtsam::Vector3 TransformFusion::lla_to_enu(const gtsam::Vector3& lla) {

    gtsam::Vector3 cur_ecef = lla_to_ecef(lla);
    gtsam::Vector3 r_ecef = cur_ecef - initial_ecef_;

    double phi = initial_lla_(0) * M_PI / 180;
    double lam = initial_lla_(1) * M_PI / 180;

    gtsam::Matrix33 R = (gtsam::Matrix33() <<
        -sin(lam),     cos(lam),       0,
        -cos(lam)*sin(phi), -sin(lam)*sin(phi), cos(phi),
        cos(lam)*cos(phi),  sin(lam)*cos(phi),  sin(phi)
    ).finished();

    return R * r_ecef;
}



void TransformFusion::mainIsam2Thread() {
    cout << "Inside Main Filtering Thread\n";
    shared_ptr<Measurement> m_ptr;
    shared_ptr<ImuMeasurement> imu_ptr_last;
    double t, t_last;
    bool first_gps = true;


 // 0.Block until first IMU measurement is received
    while (ros::ok()) {
        m_queue_.pop(m_ptr);
        if (m_ptr->getType() == IMU) {
            ROS_INFO("First IMU measurement received. Starting GTSAM.");
            double current_time = ros::Time::now().toSec();
            t_last = m_ptr->getTime();
           // ROS_INFO("time delayed: %f", current_time - t_last);
            imu_ptr_last = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }

     // 1.Main Optimizer
    while (ros::ok()) {
       //  Throw warning if measurement queue is getting too large
        if (m_queue_.size() > MAX_QUEUE_SIZE) {
            ROS_WARN("Measurement queue size (%d) is greater than MAX_QUEUE_SIZE. GTSAM is not realtime!", m_queue_.size());
        }   
        // Wait until buffer is full
        while(m_queue_.size() < QUEUE_BUFFER_SIZE) {
            this_thread::sleep_for(chrono::milliseconds(10));
        }
        // Retrieve next measurement (Blocking)
        m_queue_.pop(m_ptr);
 
        switch (m_ptr->getType()) {
            case IMU: {
                // ROS_INFO("Propagating state with IMU measurements.");
                auto imu_ptr = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
                double current_time = ros::Time::now().toSec();
                t = imu_ptr->getTime();
  // ROS_INFO("time delayed: %f", current_time - t_last);
                Fusion_Core.addIMU(imu_ptr);
                t_last = t;
                imu_ptr_last = imu_ptr;

                // Uncomment out below part to use IMU orientation measurement to compute gps-base transform 

                cur_baselink_orientation_ = imu_ptr->getOri();

                break;
            }
            case GPS: {
                // ROS_INFO("Correcting state with GPS measurements.");
                // Uncomment out this part to use GPS measurement
                auto gps_ptr = dynamic_pointer_cast<GpsMeasurement>(m_ptr);

                Eigen::Matrix<double,3,1> gps_xyz = lla_to_enu(gps_ptr->getData());
                Eigen::Vector3d position = gps_xyz.head(3); //state.getPosition();

                Eigen::Quaternion<double> orientation(cur_baselink_orientation_[3],cur_baselink_orientation_[0],cur_baselink_orientation_[1],cur_baselink_orientation_[2]);
                orientation.normalize();

                tf::Transform gps_pose;

     
                gps_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
                gps_pose.setRotation( tf::Quaternion::getIdentity() );
                tf::Transform gps_offset_rotated;
                gps_offset_rotated.setOrigin( tf::Vector3(-Og0_to_Ob0_(0),-Og0_to_Ob0_(1),-Og0_to_Ob0_(2)) );
                gps_offset_rotated.setOrigin( tf::quatRotate(tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()), gps_offset_rotated.getOrigin()) );
                gps_offset_rotated.setRotation( tf::Quaternion::getIdentity() );
                tf::Transform base_pose = gps_offset_rotated.inverse() * gps_pose;
                

                tf::Vector3 base_position = base_pose.getOrigin();
                Eigen::Vector3d base_Ob(base_position.getX(),base_position.getY(),base_position.getZ());
                base_Ob -= initial_R_*Og0_to_Ob0_; 
                break;
            }
           /* case VINS:{
                auto vins_ptr = dynamic_pointer_cast<PoseMeasurement>(m_ptr);
            }*/
            case POSE: {
                auto pose_ptr = dynamic_pointer_cast<PoseMeasurement>(m_ptr);

                // transform before add to graph
                gtsam::Pose3 pose = Fusion_Core.getCurPose();
                gtsam::Quaternion quat = pose.rotation().toQuaternion();
                Eigen::Vector3d position = pose_ptr->getData();
                tf::Transform gps_pose;
                if (first_gps) {
                    gps_pose.setRotation( tf::Quaternion(
                        cur_baselink_orientation_[0],cur_baselink_orientation_[1],cur_baselink_orientation_[2],cur_baselink_orientation_[3]) );
                    first_gps = false;
                }
                else {
                    gps_pose.setRotation( tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()) );
                }
                gps_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
                tf::Transform base_pose = gps_pose * base_to_gps_transform_.inverse();
                tf::Quaternion base_orientation = base_pose.getRotation().normalize();
                tf::Vector3 base_position = base_pose.getOrigin();
                Eigen::Vector3d base_Ob(base_position.getX(),base_position.getY(),base_position.getZ());
                base_Ob += initial_R_*Og0_to_Ob0_; // subtract origin transition
                
                geometry_msgs::PoseWithCovarianceStamped::Ptr pose_msg(new geometry_msgs::PoseWithCovarianceStamped());
                ros::Time measurement_time(pose_ptr->getTime());
                pose_msg->header.stamp = measurement_time;
                pose_msg->header.frame_id = base_frame_id_; 
                pose_msg->pose.pose.position.x = base_Ob(0); 
                pose_msg->pose.pose.position.y = base_Ob(1); 
                pose_msg->pose.pose.position.z = base_Ob(2); 
                shared_ptr<PoseMeasurement> posePtr(new PoseMeasurement(pose_msg));
                
                Fusion_Core.addGPS(posePtr);

     
                break;
            }
            default:
                cout << "Unknown measurement, skipping...\n";
        }
    }
}


// Thread for publishing the output of the filter
void TransformFusion::outputPublishingThread() {
    // Create private node handle to get topic names
    ros::NodeHandle nh("~");
    double publish_rate;
    string base_frame_id, pose_topic, state_topic, path_topic, path_frame_id, odom_topic;

    nh.param<double>("settings/publish_rate", publish_rate, 100);
    nh.param<string>("settings/base_frame_id", base_frame_id, "/base_link");
    nh.param<string>("settings/pose_topic", pose_topic, "/pose");
    nh.param<string>("settings/state_topic", state_topic, "/state");
    nh.param<string>("settings/path_topic", path_topic, "/path");
    nh.param<string>("/FGO", odom_topic, "/FGO");
    nh.param<string>("settings/path_frame_id", path_frame_id, "path");

    ROS_INFO("Map frame id set to %s.", map_frame_id_.c_str());
    ROS_INFO("Base frame id set to %s.", base_frame_id.c_str());
    ROS_INFO("Path frame id set to %s.", path_frame_id.c_str());
    ROS_INFO("Pose topic publishing under %s.", pose_topic.c_str());
    ROS_INFO("State topic publishing under %s.", state_topic.c_str());
    ROS_INFO("Path topic publishing under %s.", path_topic.c_str());
//转换Tf树
    tf::TransformListener listener;
    try {
        listener.waitForTransform(base_frame_id_, path_frame_id, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(base_frame_id_, path_frame_id, ros::Time(0), base_to_path_transform_);
        ROS_INFO("Tranform between frames %s and %s was found.", path_frame_id.c_str(), base_frame_id_.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        base_to_gps_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), path_frame_id, base_frame_id_);
    }

    // Create publishers for pose and state messages
    ros::Publisher pose_pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    ros::Publisher path_pub = n_.advertise<nav_msgs::Path>(path_topic, 1000);
    ros::Publisher odom_pub = n_.advertise<nav_msgs::Odometry>(odom_topic, 1000);


    //ros::Publisher state_pub = n_.advertise<inekf_msgs::State>(state_topic, 1000);
    static tf::TransformBroadcaster tf_broadcaster;
    
    // Init loop params
    uint32_t seq = 0;
    geometry_msgs::Point point_last;
    ros::Rate loop_rate(publish_rate);

    // Main loop
    while(true) {
        gtsam::Pose3 pose = Fusion_Core.getCurPose();
        

        
        // Create and send pose message
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.seq = seq;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = path_frame_id;
        Eigen::Vector3d position = pose.translation().vector();
        gtsam::Quaternion quat = pose.rotation().toQuaternion();
         
        pose_msg.pose.pose.position.x = position(0); 
        pose_msg.pose.pose.position.y = position(1); 
        pose_msg.pose.pose.position.z = position(2);
        pose_msg.pose.pose.orientation.x = quat.x();
        pose_msg.pose.pose.orientation.y = quat.y();
        pose_msg.pose.pose.orientation.z = quat.z();
        pose_msg.pose.pose.orientation.w = quat.w();

        nav_msgs::Odometry  globalOdometry;
        globalOdometry.header.stamp = timeLaserInfoStamp;
        globalOdometry.header.frame_id = path_frame_id;
        globalOdometry.pose.pose.position.x = position(0); 
        globalOdometry.pose.pose.position.y =  position(1); 
        globalOdometry.pose.pose.position.z = position(2);
        globalOdometry.pose.pose.orientation.x =quat.x();
        globalOdometry.pose.pose.orientation.y =  quat.y();
        globalOdometry.pose.pose.orientation.z = quat.z();
        globalOdometry.pose.pose.orientation.w = quat.w();
        odom_pub.publish(globalOdometry);


        geometry_msgs::PoseStamped final_pose_msg;
        path.header.seq = seq;
        path.header.stamp = timeLaserInfoStamp;
        path.header.frame_id = path_frame_id; 
        final_pose_msg.pose.position.x = position(0); 
        final_pose_msg.pose.position.y = position(1); 
        final_pose_msg.pose.position.z = position(2);
        final_pose_msg.pose.orientation.x = quat.x();
        final_pose_msg.pose.orientation.y = quat.y();
        final_pose_msg.pose.orientation.z = quat.z();
        final_pose_msg.pose.orientation.w = quat.w();
        path.poses.push_back(final_pose_msg);

        pose_pub.publish(pose_msg);
        path_pub.publish(path);

   
        seq++;
        loop_rate.sleep();
    }

}


