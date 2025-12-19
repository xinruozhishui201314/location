/*
 * Fused Localization System Implementation
 * 融合定位系统实现
 */

#include "fused_localization/FusedLocalization.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

FusedLocalization::FusedLocalization() 
    : Node("fused_localization"),
      init_status_(InitStatus::UNINITIALIZED),
      is_initialized_(false),
      prior_map_loaded_(false),
      gps_available_(false),
      rtk_available_(false),
      keyframe_count_(0),
      last_gps_time_(0.0),
      init_start_time_(0.0)
{
    // 初始化点云
    prior_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    
    // 初始化GTSAM
    ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.relinearizeSkip = 1;
    isam2_ = new ISAM2(isam_params);
    
    // 初始化二维码检测器
    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    aruco_detector_ = static_cast<void*>(new cv::aruco::ArucoDetector(aruco_dict_, aruco_params_));
    #else
    aruco_detector_ = nullptr;
    #endif
    
    // 初始化体素滤波器
    voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
    
    // 加载参数
    loadParameters();
    
    // 加载先验地图和二维码数据库
    if (!prior_map_path_.empty()) {
        loadPriorMap(prior_map_path_);
    }
    loadQRCodeDatabase("");  // 从配置文件加载
    
    // 初始化变换
    last_transform_.setIdentity();
    
    // 创建订阅者
    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", 10,
        std::bind(&FusedLocalization::lidarCallback, this, std::placeholders::_1));
    
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 200,
        std::bind(&FusedLocalization::imuCallback, this, std::placeholders::_1));
    
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        "camera/image", 10,
        std::bind(&FusedLocalization::imageCallback, this, std::placeholders::_1));
    
    sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix", 10,
        std::bind(&FusedLocalization::gpsCallback, this, std::placeholders::_1));
    
    sub_rtk_ = create_subscription<nav_msgs::msg::Odometry>(
        "rtk/odometry", 10,
        std::bind(&FusedLocalization::rtkCallback, this, std::placeholders::_1));
    
    // 创建发布者
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("fused_localization/odometry", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("fused_localization/path", 10);
    pub_map_aligned_ = create_publisher<sensor_msgs::msg::PointCloud2>("fused_localization/map_aligned", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("fused_localization/status", 10);
    
    // 创建100Hz定时器
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / output_frequency_));
    timer_100hz_ = create_wall_timer(
        timer_period,
        std::bind(&FusedLocalization::publish100Hz, this));
    
    RCLCPP_INFO(get_logger(), "Fused Localization System initialized");
}

FusedLocalization::~FusedLocalization() {
    if (isam2_) {
        delete isam2_;
    }
    #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    if (aruco_detector_) {
        delete static_cast<cv::aruco::ArucoDetector*>(aruco_detector_);
    }
    #endif
}

void FusedLocalization::loadParameters() {
    // 从参数服务器读取参数
    declare_parameter("output_frequency", 100.0);
    declare_parameter("prior_map_path", "");
    declare_parameter("init_with_map", true);
    declare_parameter("init_with_qr", true);
    declare_parameter("init_with_gps", true);
    declare_parameter("init_timeout", 10.0);
    declare_parameter("gps_cov_threshold", 2.0);
    declare_parameter("max_keyframe_num", 100);
    declare_parameter("keyframe_distance_threshold", 1.0);
    declare_parameter("keyframe_angle_threshold", 0.2);
    
    get_parameter("output_frequency", output_frequency_);
    get_parameter("prior_map_path", prior_map_path_);
    get_parameter("init_with_map", init_with_map_);
    get_parameter("init_with_qr", init_with_qr_);
    get_parameter("init_with_gps", init_with_gps_);
    get_parameter("init_timeout", init_timeout_);
    get_parameter("gps_cov_threshold", gps_cov_threshold_);
    get_parameter("max_keyframe_num", max_keyframe_num_);
    get_parameter("keyframe_distance_threshold", keyframe_distance_threshold_);
    get_parameter("keyframe_angle_threshold", keyframe_angle_threshold_);
    
    RCLCPP_INFO(get_logger(), "Parameters loaded: output_freq=%.1f Hz", output_frequency_);
}

void FusedLocalization::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_buffer_);
    lidar_buffer_.push_back(msg);
    if (lidar_buffer_.size() > 10) {
        lidar_buffer_.pop_front();
    }
    processData();
}

void FusedLocalization::imuCallback(const sensor_msgs::msg::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_buffer_);
    imu_buffer_.push_back(msg);
    if (imu_buffer_.size() > 2000) {
        imu_buffer_.pop_front();
    }
}

void FusedLocalization::imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_buffer_);
    image_buffer_.push_back(msg);
    if (image_buffer_.size() > 10) {
        image_buffer_.pop_front();
    }
}

void FusedLocalization::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_buffer_);
    double cov = std::max(msg->position_covariance[0], msg->position_covariance[4]);
    if (cov < gps_cov_threshold_) {
        gps_available_ = true;
        last_gps_time_ = rclcpp::Time(msg->header.stamp).seconds();
        gps_buffer_.push_back(msg);
        if (gps_buffer_.size() > 10) {
            gps_buffer_.pop_front();
        }
    } else {
        gps_available_ = false;
    }
}

void FusedLocalization::rtkCallback(const nav_msgs::msg::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_buffer_);
    rtk_available_ = true;
    rtk_buffer_.push_back(msg);
    if (rtk_buffer_.size() > 10) {
        rtk_buffer_.pop_front();
    }
}

void FusedLocalization::processData() {
    if (!synchronizeData()) {
        return;
    }
    
    // 如果未初始化，尝试初始化
    if (!is_initialized_) {
        if (init_status_ == InitStatus::UNINITIALIZED) {
            init_status_ = InitStatus::INITIALIZING;
            init_start_time_ = rclcpp::Clock().now().seconds();
        }
        
        if (initializeSystem()) {
            is_initialized_ = true;
            init_status_ = InitStatus::INITIALIZED;
            RCLCPP_INFO(get_logger(), "System initialized successfully!");
        } else {
            double elapsed = rclcpp::Clock().now().seconds() - init_start_time_;
            if (elapsed > init_timeout_) {
                init_status_ = InitStatus::INIT_FAILED;
                RCLCPP_ERROR(get_logger(), "Initialization timeout!");
            }
        }
        return;
    }
    
    // 前端处理
    processFrontend();
    
    // 后端优化
    processBackend();
}

bool FusedLocalization::synchronizeData() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }
    
    // 简单的同步：使用最新的LiDAR数据
    auto lidar_msg = lidar_buffer_.back();
    pcl::fromROSMsg(*lidar_msg, *current_cloud_);
    
    // 下采样
    voxel_filter_.setInputCloud(current_cloud_);
    voxel_filter_.filter(*current_cloud_);
    
    return true;
}

bool FusedLocalization::initializeSystem() {
    // 优先级：RTK > GPS > 二维码 > 先验地图 > 运动初始化
    if (init_with_gps_ && rtk_available_ && !rtk_buffer_.empty()) {
        return initializeWithGPS();
    }
    
    if (init_with_gps_ && gps_available_ && !gps_buffer_.empty()) {
        return initializeWithGPS();
    }
    
    if (init_with_qr_ && !image_buffer_.empty()) {
        auto image_msg = image_buffer_.back();
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            std::vector<QRCodeInfo> detected_qr;
            if (detectQRCode(cv_ptr->image, detected_qr) && !detected_qr.empty()) {
                return initializeWithQRCode();
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    if (init_with_map_ && prior_map_loaded_ && !current_cloud_->empty()) {
        return initializeWithPriorMap();
    }
    
    // 最后尝试运动初始化（需要一些运动）
    if (lidar_buffer_.size() >= 5) {
        return initializeWithMotion();
    }
    
    return false;
}

bool FusedLocalization::initializeWithPriorMap() {
    RCLCPP_INFO(get_logger(), "Initializing with prior map...");
    
    Eigen::Matrix4d init_transform;
    if (matchWithPriorMap(current_cloud_, init_transform)) {
        current_state_ = transformToState(init_transform, 
                                         rclcpp::Time(lidar_buffer_.back()->header.stamp).seconds());
        last_transform_ = init_transform;
        *last_cloud_ = *current_cloud_;
        
        // 初始化因子图
        factor_graph_.resize(0);
        initial_estimate_.clear();
        
        Pose3 init_pose(Rot3(init_transform.block<3,3>(0,0)),
                       Point3(init_transform.block<3,1>(0,3)));
        
        noiseModel::Diagonal::shared_ptr prior_noise = 
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
        factor_graph_.add(PriorFactor<Pose3>(X(0), init_pose, prior_noise));
        initial_estimate_.insert(X(0), init_pose);
        
        keyframe_count_ = 1;
        return true;
    }
    
    return false;
}

bool FusedLocalization::initializeWithQRCode() {
    RCLCPP_INFO(get_logger(), "Initializing with QR code...");
    
    if (image_buffer_.empty()) return false;
    
    auto image_msg = image_buffer_.back();
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        std::vector<QRCodeInfo> detected_qr;
        if (detectQRCode(cv_ptr->image, detected_qr) && !detected_qr.empty()) {
            auto& qr = detected_qr[0];
            if (qr.is_valid && qr_code_database_.find(qr.id) != qr_code_database_.end()) {
                auto& db_qr = qr_code_database_[qr.id];
                
                // 估计从二维码到相机的变换
                Eigen::Matrix4d qr_to_cam = estimatePoseFromQRCode(qr, cv_ptr->image);
                
                // 获取二维码的全局位姿
                Eigen::Matrix4d qr_global;
                qr_global.setIdentity();
                qr_global.block<3,1>(0,3) = db_qr.position;
                qr_global.block<3,3>(0,0) = db_qr.orientation.toRotationMatrix();
                
                // 计算初始位姿（需要相机到LiDAR的外参）
                // 这里简化处理，假设已知外参
                Eigen::Matrix4d init_transform = qr_global * qr_to_cam.inverse();
                
                current_state_ = transformToState(init_transform,
                                                 rclcpp::Time(image_msg->header.stamp).seconds());
                last_transform_ = init_transform;
                
                // 初始化因子图
                factor_graph_.resize(0);
                initial_estimate_.clear();
                
                Pose3 init_pose(Rot3(init_transform.block<3,3>(0,0)),
                               Point3(init_transform.block<3,1>(0,3)));
                
                noiseModel::Diagonal::shared_ptr prior_noise = 
                    noiseModel::Diagonal::Sigmas((Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished());
                factor_graph_.add(PriorFactor<Pose3>(X(0), init_pose, prior_noise));
                initial_estimate_.insert(X(0), init_pose);
                
                keyframe_count_ = 1;
                return true;
            }
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
    
    return false;
}

bool FusedLocalization::initializeWithGPS() {
    RCLCPP_INFO(get_logger(), "Initializing with GPS/RTK...");
    
    if (rtk_available_ && !rtk_buffer_.empty()) {
        auto rtk_msg = rtk_buffer_.back();
        current_state_.position.x() = rtk_msg->pose.pose.position.x;
        current_state_.position.y() = rtk_msg->pose.pose.position.y;
        current_state_.position.z() = rtk_msg->pose.pose.position.z;
        
        current_state_.orientation.x() = rtk_msg->pose.pose.orientation.x;
        current_state_.orientation.y() = rtk_msg->pose.pose.orientation.y;
        current_state_.orientation.z() = rtk_msg->pose.pose.orientation.z;
        current_state_.orientation.w() = rtk_msg->pose.pose.orientation.w;
        
        current_state_.timestamp = rclcpp::Time(rtk_msg->header.stamp).seconds();
        
    } else if (gps_available_ && !gps_buffer_.empty()) {
        // GPS只提供位置，需要结合IMU提供姿态
        auto gps_msg = gps_buffer_.back();
        // 简化的GPS到局部坐标转换（实际需要UTM等）
        current_state_.position.x() = gps_msg->longitude * 111320.0;  // 简化
        current_state_.position.y() = gps_msg->latitude * 111320.0;
        current_state_.position.z() = gps_msg->altitude;
        
        // 从IMU获取初始姿态
        if (!imu_buffer_.empty()) {
            auto imu_msg = imu_buffer_.back();
            tf2::Quaternion q;
            tf2::fromMsg(imu_msg->orientation, q);
            current_state_.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
        }
        
        current_state_.timestamp = rclcpp::Time(gps_msg->header.stamp).seconds();
    } else {
        return false;
    }
    
    last_transform_ = stateToTransform(current_state_);
    
    // 初始化因子图
    factor_graph_.resize(0);
    initial_estimate_.clear();
    
    Pose3 init_pose(Rot3(current_state_.orientation),
                   Point3(current_state_.position));
    
    noiseModel::Diagonal::shared_ptr prior_noise = 
        noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
    factor_graph_.add(PriorFactor<Pose3>(X(0), init_pose, prior_noise));
    initial_estimate_.insert(X(0), init_pose);
    
    keyframe_count_ = 1;
    return true;
}

bool FusedLocalization::initializeWithMotion() {
    // 需要至少几个关键帧才能进行运动初始化
    // 这里简化处理
    RCLCPP_WARN(get_logger(), "Motion initialization not fully implemented");
    return false;
}

void FusedLocalization::processFrontend() {
    if (last_cloud_->empty()) {
        *last_cloud_ = *current_cloud_;
        return;
    }
    
    // 估计里程计（类似FAST-LIVO2的直接法）
    Eigen::Matrix4d odom_transform = estimateOdometry(last_cloud_, current_cloud_);
    
    // 更新状态
    last_transform_ = last_transform_ * odom_transform;
    current_state_ = transformToState(last_transform_, 
                                     rclcpp::Time(lidar_buffer_.back()->header.stamp).seconds());
    
    *last_cloud_ = *current_cloud_;
}

Eigen::Matrix4d FusedLocalization::estimateOdometry(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2) {
    
    // 使用ICP进行粗略估计（实际应该使用FAST-LIVO2的直接法）
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(50);
    
    pcl::PointCloud<pcl::PointXYZI> aligned;
    icp.align(aligned);
    
    if (icp.hasConverged()) {
        Eigen::Matrix4f transform_f = icp.getFinalTransformation();
        Eigen::Matrix4d transform = transform_f.cast<double>();
        return transform;
    }
    
    return Eigen::Matrix4d::Identity();
}

void FusedLocalization::processBackend() {
    // 检查是否需要添加新的关键帧
    bool add_keyframe = false;
    
    if (keyframe_states_.empty()) {
        add_keyframe = true;
    } else {
        auto& last_kf = keyframe_states_.back();
        double dist = (current_state_.position - last_kf.position).norm();
        Eigen::Quaterniond q_diff = current_state_.orientation * last_kf.orientation.inverse();
        double angle = 2.0 * acos(std::abs(q_diff.w()));
        
        if (dist > keyframe_distance_threshold_ || angle > keyframe_angle_threshold_) {
            add_keyframe = true;
        }
    }
    
    if (add_keyframe) {
        // 添加里程计因子
        addOdometryFactor();
        
        // 添加GPS因子
        if (gps_available_ || rtk_available_) {
            addGPSFactor();
        }
        
        // 添加二维码因子
        if (!image_buffer_.empty()) {
            addQRCodeFactor();
        }
        
        // 添加先验地图因子
        if (prior_map_loaded_) {
            addPriorMapFactor();
        }
        
        // 优化
        optimizeGraph();
        
        // 保存关键帧
        keyframe_states_.push_back(current_state_);
        keyframe_clouds_.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>(*current_cloud_)));
        
        if (keyframe_states_.size() > max_keyframe_num_) {
            keyframe_states_.pop_front();
            keyframe_clouds_.pop_front();
        }
    }
}

void FusedLocalization::addOdometryFactor() {
    if (keyframe_count_ == 0) return;
    
    Pose3 pose_from(Rot3(keyframe_states_.back().orientation),
                   Point3(keyframe_states_.back().position));
    Pose3 pose_to(Rot3(current_state_.orientation),
                 Point3(current_state_.position));
    
    Pose3 relative_pose = pose_from.inverse() * pose_to;
    
    noiseModel::Diagonal::shared_ptr odom_noise = 
        noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());
    
    factor_graph_.add(BetweenFactor<Pose3>(X(keyframe_count_-1), X(keyframe_count_), 
                                          relative_pose, odom_noise));
    initial_estimate_.insert(X(keyframe_count_), pose_to);
}

void FusedLocalization::addGPSFactor() {
    if (rtk_available_ && !rtk_buffer_.empty()) {
        auto rtk_msg = rtk_buffer_.back();
        Point3 gps_pos(rtk_msg->pose.pose.position.x,
                      rtk_msg->pose.pose.position.y,
                      rtk_msg->pose.pose.position.z);
        
        noiseModel::Diagonal::shared_ptr gps_noise = 
            noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 0.1).finished());
        
        factor_graph_.add(GPSFactor(X(keyframe_count_), gps_pos, gps_noise));
    } else if (gps_available_ && !gps_buffer_.empty()) {
        auto gps_msg = gps_buffer_.back();
        // 简化的GPS转换
        Point3 gps_pos(gps_msg->longitude * 111320.0,
                      gps_msg->latitude * 111320.0,
                      gps_msg->altitude);
        
        double cov = std::max(gps_msg->position_covariance[0], 
                             gps_msg->position_covariance[4]);
        double sigma = std::sqrt(cov);
        
        noiseModel::Diagonal::shared_ptr gps_noise = 
            noiseModel::Diagonal::Sigmas((Vector(3) << sigma, sigma, sigma*2).finished());
        
        factor_graph_.add(GPSFactor(X(keyframe_count_), gps_pos, gps_noise));
    }
}

void FusedLocalization::addQRCodeFactor() {
    if (image_buffer_.empty()) return;
    
    auto image_msg = image_buffer_.back();
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        std::vector<QRCodeInfo> detected_qr;
        if (detectQRCode(cv_ptr->image, detected_qr)) {
            for (auto& qr : detected_qr) {
                if (qr.is_valid && qr_code_database_.find(qr.id) != qr_code_database_.end()) {
                    auto& db_qr = qr_code_database_[qr.id];
                    
                    // 创建高精度位姿约束
                    Pose3 qr_pose(Rot3(db_qr.orientation), Point3(db_qr.position));
                    
                    // 估计从二维码到相机的变换
                    Eigen::Matrix4d qr_to_cam = estimatePoseFromQRCode(qr, cv_ptr->image);
                    // 这里需要相机到LiDAR的外参，简化处理
                    Pose3 relative_pose(Rot3(qr_to_cam.block<3,3>(0,0)),
                                      Point3(qr_to_cam.block<3,1>(0,3)));
                    
                    Pose3 absolute_pose = qr_pose * relative_pose.inverse();
                    
                    // 高精度噪声模型（厘米级）
                    noiseModel::Diagonal::shared_ptr qr_noise = 
                        noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
                    
                    factor_graph_.add(PriorFactor<Pose3>(X(keyframe_count_), absolute_pose, qr_noise));
                }
            }
        }
    } catch (cv_bridge::Exception& e) {
        // 忽略错误
    }
}

void FusedLocalization::addPriorMapFactor() {
    // 与先验地图匹配，添加约束
    Eigen::Matrix4d map_transform;
    if (matchWithPriorMap(current_cloud_, map_transform)) {
        Pose3 map_pose(Rot3(map_transform.block<3,3>(0,0)),
                      Point3(map_transform.block<3,1>(0,3)));
        
        noiseModel::Diagonal::shared_ptr map_noise = 
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());
        
        factor_graph_.add(PriorFactor<Pose3>(X(keyframe_count_), map_pose, map_noise));
    }
}

void FusedLocalization::optimizeGraph() {
    if (factor_graph_.size() == 0) return;
    
    isam2_->update(factor_graph_, initial_estimate_);
    isam2_->update();
    
    current_estimate_ = isam2_->calculateEstimate();
    
    if (current_estimate_.exists(X(keyframe_count_))) {
        Pose3 optimized_pose = current_estimate_.at<Pose3>(X(keyframe_count_));
        
        current_state_.position = optimized_pose.translation();
        current_state_.orientation = optimized_pose.rotation().toQuaternion();
        
        last_transform_ = stateToTransform(current_state_);
    }
    
    factor_graph_.resize(0);
    initial_estimate_.clear();
    
    keyframe_count_++;
}

bool FusedLocalization::detectQRCode(const cv::Mat& image, std::vector<QRCodeInfo>& detected_qr) {
    detected_qr.clear();
    
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    
    // 兼容不同版本的OpenCV API
    #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    if (aruco_detector_) {
        cv::aruco::ArucoDetector* detector = static_cast<cv::aruco::ArucoDetector*>(aruco_detector_);
        detector->detectMarkers(image, marker_corners, marker_ids, rejected_candidates);
    } else {
        return false;
    }
    #else
    cv::aruco::detectMarkers(image, aruco_dict_, marker_corners, marker_ids, aruco_params_, rejected_candidates);
    #endif
    
    if (marker_ids.empty()) {
        return false;
    }
    
    for (size_t i = 0; i < marker_ids.size(); i++) {
        QRCodeInfo qr;
        qr.id = marker_ids[i];
        qr.is_valid = true;
        qr.confidence = 1.0;  // 简化
        
        // 计算二维码中心
        cv::Point2f center(0, 0);
        for (const auto& corner : marker_corners[i]) {
            center += corner;
        }
        center /= 4.0;
        
        detected_qr.push_back(qr);
    }
    
    return !detected_qr.empty();
}

Eigen::Matrix4d FusedLocalization::estimatePoseFromQRCode(
    const QRCodeInfo& qr_info, const cv::Mat& image) {
    
    // 这里需要相机内参和二维码尺寸
    // 简化处理，返回单位矩阵
    // 实际应该使用cv::aruco::estimatePoseSingleMarkers
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    return transform;
}

bool FusedLocalization::matchWithPriorMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    Eigen::Matrix4d& transform) {
    
    if (!prior_map_loaded_ || prior_map_->empty()) {
        return false;
    }
    
    // 使用NDT进行匹配
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputSource(cloud);
    ndt.setInputTarget(prior_map_);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(50);
    ndt.setStepSize(0.1);
    ndt.setTransformationEpsilon(0.01);
    
    pcl::PointCloud<pcl::PointXYZI> aligned;
    ndt.align(aligned, last_transform_.cast<float>());
    
    if (ndt.hasConverged()) {
        Eigen::Matrix4f transform_f = ndt.getFinalTransformation();
        transform = transform_f.cast<double>();
        return true;
    }
    
    return false;
}

void FusedLocalization::publish100Hz() {
    std::lock_guard<std::mutex> lock(mutex_state_);
    
    // 发布里程计
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = current_state_.position.x();
    odom_msg.pose.pose.position.y = current_state_.position.y();
    odom_msg.pose.pose.position.z = current_state_.position.z();
    
    odom_msg.pose.pose.orientation.x = current_state_.orientation.x();
    odom_msg.pose.pose.orientation.y = current_state_.orientation.y();
    odom_msg.pose.pose.orientation.z = current_state_.orientation.z();
    odom_msg.pose.pose.orientation.w = current_state_.orientation.w();
    
    odom_msg.twist.twist.linear.x = current_state_.velocity.x();
    odom_msg.twist.twist.linear.y = current_state_.velocity.y();
    odom_msg.twist.twist.linear.z = current_state_.velocity.z();
    
    pub_odom_->publish(odom_msg);
    
    // 发布路径
    static nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    path_msg.poses.push_back(pose_stamped);
    
    if (path_msg.poses.size() > 1000) {
        path_msg.poses.erase(path_msg.poses.begin());
    }
    
    pub_path_->publish(path_msg);
}

Eigen::Matrix4d FusedLocalization::stateToTransform(const FusedState& state) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = state.orientation.toRotationMatrix();
    transform.block<3,1>(0,3) = state.position;
    return transform;
}

FusedState FusedLocalization::transformToState(const Eigen::Matrix4d& transform, double timestamp) {
    FusedState state;
    state.position = transform.block<3,1>(0,3);
    state.orientation = Eigen::Quaterniond(transform.block<3,3>(0,0));
    state.timestamp = timestamp;
    return state;
}

void FusedLocalization::loadQRCodeDatabase(const std::string& config_path) {
    // 从配置文件加载二维码数据库
    // 这里简化处理，实际应该从YAML或JSON文件加载
    QRCodeInfo qr1;
    qr1.id = 0;
    qr1.position = Eigen::Vector3d(0, 0, 0);
    qr1.orientation = Eigen::Quaterniond::Identity();
    qr1.size = 0.1;
    qr1.is_valid = true;
    qr_code_database_[0] = qr1;
    
    RCLCPP_INFO(get_logger(), "QR code database loaded: %zu codes", qr_code_database_.size());
}

void FusedLocalization::loadPriorMap(const std::string& map_path) {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *prior_map_) == -1) {
        RCLCPP_WARN(get_logger(), "Could not load prior map: %s", map_path.c_str());
        return;
    }
    
    map_kdtree_->setInputCloud(prior_map_);
    prior_map_loaded_ = true;
    
    RCLCPP_INFO(get_logger(), "Prior map loaded: %zu points", prior_map_->size());
}

