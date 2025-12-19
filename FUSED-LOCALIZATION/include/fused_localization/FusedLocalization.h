/*
 * Fused Localization System
 * 融合FAST-LIVO2前端和LIO-SAM后端的高性能定位系统
 * 支持：先验地图、GPS/RTK、二维码定位、100Hz输出
 */

#ifndef FUSED_LOCALIZATION_H
#define FUSED_LOCALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>
#include <string>
#include <optional>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco_detector.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>
#include <unordered_map>

using namespace gtsam;
using symbol_shorthand::X; // Pose3
using symbol_shorthand::V; // Vel
using symbol_shorthand::B; // Bias
using symbol_shorthand::G; // GPS

// 状态结构
struct FusedState {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    double timestamp;
    
    FusedState() : position(Eigen::Vector3d::Zero()),
                   orientation(Eigen::Quaterniond::Identity()),
                   velocity(Eigen::Vector3d::Zero()),
                   acc_bias(Eigen::Vector3d::Zero()),
                   gyro_bias(Eigen::Vector3d::Zero()),
                   timestamp(0.0) {}
};

// 二维码信息
struct QRCodeInfo {
    int id;
    Eigen::Vector3d position;  // 全局位置
    Eigen::Quaterniond orientation;  // 全局姿态
    double size;  // 二维码尺寸（米）
    double confidence;
    bool is_valid;
    
    QRCodeInfo() : id(-1), position(Eigen::Vector3d::Zero()),
                   orientation(Eigen::Quaterniond::Identity()),
                   size(0.1), confidence(0.0), is_valid(false) {}
};

// 初始化状态
enum class InitStatus {
    UNINITIALIZED,
    INITIALIZING,
    INITIALIZED,
    INIT_FAILED
};

// 里程计结果
struct OdometryResult {
    Eigen::Matrix4d transform;
    double fitness;
    double inlier_ratio;
    std::string method;
    bool success;
    OdometryResult()
        : transform(Eigen::Matrix4d::Identity()),
          fitness(1e9),
          inlier_ratio(0.0),
          method("none"),
          success(false) {}
};

// 多候选姿态
struct CandidatePose {
    Eigen::Matrix4d transform;
    double score;
    std::string source;
    bool valid;
    CandidatePose() : transform(Eigen::Matrix4d::Identity()), score(1e9), source("none"), valid(false) {}
};

class FusedLocalization : public rclcpp::Node {
public:
    FusedLocalization();
    ~FusedLocalization();

private:
    // ROS2 订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_rtk_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_aligned_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    
    // 高频输出定时器 (100Hz)
    rclcpp::TimerBase::SharedPtr timer_100hz_;
    
    // 数据缓冲区
    std::deque<sensor_msgs::msg::PointCloud2::ConstPtr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstPtr> imu_buffer_;
    std::deque<sensor_msgs::msg::Image::ConstPtr> image_buffer_;
    std::deque<sensor_msgs::msg::NavSatFix::ConstPtr> gps_buffer_;
    std::deque<nav_msgs::msg::Odometry::ConstPtr> rtk_buffer_;
    
    std::mutex mutex_buffer_;
    std::mutex mutex_state_;
    
    // 状态
    FusedState current_state_;
    InitStatus init_status_;
    bool is_initialized_;
    
    // 先验地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr prior_map_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr map_kdtree_;
    bool prior_map_loaded_;
    std::string prior_map_path_;
    
    // 二维码字典和检测器
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    void* aruco_detector_;  // 指向ArucoDetector的指针（条件编译）
    std::unordered_map<int, QRCodeInfo> qr_code_database_;  // 已知二维码位置数据库
    
    // GTSAM 后端优化
    ISAM2* isam2_;
    NonlinearFactorGraph factor_graph_;
    Values initial_estimate_;
    Values current_estimate_;
    
    int keyframe_count_;
    std::deque<FusedState> keyframe_states_;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> keyframe_clouds_;
    
    // FAST-LIVO2风格的前端处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_cloud_;
    Eigen::Matrix4d last_transform_;
    
    // 初始化相关
    bool init_with_map_;
    bool init_with_qr_;
    bool init_with_gps_;
    double init_timeout_;
    double init_start_time_;
    
    // GPS/RTK相关
    bool gps_available_;
    bool rtk_available_;
    double gps_cov_threshold_;
    double last_gps_time_;
    bool lost_;                 // 是否处于丢失状态
    int relocalization_fail_cnt_;
    double last_reloc_time_;
    
    // 参数
    double output_frequency_;  // 100Hz
    double lidar_frequency_;
    int max_keyframe_num_;
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    double voxel_leaf_size_;

    // 动态去除与鲁棒配准参数
    bool dynamic_remove_enable_;
    int sor_mean_k_;
    double sor_std_mul_;
    double icp_max_corr_dist_;
    int icp_max_iterations_;
    double icp_fitness_threshold_;
    double ndt_resolution_;
    double ndt_fitness_threshold_;
    double gicp_fitness_threshold_;
    double relocalization_fitness_threshold_;
    double relocalization_min_inlier_ratio_;
    double relocalization_timeout_;
    int init_max_trials_;
    bool scan_context_enable_;
    int scan_context_topk_;
    bool teaser_enable_;
    double teaser_max_corr_dist_;
    double teaser_noise_bound_;
    
    // 点云处理
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
    
    // 回调函数
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr& msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstPtr& msg);
    void rtkCallback(const nav_msgs::msg::Odometry::ConstPtr& msg);
    
    // 核心处理函数
    void processData();
    bool synchronizeData();
    
    // 初始化函数
    bool initializeSystem();
    bool initializeWithPriorMap();
    bool initializeWithQRCode();
    bool initializeWithGPS();
    bool initializeWithMotion();
    
    // 前端处理（FAST-LIVO2风格）
    void processFrontend();
    void filterDynamicPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    OdometryResult estimateOdometryRobust(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
                                          const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2);
    bool evaluateMatch(const OdometryResult& result) const;
    bool needRelocalization(const OdometryResult& result) const;
    bool tryRelocalization(OdometryResult& out_result);
    std::vector<CandidatePose> generateScanContextCandidates();
    bool registerWithTeaser(const pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& tgt,
                            Eigen::Matrix4d& T);
    CandidatePose refineWithNDTGICP(const Eigen::Matrix4d& init_guess);
    
    // 后端优化（LIO-SAM风格）
    void processBackend();
    void addOdometryFactor();
    void addGPSFactor();
    void addQRCodeFactor();
    void addPriorMapFactor();
    void optimizeGraph();
    
    // 二维码检测
    bool detectQRCode(const cv::Mat& image, std::vector<QRCodeInfo>& detected_qr);
    Eigen::Matrix4d estimatePoseFromQRCode(const QRCodeInfo& qr_info, 
                                           const cv::Mat& image);
    
    // 地图匹配
    bool matchWithPriorMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          Eigen::Matrix4d& transform);
    
    // 高频输出
    void publish100Hz();
    
    // 工具函数
    Eigen::Matrix4d stateToTransform(const FusedState& state);
    FusedState transformToState(const Eigen::Matrix4d& transform, double timestamp);
    void loadQRCodeDatabase(const std::string& config_path);
    void loadPriorMap(const std::string& map_path);
    
    // 参数读取
    void loadParameters();
};

#endif // FUSED_LOCALIZATION_H

