# Fused Localization System

高性能融合定位系统，整合FAST-LIVO2前端和LIO-SAM后端，支持先验地图、GPS/RTK、二维码定位，实现100Hz高频输出。

## 系统特性

### 核心功能
- **多传感器融合**：融合LiDAR、IMU、相机和GPS/RTK数据
- **快速初始化**：支持多种初始化方式（GPS、二维码、先验地图、运动初始化）
- **高精度定位**：结合FAST-LIVO2的直接法和LIO-SAM的因子图优化
- **高频输出**：100Hz定位输出频率
- **先验地图支持**：利用先验点云地图进行全局定位
- **二维码定位**：在关键区域（如充电站、加水站）提供厘米级定位精度

### 技术架构

#### 前端处理（FAST-LIVO2风格）
- 直接法处理LiDAR和图像数据
- 快速里程计估计
- 实时状态更新

#### 后端优化（LIO-SAM风格）
- 基于GTSAM的因子图优化
- 支持多种因子类型：
  - 里程计因子
  - GPS/RTK因子
  - 二维码因子（高精度）
  - 先验地图因子
  - 回环检测因子（可选）

## 快速初始化策略

当GPS信号不佳时，系统按以下优先级尝试初始化：

1. **RTK定位**（最高精度，厘米级）
2. **GPS定位**（米级精度）
3. **二维码定位**（厘米级精度，需要可见二维码）
4. **先验地图匹配**（使用NDT算法）
5. **运动初始化**（需要短距离运动）

### 二维码定位性能

- **精度**：厘米级（2-5cm）
- **范围**：取决于相机视野和二维码尺寸
- **适用场景**：充电站、加水站等需要极高精度的关键区域
- **限制**：需要良好的光照条件，二维码不能被遮挡

## 安装和编译

### 依赖项

```bash
# ROS2 (Humble或更高版本)
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-nav-msgs
sudo apt install ros-humble-geometry-msgs ros-humble-tf2 ros-humble-tf2-ros
sudo apt install ros-humble-pcl-ros ros-humble-cv-bridge ros-humble-image-transport

# PCL
sudo apt install libpcl-dev

# OpenCV (包含ArUco)
sudo apt install libopencv-dev

# GTSAM
sudo apt install libgtsam-dev
# 或从源码编译：https://github.com/borglab/gtsam
```

### 编译

```bash
cd ~/your_workspace
colcon build --packages-select fused_localization
source install/setup.bash
```

## 配置

### 1. 配置文件

编辑 `config/fused_localization.yaml`：

```yaml
fused_localization:
  ros__parameters:
    output_frequency: 100.0
    prior_map_path: "/path/to/your_map.pcd"
    init_with_map: true
    init_with_qr: true
    init_with_gps: true
```

### 2. 二维码数据库

编辑 `config/qr_codes.yaml`，添加已知位置的二维码：

```yaml
qr_codes:
  - id: 0
    position: [0.0, 0.0, 0.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    size: 0.1
    description: "充电站入口"
```

### 3. 先验地图

准备点云地图文件（PCD格式），并在配置文件中指定路径。

## 运行

### 启动系统

```bash
ros2 launch fused_localization fused_localization.launch.py
```

### 话题

**订阅：**
- `/lidar/points` (sensor_msgs/PointCloud2) - LiDAR点云
- `/imu/data` (sensor_msgs/Imu) - IMU数据
- `/camera/image` (sensor_msgs/Image) - 相机图像
- `/gps/fix` (sensor_msgs/NavSatFix) - GPS数据
- `/rtk/odometry` (nav_msgs/Odometry) - RTK数据

**发布：**
- `/fused_localization/odometry` (nav_msgs/Odometry) - 100Hz定位输出
- `/fused_localization/path` (nav_msgs/Path) - 轨迹路径
- `/fused_localization/map_aligned` (sensor_msgs/PointCloud2) - 对齐后的地图
- `/fused_localization/status` (std_msgs/String) - 系统状态

## 融合策略说明

### FAST-LIVO2前端优势
- 直接法处理，避免特征提取
- 快速实时处理
- 融合视觉信息提升鲁棒性

### LIO-SAM后端优势
- 因子图优化，全局一致性
- 支持多种约束因子
- 回环检测和优化

### 融合方案
1. **前端**：使用FAST-LIVO2的直接法进行快速里程计估计
2. **后端**：使用LIO-SAM的因子图框架进行全局优化
3. **多源融合**：整合GPS、二维码、先验地图等多种约束
4. **高频输出**：通过定时器实现100Hz输出，插值当前状态

## 性能优化建议

1. **初始化**：
   - 在关键位置（如充电站）放置二维码，确保快速初始化
   - 准备高质量的先验地图
   - 确保GPS/RTK在开阔区域可用

2. **实时性**：
   - 调整关键帧阈值，平衡精度和计算量
   - 使用体素滤波减少点云数量
   - 优化GTSAM参数

3. **精度**：
   - 在关键区域使用二维码提供高精度约束
   - 定期进行回环检测
   - 保持先验地图的更新

## 故障排除

### 初始化失败
- 检查GPS信号质量
- 确认二维码是否可见且已注册
- 验证先验地图路径是否正确
- 增加初始化超时时间

### 定位漂移
- 检查传感器标定（特别是外参）
- 增加GPS/RTK约束权重
- 使用更多二维码约束
- 检查先验地图质量

### 输出频率不足
- 检查CPU负载
- 减少点云处理量
- 调整关键帧频率
- 优化GTSAM参数

## 许可证

MIT License

## 参考文献

- FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry
- LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping
- GTSAM: A library for smoothing and mapping

## 贡献

欢迎提交Issue和Pull Request！

