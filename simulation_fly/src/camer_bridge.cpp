#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <mutex>
#include <cstring>
#include <csignal>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gz/msgs/clock.pb.h>
#include <memory>
#include <optional>

class GzToRos2Bridge : public rclcpp::Node
{
public:
    GzToRos2Bridge()
        : Node("gz_to_agiros_bridge")
    {
        // ---------------- 参数 ----------------
        this->declare_parameter<std::string>("gz_rgb_topic", "/camera/image");
        this->declare_parameter<std::string>("agiros_rgb_topic", "/drone_0_/camera/image_raw");
        this->declare_parameter<std::string>("gz_depth_topic", "/camera/depth_image");
        this->declare_parameter<std::string>("agiros_depth_topic", "/drone_0_/camera/depth/image_raw");
        this->declare_parameter<std::string>("rgb_frame_id", "camera_link");
        this->declare_parameter<std::string>("depth_frame_id", "camera_link");
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<std::string>("imu_frame_id", "camera_link");
        this->declare_parameter<std::string>("gz_lidar_scan_topic", "/lidar");
        this->declare_parameter<std::string>("agiros_lidar_scan_topic", "/drone_0_/lidar/scan");
        this->declare_parameter<std::string>("gz_lidar_points_topic", "/lidar/points");
        this->declare_parameter<std::string>("agiros_lidar_points_topic", "/drone_0_/lidar/points");
        this->declare_parameter<std::string>("lidar_frame_id", "world");

        gz_rgb_topic_  = this->get_parameter("gz_rgb_topic").as_string();
        ros_rgb_topic_ = this->get_parameter("agiros_rgb_topic").as_string();
        gz_depth_topic_ = this->get_parameter("gz_depth_topic").as_string();
        ros_depth_topic_ = this->get_parameter("agiros_depth_topic").as_string();
        rgb_frame_id_ = this->get_parameter("rgb_frame_id").as_string();
        depth_frame_id_ = this->get_parameter("depth_frame_id").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
        gz_lidar_scan_topic_ = this->get_parameter("gz_lidar_scan_topic").as_string();
        ros_lidar_scan_topic_ = this->get_parameter("agiros_lidar_scan_topic").as_string();
        gz_lidar_points_topic_ = this->get_parameter("gz_lidar_points_topic").as_string();
        ros_lidar_points_topic_ = this->get_parameter("agiros_lidar_points_topic").as_string();
        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();

        // ---------------- 发布器 ----------------
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_rgb_topic_, 100);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_depth_topic_, 100);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 100);
        lidar_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(ros_lidar_scan_topic_, 100);
        lidar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_lidar_points_topic_, 100);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // ---------------- CameraInfo 发布器 ----------------
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        info_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_rgb_topic_ + "_camera_info", info_qos);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_depth_topic_ + "_camera_info", info_qos);

        // 初始化camera_info
        initializeCameraInfo();

        // ---------------- Gazebo 订阅 ----------------
        // RGB
        if (!gz_node_.Subscribe(gz_rgb_topic_, &GzToRos2Bridge::rgbCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 RGB 失败: %s", gz_rgb_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 RGB: %s", gz_rgb_topic_.c_str());

        //time
        if(!gz_node_.Subscribe("/clock", &GzToRos2Bridge::gzClockCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Gazebo Clock 失败: /clock");
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Gazebo Clock: /clock");

        // Depth
        if (!gz_node_.Subscribe(gz_depth_topic_, &GzToRos2Bridge::depthCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Depth 失败: %s", gz_depth_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Depth: %s", gz_depth_topic_.c_str());

        // IMU
        if (!gz_node_.Subscribe("/imu", &GzToRos2Bridge::gzImuCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Gazebo IMU 失败: /imu");
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Gazebo IMU: /imu");

        // 激光雷达扫描数据
        if (!gz_node_.Subscribe(gz_lidar_scan_topic_, &GzToRos2Bridge::lidarScanCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅激光雷达扫描失败: %s", gz_lidar_scan_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅激光雷达扫描: %s", gz_lidar_scan_topic_.c_str());

        // 激光雷达点云数据
        if (!gz_node_.Subscribe(gz_lidar_points_topic_, &GzToRos2Bridge::lidarPointsCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅激光雷达点云失败: %s", gz_lidar_points_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅激光雷达点云: %s", gz_lidar_points_topic_.c_str());

        RCLCPP_INFO(this->get_logger(), "Gazebo → ROS2 桥启动");
        RCLCPP_INFO(this->get_logger(), "RGB 发布到: %s", ros_rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Depth 发布到: %s", ros_depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU 发布到: %s", imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Lidar Scan 发布到: %s", ros_lidar_scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Lidar Points 发布到: %s", ros_lidar_points_topic_.c_str());

        std::signal(SIGINT, [](int){
            rclcpp::shutdown();
        });
    }

private:
    void initializeCameraInfo()
    {
        // RGB相机信息
        rgb_camera_info_.header.frame_id = rgb_frame_id_;
        rgb_camera_info_.height = 480;
        rgb_camera_info_.width = 640;
        rgb_camera_info_.distortion_model = "plumb_bob";
        rgb_camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        rgb_camera_info_.k = {554.254691191187, 0.0, 320.5,
                              0.0, 554.254691191187, 240.5,
                              0.0, 0.0, 1.0};
        rgb_camera_info_.r = {1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0};
        rgb_camera_info_.p = {554.254691191187, 0.0, 320.5, 0.0,
                              0.0, 554.254691191187, 240.5, 0.0,
                              0.0, 0.0, 1.0, 0.0};

        // 深度相机信息（与RGB相同）
        depth_camera_info_ = rgb_camera_info_;
        depth_camera_info_.header.frame_id = depth_frame_id_;
    }

    // ---------------- RGB / Depth 回调 ----------------
    void rgbCallback(const gz::msgs::Image &gz_msg)
    {
        auto ros_msg = convertToRosImage(gz_msg, rgb_frame_id_);
        
        // 设置camera_info的时间戳与图像一致
        rgb_camera_info_.header.stamp = ros_msg.header.stamp;
        
        rgb_pub_->publish(ros_msg);
        rgb_info_pub_->publish(rgb_camera_info_);
    }

    void depthCallback(const gz::msgs::Image &gz_msg)
    {
        auto ros_msg = convertToRosImage(gz_msg, depth_frame_id_);
        
        // 设置camera_info的时间戳与图像一致
        depth_camera_info_.header.stamp = ros_msg.header.stamp;
        
        depth_pub_->publish(ros_msg);
        depth_info_pub_->publish(depth_camera_info_);
    }

     void gzClockCallback(const gz::msgs::Clock &_msg)
    {
        rosgraph_msgs::msg::Clock ros_clock_msg;
        ros_clock_msg.clock.sec = _msg.sim().sec();
        ros_clock_msg.clock.nanosec = _msg.sim().nsec();

        // 发布到 ROS2
        clock_pub_->publish(ros_clock_msg);
    }

    // ---------------- Gazebo IMU 回调 ----------------
    void gzImuCallback(const gz::msgs::IMU &gz_msg)
    {
        sensor_msgs::msg::Imu ros_msg;

        // 时间戳
        auto s = gz_msg.header().stamp();
        ros_msg.header.stamp.sec = s.sec();
        ros_msg.header.stamp.nanosec = s.nsec();
        ros_msg.header.frame_id = imu_frame_id_;

        // 线加速度
        ros_msg.linear_acceleration.x = gz_msg.linear_acceleration().x();
        ros_msg.linear_acceleration.y = gz_msg.linear_acceleration().y();
        ros_msg.linear_acceleration.z = gz_msg.linear_acceleration().z();

        // 角速度
        ros_msg.angular_velocity.x = gz_msg.angular_velocity().x();
        ros_msg.angular_velocity.y = gz_msg.angular_velocity().y();
        ros_msg.angular_velocity.z = gz_msg.angular_velocity().z();

        // 四元数
        ros_msg.orientation.x = gz_msg.orientation().x();
        ros_msg.orientation.y = gz_msg.orientation().y();
        ros_msg.orientation.z = gz_msg.orientation().z();
        ros_msg.orientation.w = gz_msg.orientation().w();

        // 协方差置0
        for (int i=0; i<9; ++i) {
            ros_msg.orientation_covariance[i] = 0.0;
            ros_msg.angular_velocity_covariance[i] = 0.0;
            ros_msg.linear_acceleration_covariance[i] = 0.0;
        }

        imu_pub_->publish(ros_msg);
    }

    // ---------------- 激光雷达扫描数据回调 ----------------
    void lidarScanCallback(const gz::msgs::LaserScan &gz_msg)
    {
        sensor_msgs::msg::LaserScan ros_msg;

        // 时间戳
        auto s = gz_msg.header().stamp();
        ros_msg.header.stamp.sec = s.sec();
        ros_msg.header.stamp.nanosec = s.nsec();
        ros_msg.header.frame_id = lidar_frame_id_;

        // 设置雷达参数
        ros_msg.angle_min = gz_msg.angle_min();
        ros_msg.angle_max = gz_msg.angle_max();
        ros_msg.angle_increment = gz_msg.angle_step();
        ros_msg.time_increment = 0.0;  // 固定扫描，没有时间增量
        ros_msg.scan_time = 0.0;       // 固定扫描，没有扫描时间
        ros_msg.range_min = gz_msg.range_min();
        ros_msg.range_max = gz_msg.range_max();

        // 复制距离数据
        ros_msg.ranges.resize(gz_msg.ranges_size());
        for (int i = 0; i < gz_msg.ranges_size(); ++i) {
            ros_msg.ranges[i] = gz_msg.ranges(i);
        }

        // 复制强度数据（如果存在）
        if (gz_msg.intensities_size() > 0) {
            ros_msg.intensities.resize(gz_msg.intensities_size());
            for (int i = 0; i < gz_msg.intensities_size(); ++i) {
                ros_msg.intensities[i] = gz_msg.intensities(i);
            }
        } else {
            ros_msg.intensities.clear();
        }

        lidar_scan_pub_->publish(ros_msg);
    }

    // ---------------- 激光雷达点云数据回调 ----------------
    void lidarPointsCallback(const gz::msgs::PointCloudPacked &gz_msg)
    {
        sensor_msgs::msg::PointCloud2 ros_msg;

        // 时间戳
        auto s = gz_msg.header().stamp();
        ros_msg.header.stamp.sec = s.sec();
        ros_msg.header.stamp.nanosec = s.nsec();
        ros_msg.header.frame_id = lidar_frame_id_;

        // 设置点云参数
        ros_msg.height = 1;  // 非组织化点云
        ros_msg.width = gz_msg.data().size() / gz_msg.point_step();
        ros_msg.is_bigendian = false;
        ros_msg.point_step = gz_msg.point_step();
        ros_msg.row_step = ros_msg.width * ros_msg.point_step;
        ros_msg.is_dense = true;

        // 设置字段
        ros_msg.fields.clear();
        for (const auto &field : gz_msg.field()) {
            sensor_msgs::msg::PointField pf;
            pf.name = field.name();
            pf.offset = field.offset();
            
            // 转换数据类型
            switch (field.datatype()) {
                case gz::msgs::PointCloudPacked::Field::INT8:
                    pf.datatype = sensor_msgs::msg::PointField::INT8;
                    break;
                case gz::msgs::PointCloudPacked::Field::UINT8:
                    pf.datatype = sensor_msgs::msg::PointField::UINT8;
                    break;
                case gz::msgs::PointCloudPacked::Field::INT16:
                    pf.datatype = sensor_msgs::msg::PointField::INT16;
                    break;
                case gz::msgs::PointCloudPacked::Field::UINT16:
                    pf.datatype = sensor_msgs::msg::PointField::UINT16;
                    break;
                case gz::msgs::PointCloudPacked::Field::INT32:
                    pf.datatype = sensor_msgs::msg::PointField::INT32;
                    break;
                case gz::msgs::PointCloudPacked::Field::UINT32:
                    pf.datatype = sensor_msgs::msg::PointField::UINT32;
                    break;
                case gz::msgs::PointCloudPacked::Field::FLOAT32:
                    pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
                    break;
                case gz::msgs::PointCloudPacked::Field::FLOAT64:
                    pf.datatype = sensor_msgs::msg::PointField::FLOAT64;
                    break;
                default:
                    pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
            }
            
            pf.count = 1;
            ros_msg.fields.push_back(pf);
        }

        // 复制数据
        ros_msg.data.resize(gz_msg.data().size());
        std::memcpy(ros_msg.data.data(), gz_msg.data().data(), gz_msg.data().size());

        lidar_points_pub_->publish(ros_msg);
    }

    // ---------------- Gazebo Image → ROS Image ----------------
    sensor_msgs::msg::Image convertToRosImage(const gz::msgs::Image &gz_msg,
                                              const std::string &frame_id)
    {
        sensor_msgs::msg::Image ros_msg;

        if (gz_msg.has_header())
        {
            auto s = gz_msg.header().stamp();
            ros_msg.header.stamp.sec = s.sec();
            ros_msg.header.stamp.nanosec = s.nsec();
        }
        else
        {
            ros_msg.header.stamp = this->now();
        }

        ros_msg.header.frame_id = frame_id;
        ros_msg.height = gz_msg.height();
        ros_msg.width = gz_msg.width();
        ros_msg.is_bigendian = false;

        switch (gz_msg.pixel_format_type())
        {
            case gz::msgs::PixelFormatType::RGB_INT8: ros_msg.encoding = "rgb8"; ros_msg.step = ros_msg.width*3; break;
            case gz::msgs::PixelFormatType::RGBA_INT8: ros_msg.encoding = "rgba8"; ros_msg.step = ros_msg.width*4; break;
            case gz::msgs::PixelFormatType::L_INT8: ros_msg.encoding = "mono8"; ros_msg.step = ros_msg.width; break;
            case gz::msgs::PixelFormatType::L_INT16: ros_msg.encoding = "mono16"; ros_msg.step = ros_msg.width*2; break;
            case gz::msgs::PixelFormatType::BGR_INT8: ros_msg.encoding = "bgr8"; ros_msg.step = ros_msg.width*3; break;
            case gz::msgs::PixelFormatType::R_FLOAT32: ros_msg.encoding = "32FC1"; ros_msg.step = ros_msg.width*4; break;
            default: ros_msg.encoding = "rgb8"; ros_msg.step = ros_msg.width*3; break;
        }

        ros_msg.data.resize(gz_msg.data().size());
        std::memcpy(ros_msg.data.data(), gz_msg.data().data(), gz_msg.data().size());

        return ros_msg;
    }

private:
    gz::transport::Node gz_node_;
    
    // 图像和IMU发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_, depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    // 雷达发布器
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_pub_;
    
    // CameraInfo发布器
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_, depth_info_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    std::string gz_rgb_topic_, ros_rgb_topic_;
    std::string gz_depth_topic_, ros_depth_topic_;
    std::string rgb_frame_id_, depth_frame_id_;
    std::string imu_topic_, imu_frame_id_;
    std::string gz_lidar_scan_topic_, ros_lidar_scan_topic_;
    std::string gz_lidar_points_topic_, ros_lidar_points_topic_;
    std::string lidar_frame_id_;

    // CameraInfo消息
    sensor_msgs::msg::CameraInfo rgb_camera_info_, depth_camera_info_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GzToRos2Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}