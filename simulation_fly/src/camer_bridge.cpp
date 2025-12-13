#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <mutex>
#include <cstring>
#include <csignal>
#include <optional>

class GzToRos2ImageBridge : public rclcpp::Node
{
public:
    GzToRos2ImageBridge()
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

        gz_rgb_topic_  = this->get_parameter("gz_rgb_topic").as_string();
        ros_rgb_topic_ = this->get_parameter("agiros_rgb_topic").as_string();
        gz_depth_topic_ = this->get_parameter("gz_depth_topic").as_string();
        ros_depth_topic_ = this->get_parameter("agiros_depth_topic").as_string();
        rgb_frame_id_ = this->get_parameter("rgb_frame_id").as_string();
        depth_frame_id_ = this->get_parameter("depth_frame_id").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();

        // ---------------- 发布器 ----------------
    
        
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_rgb_topic_, 100);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_depth_topic_, 100);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 100);

        // ---------------- CameraInfo 发布器 ----------------
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        info_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_rgb_topic_ + "_camera_info", info_qos);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_depth_topic_ + "_camera_info", info_qos);

        // 初始化camera_info
        initializeCameraInfo();

        // ---------------- Gazebo 订阅 ----------------
        if (!gz_node_.Subscribe(gz_rgb_topic_, &GzToRos2ImageBridge::rgbCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 RGB 失败: %s", gz_rgb_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 RGB: %s", gz_rgb_topic_.c_str());

        if (!gz_node_.Subscribe(gz_depth_topic_, &GzToRos2ImageBridge::depthCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Depth 失败: %s", gz_depth_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Depth: %s", gz_depth_topic_.c_str());

        if (!gz_node_.Subscribe("/imu", &GzToRos2ImageBridge::gzImuCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Gazebo IMU 失败: /imu");
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Gazebo IMU: /imu");

        RCLCPP_INFO(this->get_logger(), "Gazebo → ROS2 桥启动");
        RCLCPP_INFO(this->get_logger(), "RGB 发布到: %s", ros_rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Depth 发布到: %s", ros_depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU 发布到: %s", imu_topic_.c_str());

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
    
    // CameraInfo发布器
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_, depth_info_pub_;

    std::string gz_rgb_topic_, ros_rgb_topic_;
    std::string gz_depth_topic_, ros_depth_topic_;
    std::string rgb_frame_id_, depth_frame_id_;
    std::string imu_topic_, imu_frame_id_;

    // CameraInfo消息
    sensor_msgs::msg::CameraInfo rgb_camera_info_, depth_camera_info_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GzToRos2ImageBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}