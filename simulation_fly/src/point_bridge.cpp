#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <limits>
#include <cstring>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer()
        : Node("point_cloud_transformer")
    {
        // 声明参数
        this->declare_parameter<std::string>("input_cloud_topic", "/rtabmap/odom_local_map");
        this->declare_parameter<std::string>("output_cloud_topic", "/transformed_cloud");
        this->declare_parameter<std::string>("target_frame", "world");
        
        // 转换模式
        this->declare_parameter<std::string>("transformation_mode", "optical_transform");
        // 可选值: "frame_id_only", "optical_transform", "pose_transform"
        
        // 相机位姿参数
        this->declare_parameter<double>("cam_w_tx", 0.0);
        this->declare_parameter<double>("cam_w_ty", 0.0);
        this->declare_parameter<double>("cam_w_tz", 0.0);
        this->declare_parameter<double>("cam_w_qx", 0.0);
        this->declare_parameter<double>("cam_w_qy", 0.0);
        this->declare_parameter<double>("cam_w_qz", 0.0);
        this->declare_parameter<double>("cam_w_qw", 1.0);
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_cloud_topic").as_string();
        std::string output_topic = this->get_parameter("output_cloud_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        transformation_mode_ = this->get_parameter("transformation_mode").as_string();

        cam_w_tx_ = this->get_parameter("cam_w_tx").as_double();
        cam_w_ty_ = this->get_parameter("cam_w_ty").as_double();
        cam_w_tz_ = this->get_parameter("cam_w_tz").as_double();
        cam_w_qx_ = this->get_parameter("cam_w_qx").as_double();
        cam_w_qy_ = this->get_parameter("cam_w_qy").as_double();
        cam_w_qz_ = this->get_parameter("cam_w_qz").as_double();
        cam_w_qw_ = this->get_parameter("cam_w_qw").as_double();

        // 创建发布者和订阅者
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudTransformer::cloudCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "点云变换器已启动");
        RCLCPP_INFO(this->get_logger(), "输入话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "输出话题: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "目标坐标系: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "转换模式: %s", transformation_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "======================================");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "收到点云: 宽度=%d, 高度=%d, 点数=%d", 
                    msg->width, msg->height, (int)(msg->width * msg->height));
        
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        
        if (transformation_mode_ == "frame_id_only") {
            transformed_cloud = changeFrameOnly(*msg);
        } else if (transformation_mode_ == "optical_transform") {
            transformed_cloud = transformPointCloudOptical(*msg);
        } else if (transformation_mode_ == "pose_transform") {
            transformed_cloud = transformPointCloudWithPose(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "未知转换模式: %s, 使用 frame_id_only", transformation_mode_.c_str());
            transformed_cloud = changeFrameOnly(*msg);
        }
        
        auto end_time = this->now();
        auto duration = end_time - start_time;
        
        RCLCPP_DEBUG(this->get_logger(), "转换完成: 输入点数=%d, 输出点数=%d, 耗时=%.3fms",
                    (int)(msg->width * msg->height), 
                    (int)(transformed_cloud.width * transformed_cloud.height),
                    duration.seconds() * 1000.0);
        
        cloud_pub_->publish(transformed_cloud);
    }
    
    // 模式1: 只改变坐标系，不修改点数据
    sensor_msgs::msg::PointCloud2 changeFrameOnly(const sensor_msgs::msg::PointCloud2& input_cloud)
    {
        sensor_msgs::msg::PointCloud2 output_cloud = input_cloud;
        output_cloud.header.stamp = this->now();
        output_cloud.header.frame_id = target_frame_;
        return output_cloud;
    }
    
    // 模式2: 光学坐标系转换
    sensor_msgs::msg::PointCloud2 transformPointCloudOptical(const sensor_msgs::msg::PointCloud2& input_cloud)
    {
        sensor_msgs::msg::PointCloud2 output_cloud;
        
        // 复制基本结构
        output_cloud.header.stamp = this->now();
        output_cloud.header.frame_id = target_frame_;
        output_cloud.height = input_cloud.height;
        output_cloud.width = input_cloud.width;
        output_cloud.fields = input_cloud.fields;
        output_cloud.is_bigendian = input_cloud.is_bigendian;
        output_cloud.point_step = input_cloud.point_step;
        output_cloud.row_step = input_cloud.row_step;
        output_cloud.is_dense = input_cloud.is_dense;
        
        // 预分配数据空间
        output_cloud.data.resize(input_cloud.data.size());
        
        // 创建点云迭代器
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(input_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(input_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(input_cloud, "z");
        
        sensor_msgs::PointCloud2Iterator<float> out_iter_x(output_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(output_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(output_cloud, "z");
        
        int valid_points = 0;
        
        // 转换每个点
        for (; iter_x != iter_x.end(); 
             ++iter_x, ++iter_y, ++iter_z, 
             ++out_iter_x, ++out_iter_y, ++out_iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
            {
                // 光学坐标系转换
                // 光学坐标系: X_right, Y_down, Z_forward
                // ROS坐标系: X_forward, Y_left, Z_up
                *out_iter_x = z;      // forward
                *out_iter_y = -x;     // left
                *out_iter_z = -y;     // up
                valid_points++;
            }
            else
            {
                // 无效点
                *out_iter_x = std::numeric_limits<float>::quiet_NaN();
                *out_iter_y = std::numeric_limits<float>::quiet_NaN();
                *out_iter_z = std::numeric_limits<float>::quiet_NaN();
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "光学转换: 有效点数=%d/%d", 
                    valid_points, (int)(input_cloud.width * input_cloud.height));
        
        return output_cloud;
    }
    
    // 模式3: 带位姿变换的光学转换
    sensor_msgs::msg::PointCloud2 transformPointCloudWithPose(const sensor_msgs::msg::PointCloud2& input_cloud)
    {
        sensor_msgs::msg::PointCloud2 output_cloud;
        
        // 复制基本结构
        output_cloud.header.stamp = this->now();
        output_cloud.header.frame_id = target_frame_;
        output_cloud.height = input_cloud.height;
        output_cloud.width = input_cloud.width;
        output_cloud.fields = input_cloud.fields;
        output_cloud.is_bigendian = input_cloud.is_bigendian;
        output_cloud.point_step = input_cloud.point_step;
        output_cloud.row_step = input_cloud.row_step;
        output_cloud.is_dense = input_cloud.is_dense;
        
        // 预分配数据空间
        output_cloud.data.resize(input_cloud.data.size());
        
        // 相机到world的位姿
        tf2::Quaternion q(cam_w_qx_, cam_w_qy_, cam_w_qz_, cam_w_qw_);
        tf2::Matrix3x3 R(q);
        const tf2::Vector3 t(cam_w_tx_, cam_w_ty_, cam_w_tz_);
        
        // 创建点云迭代器
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(input_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(input_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(input_cloud, "z");
        
        sensor_msgs::PointCloud2Iterator<float> out_iter_x(output_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(output_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(output_cloud, "z");
        
        int valid_points = 0;
        
        // 转换每个点
        for (; iter_x != iter_x.end(); 
             ++iter_x, ++iter_y, ++iter_z, 
             ++out_iter_x, ++out_iter_y, ++out_iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0)
            {
                // 1. 光学坐标系转换
                float x_optical = z;      // forward
                float y_optical = -x;     // left  
                float z_optical = -y;     // up
                
                // 2. 应用相机位姿变换
                tf2::Vector3 p_optical(x_optical, y_optical, z_optical);
                tf2::Vector3 p_world = R * p_optical + t;
                
                *out_iter_x = static_cast<float>(p_world.x());
                *out_iter_y = static_cast<float>(p_world.y());
                *out_iter_z = static_cast<float>(p_world.z());
                valid_points++;
            }
            else
            {
                // 无效点
                *out_iter_x = std::numeric_limits<float>::quiet_NaN();
                *out_iter_y = std::numeric_limits<float>::quiet_NaN();
                *out_iter_z = std::numeric_limits<float>::quiet_NaN();
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "位姿变换: 有效点数=%d/%d", 
                    valid_points, (int)(input_cloud.width * input_cloud.height));
        
        return output_cloud;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    std::string target_frame_{"world"};
    std::string transformation_mode_{"frame_id_only"};
    
    double cam_w_tx_{0.0};
    double cam_w_ty_{0.0};
    double cam_w_tz_{0.0};
    double cam_w_qx_{0.0};
    double cam_w_qy_{0.0};
    double cam_w_qz_{0.0};
    double cam_w_qw_{1.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}