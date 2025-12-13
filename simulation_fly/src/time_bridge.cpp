#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <memory>
#include <mutex>

class GazeboTimeBridge : public rclcpp::Node
{
public:
    GazeboTimeBridge()
        : Node("gazebo_time_bridge")
    {
        // ROS2 Clock 发布器
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Gazebo Node
        gz_node_ = std::make_shared<gz::transport::Node>();

        // 订阅 Gazebo clock
        gz_node_->Subscribe("/clock", &GazeboTimeBridge::gzClockCallback, this);

        RCLCPP_INFO(this->get_logger(), "Gazebo ↔ ROS2 Clock bridge started.");
    }

private:
    void gzClockCallback(const gz::msgs::Clock &_msg)
    {
        rosgraph_msgs::msg::Clock ros_clock_msg;
        ros_clock_msg.clock.sec = _msg.sim().sec();
        ros_clock_msg.clock.nanosec = _msg.sim().nsec();

        // 发布到 ROS2
        clock_pub_->publish(ros_clock_msg);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    std::shared_ptr<gz::transport::Node> gz_node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboTimeBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
