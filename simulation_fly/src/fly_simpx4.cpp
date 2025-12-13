#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <chrono>
#include <deque>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>


using namespace std::chrono_literals;


#define BUFFER_SIZE 1024
#define TargeyHeight 1.5

std::mutex pid_mutex;
std::mutex label_mutex;
std::mutex a_mutex;

class MavrosSubscriber : public rclcpp::Node {
public:
    MavrosSubscriber() : Node("mavros_subscriber") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos, std::bind(&MavrosSubscriber::state_callback, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&MavrosSubscriber::pose_callback, this, std::placeholders::_1));
    }

    mavros_msgs::msg::State getCurrentState() const { return current_state; }
    geometry_msgs::msg::PoseStamped getCurrentPose() const { return current_pose; }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state = *msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose = *msg;
    }

    mavros_msgs::msg::State current_state;
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

enum class FlightState {
    TAKEOFF = 1,
    Fly_frist,
    Fly_second,
    Fly_third,
    Fly_fourth,
    Fly_fifth,
    Fly_sixth,
    LANDING  
};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Caught Ctrl+C, exiting..." << std::endl;
        rclcpp::shutdown(); 
        exit(0);
    }
}

class Task : public rclcpp::Node {
public:
    Task(std::shared_ptr<MavrosSubscriber> mavros_sub)
    : Node("task"), mavros_subscriber_(mavros_sub), armed(false), 
      flight_state(FlightState::TAKEOFF), cur_state(0), delay_started(false),
      hight_ok(false), fly_dian(true) {
        
        last_request_time = get_clock()->now();
        
        offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        offb_set_mode->custom_mode = "OFFBOARD";
        
        arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_cmd->value = true;

        set_mode_client = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        local_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        set_raw_pub = create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
        set_v_pub = create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        odm_publsiher = this->create_publisher<nav_msgs::msg::Odometry>("/drone_0_/Odometry", 10);
        ego_point_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
  
        ego_ok_sub = this->create_subscription<std_msgs::msg::Float64>("ego_ok", 10, std::bind(&Task::ego_ok_callback, this, std::placeholders::_1));
        pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>("/drone_0_planning/pos_cmd", 10, std::bind(&Task::pos_cmd_callback, this, std::placeholders::_1));
        Serialtimer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Task::runtask, this));
        Serialtimer_odm = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Task::odmpub, this));
    }

private:    
    std::shared_ptr<MavrosSubscriber> mavros_subscriber_;
    bool armed;
    FlightState flight_state;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_point_pub;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr set_raw_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr set_v_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odm_publsiher;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_ok_sub;
    rclcpp::TimerBase::SharedPtr Serialtimer_odm;
    rclcpp::TimerBase::SharedPtr Serialtimer_;
    int cur_state;
    bool delay_started;
    rclcpp::Time delay_start_time;
    bool hight_ok;
    bool fly_dian;
    bool ego_finlish = true;
    bool ego_v_finish = true;
    std::shared_ptr<mavros_msgs::srv::SetMode::Request> offb_set_mode;
    std::shared_ptr<mavros_msgs::srv::CommandBool::Request> arm_cmd;
    rclcpp::Time last_request_time;
    geometry_msgs::msg::PoseStamped target_position;

    void ego_ok_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    constexpr double EPSILON = 1e-6;
    static std::deque<double> buffer;
    
    buffer.push_back(msg->data);
    if (buffer.size() > 5) {
      buffer.pop_front();
    }

    if (buffer.size() < 5) {
      return;
    }

    int zero_count = 0;
    int one_count = 0;

    for (const auto& val : buffer) {
      if (std::abs(val - 0.0) < EPSILON) {
        zero_count++;
      } else if (std::abs(val - 1.0) < EPSILON) {
        one_count++;
      }
    }

    if (zero_count > one_count) {
      ego_finlish = true;
    } else if (one_count > zero_count) {
      ego_finlish = false;
    } else {
      ego_finlish = true;
    }
  }

  void pos_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  { 
    if (!set_raw_pub || !local_pose_pub) {
      RCLCPP_ERROR(this->get_logger(), "Publishers not initialized!");
      return;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    try {
      current_pose = mavros_subscriber_->getCurrentPose();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error getting current pose: %s", e.what());
      return;
    }

    const double target_ned_x = msg->position.x;   
    const double target_ned_y = msg->position.y;   
    const double target_ned_z = msg->position.z;
    auto target = std::make_shared<mavros_msgs::msg::PositionTarget>();
    target->header.stamp = this->now();
    target->header.frame_id = "map";
    target->coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    target->type_mask = 
      mavros_msgs::msg::PositionTarget::IGNORE_AFX | 
      mavros_msgs::msg::PositionTarget::IGNORE_AFY | 
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
    
    target->position.x = target_ned_x;
    target->position.y = target_ned_y;
    target->position.z = TargeyHeight;
    target->velocity.x = msg->velocity.x;  
    target->velocity.y = msg->velocity.y; 
    target->velocity.z =0; 
    target->yaw = msg->yaw;
    
    if(ego_v_finish){
      set_raw_pub->publish(*target);
    }
  }

  void fly_ego(double x, double y, double z) {
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.pose.position.x = x;
    current_pose.pose.position.y = y;
    current_pose.pose.position.z = z;
    ego_point_pub->publish(current_pose);  
  }

  void odmpub() {
    auto current_pose = mavros_subscriber_->getCurrentPose();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose = current_pose.pose;
    odom_msg.header.frame_id = "odom";          // 世界坐标
    odom_msg.child_frame_id = "camera_link";      // 机体坐标
    // Ensure odom header uses this node's clock (same time source as other nodes)
    odom_msg.header = current_pose.header;
    odom_msg.header.stamp = this->now();
    odm_publsiher->publish(odom_msg);
  }

    void fly_dianvv_xy(double x, double y, double z, double yaw) {
        auto target = mavros_msgs::msg::PositionTarget();
        target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX | 
                          mavros_msgs::msg::PositionTarget::IGNORE_AFY | 
                          mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
        target.coordinate_frame = 1;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.yaw = yaw;
        target.velocity.x = 0;
        target.velocity.y = 0;
        target.velocity.z = 0;
        set_raw_pub->publish(target);
    }

    void fly_v_xy(double vx, double vy, double vz, double vyaw) {
        if (fabs(vx) < 0.001 && fabs(vy) < 0.001 && fabs(vz) < 0.001 && fabs(vyaw) < 0.001) {
            return;
        }
        
        auto target = geometry_msgs::msg::Twist();
        target.linear.x = vx;
        target.linear.y = vy;
        target.linear.z = vz;
        target.angular.z = vyaw;
        target.angular.x = 0;
        target.angular.y = 0;
        set_v_pub->publish(target);
    }

    void runtask() {
        auto current_state = mavros_subscriber_->getCurrentState();
        auto current_pose = mavros_subscriber_->getCurrentPose();
        
         if(current_state.mode != "OFFBOARD" && 
           (this->get_clock()->now() - last_request_time > rclcpp::Duration::from_seconds(5.0)))
        {
            // 使用异步调用
            set_mode_client->async_send_request(offb_set_mode);
            RCLCPP_INFO(this->get_logger(), "Trying to enable Offboard mode");
            last_request_time = this->get_clock()->now();
        }
        else
        {
            if(!current_state.armed &&
               (this->get_clock()->now() - last_request_time > rclcpp::Duration::from_seconds(5.0)))
            {
                arming_client->async_send_request(arm_cmd);
                RCLCPP_INFO(this->get_logger(), "Trying to arm vehicle");
                armed = true; // 暂时设置为true，实际项目中应该通过回调确认
                last_request_time = this->get_clock()->now();
            } 
        }

        if (armed) {
            switch(flight_state) {
        case FlightState::TAKEOFF:
        {
          handle_takeoff(current_pose);
          break;
        }
        case FlightState::Fly_frist:
        {
         ego_task(current_pose,6,1);
          break;
        }
        case FlightState::Fly_second:
        {
           ego_task(current_pose,0,0);
          break;
        }
        case FlightState::Fly_third:
        {
        ego_task(current_pose,6,3);
          break;
        }
        case FlightState::Fly_fourth:
        {
         ego_task(current_pose,0,0);
          break;
        }
        case FlightState::Fly_fifth:
        {
         ego_task(current_pose,0,3);
          break;
        }
        case FlightState::Fly_sixth:
        {
         ego_task(current_pose,0,0);
          break;
        }

        
        case FlightState::LANDING:
        {
          handle_landing(current_pose);
          break;
        }
      }

        }
    }

   void handle_takeoff(const geometry_msgs::msg::PoseStamped& current_pose) {
    if(current_pose.pose.position.z < TargeyHeight-0.01) {
      if(cur_state == 0) {
        target_position = current_pose;
        target_position.pose.position.x = 0;
        target_position.pose.position.y = 0;
        target_position.pose.position.z = TargeyHeight;
        cur_state = 1;
      }
      RCLCPP_INFO(this->get_logger(), "%lf", current_pose.pose.position.z);
      if(!hight_ok){
        local_pose_pub->publish(target_position);
      }
    }
    else {
      if(!delay_started) {
        delay_start_time = this->get_clock()->now();
        delay_started = true;
      }
      auto current_time = this->get_clock()->now();
      auto elapsed = (current_time - delay_start_time).seconds();
      if(elapsed >= 1.0) {
        flight_state = FlightState::Fly_frist;
        delay_started = false;
        hight_ok = true;
        ego_v_finish = true;
        target_position = current_pose;
        target_position.pose.position.x = 0;
        target_position.pose.position.y = 0;
        target_position.pose.position.z = TargeyHeight;
        fly_dianvv_xy(0, 0, TargeyHeight, 0);
        RCLCPP_INFO(this->get_logger(), "step2");
      }
    }
  }
  
  void handle_landing(const geometry_msgs::msg::PoseStamped& current_pose) {
    if(current_pose.pose.position.z > 0.05) {
      if(cur_state == 1) {
        target_position = current_pose;
        target_position.pose.position.z = -0.2;
        cur_state = 0;
      }
      local_pose_pub->publish(target_position);
    }
    else {
      if(!delay_started) {
        delay_start_time = this->get_clock()->now();
        delay_started = true;
        RCLCPP_INFO(this->get_logger(), "Landing completed, waiting 1 second before disarming...");
      }
      auto current_time = this->get_clock()->now();
      auto elapsed = (current_time - delay_start_time).seconds();
      if(elapsed >= 1.0) {
        auto disarm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        disarm_cmd->value = false;
        arming_client->async_send_request(disarm_cmd);
        RCLCPP_INFO(this->get_logger(), "Trying to disarm vehicle");
        armed = false;
        delay_started = false;
        rclcpp::shutdown();
      }
    }
  }
  
  void ego_task(const geometry_msgs::msg::PoseStamped& current_pose, double x, double y) {
     if(!delay_started) {
            delay_start_time = this->get_clock()->now();
            delay_started = true;
            fly_ego(x, y, TargeyHeight);
          }
          auto current_time = this->get_clock()->now();
          auto elapsed = (current_time - delay_start_time).seconds();
          if (elapsed >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "ego erro (%lf, %lf)",fabs(current_pose.pose.position.x - x), fabs(current_pose.pose.position.y - y));
            if( ego_finlish &&
               (fabs(current_pose.pose.position.x - x) < 0.05) && 
               (fabs(current_pose.pose.position.y - y) < 0.05)) {
              switch(flight_state) {
                case FlightState::Fly_frist:
                  delay_started = false;
                  ego_v_finish = true;
                  flight_state = FlightState::Fly_second;
                  RCLCPP_INFO(this->get_logger(), "step3");
                  break;
                case FlightState::Fly_second:
                  delay_started = false;
                 ego_v_finish = true;
                    //ego_v_finish = false;
                 flight_state = FlightState::Fly_third;
                  RCLCPP_INFO(this->get_logger(), "step4");
                  break;
                case FlightState::Fly_third:
                  delay_started = false;
                  ego_v_finish = true;
                  flight_state = FlightState::Fly_fourth;
                  RCLCPP_INFO(this->get_logger(), "step5");
                  break;
                case FlightState::Fly_fourth:
                  delay_started = false;
                  ego_v_finish = true;
                  flight_state = FlightState::Fly_fifth;  
                  RCLCPP_INFO(this->get_logger(), "step6");
                  break;
                case FlightState::Fly_fifth:
                  delay_started = false;
                  ego_v_finish = true;
                  flight_state = FlightState::Fly_sixth;  
                  RCLCPP_INFO(this->get_logger(), "step7");
                  break;
                case FlightState::Fly_sixth:
                  delay_started = false;
                  ego_v_finish = false;
                  flight_state = FlightState::LANDING;  
                  RCLCPP_INFO(this->get_logger(), "landing");
                  break;



                default:
                  break;
              }
            }
          }
        }

};

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);

    
    rclcpp::executors::MultiThreadedExecutor executor;

    auto MavrosSubscribernode = std::make_shared<MavrosSubscriber>();

    auto Tasknode = std::make_shared<Task>(MavrosSubscribernode);

    

    executor.add_node(MavrosSubscribernode);
    executor.add_node(Tasknode);

    
    executor.spin();
    rclcpp::shutdown(); 
    return 0;
}