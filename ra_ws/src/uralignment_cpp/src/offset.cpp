#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class OffsetNode : public rclcpp::Node 
{
public:
  OffsetNode() : Node("offset_node") 
  {
    // Parameters
    f_contact_ = declare_parameter<double>("f_contact", 10.0); // N
    debounce_n_ = declare_parameter<int>("debounce_n", 3);
    v0_ = declare_parameter<double>("v0", 0.025); // m/s
    vmin_ = declare_parameter<double>("vmin", 0.005); // m/s
    tau_ = declare_parameter<double>("tau", 4.0); // s
    timeout_s_ = declare_parameter<double>("approach_timeout_s", 30.0);

    sub_conv_ = create_subscription<std_msgs::msg::Bool>(
      "/pbvs/converged", 10,
      [&](std_msgs::msg::Bool::SharedPtr msg){ pbvs_converged_ = msg->data; });
/*
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      "/haptics/enable", 10,
      [&](std_msgs::msg::Bool::SharedPtr msg){ haptics_enabled_ = msg->data; });
*/
    sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/ur/tcp_wrench", rclcpp::SensorDataQoS(),
      std::bind(&OffsetNode::wrench_callback, this, std::placeholders::_1));

    pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/offset/cmd_twist_cf", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(8ms, std::bind(&OffsetNode::tick, this)); // ~125Hz

    RCLCPP_INFO(get_logger(), "Offset node ready.");
  }

private:
  enum class State { IDLE, APPROACH, DONE, FAULT };
  State state_{State::IDLE};
  // IO
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_conv_;
  //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
  // State
  bool pbvs_converged_{false};
  bool haptics_enabled_{true}; // default true for teleop gating
  bool have_wrench_{false};
  bool printed_first_wrench_{false};
  geometry_msgs::msg::WrenchStamped last_wrench_;
  rclcpp::Time t_start_;
  int contact_count_{0};
  rclcpp::Time last_wrench_rx_time_;
  // Params
  double f_contact_{10.0};
  int debounce_n_{3};
  double v0_{0.02}, vmin_{0.002}, tau_{1.0}, timeout_s_{10.0};

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (!msg) return;
    if (!printed_first_wrench_)
    {
      printed_first_wrench_ = true;
      RCLCPP_INFO(get_logger(),
      "First wrench received on /ur/tcp_wrench. frame_id='%s'",
      msg->header.frame_id.c_str());
    }
    last_wrench_ = *msg;
    last_wrench_rx_time_ = this->now();
    have_wrench_ = true;
  }

  void tick()
  {
    const auto now_t = now();

    switch (state_) 
    {
      case State::IDLE: 
      {
        publishZero(now_t);
        if (pbvs_converged_ && haptics_enabled_) 
        {
          state_ = State::APPROACH;
          t_start_ = now_t;
          contact_count_ = 0;
          RCLCPP_INFO(get_logger(), "Entering APPROACH.");
        }
        return;
      }
      case State::APPROACH: 
      {
        // Give a short grace period right after entering APPROACH
        if (!have_wrench_) 
        {
          if ((now_t - t_start_).seconds() < 0.5)
          {
            publishZero(now_t);
            return;
          }
          fault("No wrench received.");
          return;
        }

        // Detect stale wrench stream
        const double wrench_age_s = (now_t - last_wrench_rx_time_).seconds();
        if (wrench_age_s > 0.25) 
        {
          fault("Wrench stream stale (>0.25s).");
          return;
        }
        const double t = (now_t - t_start_).seconds();
        if (t > timeout_s_) 
        {
          fault("Approach timeout.");
          return;
        }
        // Contact detection (magnitude)
        const auto & f = last_wrench_.wrench.force;
        const double fmag = std::sqrt(f.x*f.x + f.y*f.y + f.z*f.z);
        if (fmag > f_contact_) 
        {
          contact_count_++;
        } 
        else 
        {
          contact_count_ = 0;
        }

        if (contact_count_ >= debounce_n_) 
        {
          publishZero(now_t);
          state_ = State::DONE;
          RCLCPP_INFO(get_logger(), "Contact detected. DONE.");
          return;
        }

        // Decaying approach velocity along camera-frame +Z
        const double v = std::max(vmin_, v0_ * std::exp(-t / tau_));
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = now_t;
        cmd.header.frame_id = "tooling_frame";
        cmd.twist.linear.z = v;
        pub_twist_->publish(cmd);
        return;
      }
      case State::DONE:
      case State::FAULT: 
      {
        publishZero(now_t);
        return;
      }
    }
  }


  void publishZero(const rclcpp::Time& stamp) 
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = "tooling_frame";
    pub_twist_->publish(cmd);
  }
  void fault(const std::string& why) 
  {
    RCLCPP_ERROR(get_logger(), "FAULT: %s", why.c_str());
    state_ = State::FAULT;
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffsetNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}