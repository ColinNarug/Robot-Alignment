#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>
#include <algorithm>
#include <ur_rtde/rtde_receive_interface.h> // from ur_rtde
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

using namespace std::chrono_literals;

class OffsetNode : public rclcpp::Node
{
public:
  OffsetNode() : Node("offset_node")
  {
    // Parameters
    f_contact_        = declare_parameter<double>("f_contact", 5.0); // N 
    debounce_n_       = declare_parameter<int>("debounce_n", 6); // threshold
    v0_               = declare_parameter<double>("v0", 0.003); // m/s
    vmin_             = declare_parameter<double>("vmin", 0.001); // m/s
    tau_              = declare_parameter<double>("tau", 1.0); // s
    timeout_s_        = declare_parameter<double>("approach_timeout_s", 180.0); // s
    contact_grace_s_  = declare_parameter<double>("contact_grace_s", 0.25); // s
    baseline_window_s_ = declare_parameter<double>("baseline_window_s", 0.25); // s

    //Callback Groups:
    wrench_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // wrench only
    offset_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // tick & key
    rclcpp::SubscriptionOptions offset_opts;
    offset_opts.callback_group = offset_handling_;

    constexpr const char* ROBOT_IP = "192.168.1.101";
    try 
    {
      rtde_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(ROBOT_IP, 125.0);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(this->get_logger(), "RTDE connection failed to %s: %s", ROBOT_IP, e.what());
      throw;
    }

    // Convergence Bool Subscription:
    sub_conv_ = create_subscription<std_msgs::msg::Bool>(
      "/pbvs/converged", 10,
      [&](std_msgs::msg::Bool::SharedPtr msg) 
      {
        if (!msg) return;
        pbvs_converged_ = msg->data;
      }, offset_opts);

    // Key subscription (latched)
    rclcpp::QoS key_qos(1);
    key_qos.reliable().transient_local();
    sub_key_ = create_subscription<std_msgs::msg::Char>(
      "/teleop/last_key", key_qos,
      std::bind(&OffsetNode::key_callback, this, std::placeholders::_1), offset_opts);

    // Publish Force/Torque from RTDE
    pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("/offset/cmd_twist", rclcpp::SensorDataQoS());

    // Create timers for callback groups
    wrench_timer_ = this->create_wall_timer(8ms, std::bind(&OffsetNode::wrench_callback, this), wrench_handling_); // ~125Hz
    offset_timer_ = this->create_wall_timer(8ms, std::bind(&OffsetNode::tick, this), offset_handling_); // ~125Hz

    RCLCPP_INFO(this->get_logger(), "wrench started. RTDE (robot_ip=%s)", ROBOT_IP);
    RCLCPP_INFO(get_logger(), "Offset node ready. Press X to enable approach; Space to disable.");
  }

private:
  enum class State { IDLE, BASELINE, APPROACH, DONE, FAULT };
  State state_{State::IDLE};

  rclcpp::CallbackGroup::SharedPtr wrench_handling_, offset_handling_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_conv_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr sub_key_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::TimerBase::SharedPtr wrench_timer_, offset_timer_;
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_;
  mutable std::mutex wrench_mtx_;

  bool pbvs_converged_{false};
  bool haptics_enabled_{false}; // require X to run
  bool have_wrench_{false};
  bool printed_first_wrench_{false};
  geometry_msgs::msg::WrenchStamped last_wrench_;
  rclcpp::Time last_wrench_rx_time_;
  rclcpp::Time t_start_; // APPROACH start time
  rclcpp::Time t_baseline_; // BASELINE start time
  int contact_count_{0};
  // Baseline accumulation (stationary):
  bool baseline_valid_{false};
  int baseline_n_{0};
  double baseline_sum_fx_{0.0}, baseline_sum_fy_{0.0}, baseline_sum_fz_{0.0}, baseline_sum_fmag_{0.0};
  double baseline_fx_{0.0}, baseline_fy_{0.0}, baseline_fz_{0.0}, baseline_fmag_{0.0};

  // Params
  double f_contact_{5.0};
  int debounce_n_{6};
  double v0_{0.003}, vmin_{0.001}, tau_{1.0}, timeout_s_{30.0};
  double contact_grace_s_{0.25};
  double baseline_window_s_{0.25};

  void wrench_callback() // Get F/T from RTDE and store it
  {
    // Returns [Fx, Fy, Fz, Tx, Ty, Tz]
    const std::vector<double> w = rtde_->getActualTCPForce();
    if (w.size() != 6)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "RTDE wrench vector size != 6 (actual: %zu)", w.size());
      return;
    }

    std::string frame_id_copy;
    {
      std::lock_guard<std::mutex> lk(wrench_mtx_);

      // Receipt timestamp
      const rclcpp::Time stamp = this->get_clock()->now();
      last_wrench_rx_time_ = stamp;

      // Header stamp
      const int64_t ns = stamp.nanoseconds();
      last_wrench_.header.stamp.sec     = static_cast<int32_t>(ns / 1000000000LL);
      last_wrench_.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

      // Identify the frame (UR reports wrench at TCP; conventionally treat as base-frame)
      last_wrench_.header.frame_id = "base";

      // Store force/torque values
      last_wrench_.wrench.force.x  = w[0];
      last_wrench_.wrench.force.y  = w[1];
      last_wrench_.wrench.force.z  = w[2];
      last_wrench_.wrench.torque.x = w[3];
      last_wrench_.wrench.torque.y = w[4];
      last_wrench_.wrench.torque.z = w[5];

      have_wrench_ = true;
      frame_id_copy = last_wrench_.header.frame_id;
    }

    if (!printed_first_wrench_)
    {
      printed_first_wrench_ = true;
      RCLCPP_INFO(this->get_logger(),
        "First RTDE wrench received. frame_id='%s'", frame_id_copy.c_str());
    }
  }

  void key_callback(const std_msgs::msg::Char::SharedPtr msg) // Check the TUI commands
  {
    if (!msg) return;
    const char k = static_cast<char>(msg->data);

    if (k == 'x' || k == 'X')
    {
      haptics_enabled_ = true;

      // Allow restart after DONE/FAULT
      if (state_ == State::DONE || state_ == State::FAULT) {
        state_ = State::IDLE;
      }

      RCLCPP_INFO(get_logger(), "Key=X/x -> ENABLE.");
    }
    else if (k == ' ')
    {
      haptics_enabled_ = false;
      state_ = State::IDLE;
      baseline_valid_ = false;
      contact_count_ = 0;
      publishZero(now());
      RCLCPP_WARN(get_logger(), "Key=Space -> DISABLE. Forcing IDLE + zero twist.");
    }
    else if (k == 'q' || k == 'Q' || k == 0x1B)
    {
      haptics_enabled_ = false;
      state_ = State::IDLE;
      baseline_valid_ = false;
      contact_count_ = 0;
      publishZero(now());
      RCLCPP_WARN(get_logger(), "Key=Esc/Q/q -> DISABLE. Forcing IDLE + zero twist.");
    }
  }

  // Run node states and publish velocity commands for robot node
  void tick()
  {
    const auto now_t = now();
    geometry_msgs::msg::WrenchStamped wrench;
    rclcpp::Time wrench_time;
    bool have_wrench_snap = false;

    {
      std::lock_guard<std::mutex> lk(wrench_mtx_);
      wrench = last_wrench_;
      wrench_time = last_wrench_rx_time_;
      have_wrench_snap = have_wrench_;
    }

    // Hard gate: if disabled, always publish zero and stay IDLE
    if (!haptics_enabled_) {
      publishZero(now_t);
      state_ = State::IDLE;
      return;
    }

    switch (state_)
    {
      case State::IDLE:
      {
        publishZero(now_t);

        // Proceed once PBVS converged and have wrench stream
        if (pbvs_converged_) {
          if (!have_wrench_snap) {
            return;
          }

          const double wrench_age_s = (now_t - wrench_time).seconds();
          if (wrench_age_s > 0.1) {
            fault("Wrench stream stale in IDLE (>0.1s).");
            return;
          }
          enterBaseline(now_t);
        }
        return;
      }

      case State::BASELINE:
      {
        // Keep still while measuring baseline
        publishZero(now_t);

        // If PBVS un-converges, drop back to IDLE (avoid using a stale baseline)
        if (!pbvs_converged_) {
          baseline_valid_ = false;
          state_ = State::IDLE;
          RCLCPP_WARN(get_logger(), "PBVS unconverged during BASELINE -> returning to IDLE.");
          return;
        }

        if (!have_wrench_snap) {
          fault("No wrench received during BASELINE.");
          return;
        }

        const double wrench_age_s = (now_t - wrench_time).seconds();
        if (wrench_age_s > 0.25) {
          fault("Wrench stream stale during BASELINE (>0.25s).");
          return;
        }

        const auto & f = wrench.wrench.force;
        const double fx = f.x;
        const double fy = f.y;
        const double fz = f.z;
        const double fmag = std::sqrt(fx*fx + fy*fy + fz*fz);

        baseline_sum_fx_ += fx;
        baseline_sum_fy_ += fy;
        baseline_sum_fz_ += fz;
        baseline_sum_fmag_ += fmag;
        baseline_n_++;

        const double t_bl = (now_t - t_baseline_).seconds();
        if (t_bl >= baseline_window_s_ && baseline_n_ > 0)
        {
          // Compute mean baseline
          baseline_fx_   = baseline_sum_fx_   / baseline_n_;
          baseline_fy_   = baseline_sum_fy_   / baseline_n_;
          baseline_fz_   = baseline_sum_fz_   / baseline_n_;
          baseline_fmag_ = baseline_sum_fmag_ / baseline_n_;
          baseline_valid_ = true;

          RCLCPP_INFO(get_logger(),
            "Baseline captured over %.3fs (%d samples): "
            "fx=%.3f fy=%.3f fz=%.3f |F|=%.3f. "
            "Contact threshold uses baseline + f_contact (%.3f).",
            t_bl, baseline_n_, baseline_fx_, baseline_fy_, baseline_fz_, baseline_fmag_, f_contact_);

          // Now enter APPROACH
          state_ = State::APPROACH;
          t_start_ = now_t;
          contact_count_ = 0;

          RCLCPP_INFO(get_logger(), "Entering APPROACH.");
        }

        return;
      }

      case State::APPROACH:
      {
        if (!pbvs_converged_) {
          // If PBVS drops out, stop and go idle (avoid pushing based on stale convergence)
          publishZero(now_t);
          baseline_valid_ = false;
          state_ = State::IDLE;
          RCLCPP_WARN(get_logger(), "PBVS unconverged during APPROACH -> returning to IDLE.");
          return;
        }

        if (!have_wrench_snap) {
          if ((now_t - t_start_).seconds() < 0.5) {
            publishZero(now_t);
            return;
            
          }
          fault("No wrench received.");
          return;
        }

        const double wrench_age_s = (now_t - wrench_time).seconds();
        if (wrench_age_s > 0.25) 
        {
          fault("Wrench stream stale (>0.25s).");
          return;
        }

        const double t = (now_t - t_start_).seconds();

        // Smooth ramp-up from 0 -> v0
        const double v_target = v0_;
        double v = 0.0;
        if (tau_ > 1e-6) {
          v = v_target * (1.0 - std::exp(-t / tau_));
        } else {
          v = v_target;
        }
        v = std::clamp(v, 0.0, v_target);

        // Ignore contact to avoid start transient false stops
        if (t < contact_grace_s_) {
          contact_count_ = 0;
        } else {
          const auto & f = wrench.wrench.force;
          const double fx = f.x;
          const double fy = f.y;
          const double fz = f.z;
          const double fmag = std::sqrt(fx*fx + fy*fy + fz*fz);

          const double base = baseline_valid_ ? baseline_fmag_ : 0.0;
          const double thresh = base + f_contact_;

          if (fmag > thresh) {
            contact_count_++;
          } else {
            contact_count_ = 0;
          }

          if (contact_count_ >= debounce_n_) {
            publishZero(now_t);
            state_ = State::DONE;
            RCLCPP_INFO(get_logger(),
              "Contact detected. DONE. |F|=%.3f > (baseline %.3f + f_contact %.3f) = %.3f",
              fmag, base, f_contact_, thresh);
            return;
          }
        }

        // Publish approach twist along tooling-frame +Z
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

  void enterBaseline(const rclcpp::Time& now_t)
  {
    // Reset baseline accumulation
    baseline_valid_ = false;
    baseline_n_ = 0;
    baseline_sum_fx_ = baseline_sum_fy_ = baseline_sum_fz_ = baseline_sum_fmag_ = 0.0;
    baseline_fx_ = baseline_fy_ = baseline_fz_ = baseline_fmag_ = 0.0;

    t_baseline_ = now_t;
    state_ = State::BASELINE;

    RCLCPP_INFO(get_logger(),
      "Entering BASELINE (stationary %.3fs) to measure wrench bias at APPROACH pose...",
      baseline_window_s_);
  }

  void publishZero(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = "tooling_frame";
    // all twist fields default to 0
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
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
