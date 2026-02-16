// L1
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
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
    // -----------------------------
    // Parameters
    // -----------------------------
    f_contact_        = declare_parameter<double>("f_contact", 10.0);          // N (CONTACT delta threshold above baseline)
    debounce_n_       = declare_parameter<int>("debounce_n", 25);              // consecutive samples over threshold
    v0_               = declare_parameter<double>("v0", 0.010);                // m/s (BASE / TARGET approach speed; ramps up to this)
    vmin_             = declare_parameter<double>("vmin", 0.005);              // m/s (kept for compatibility; not used in ramp mode)
    tau_              = declare_parameter<double>("tau", 1.0);                 // s   (ramp-up time constant)
    timeout_s_        = declare_parameter<double>("approach_timeout_s", 30.0); // s

    // NEW: ignore contact detection briefly after APPROACH begins (handles transient)
    contact_grace_s_  = declare_parameter<double>("contact_grace_s", 0.25);    // s

    // NEW: measure baseline wrench while stationary between PBVS converge and APPROACH
    baseline_window_s_ = declare_parameter<double>("baseline_window_s", 0.25); // s (stationary averaging window)

    // -----------------------------
    // Subscriptions / Publications
    // -----------------------------
    sub_conv_ = create_subscription<std_msgs::msg::Bool>(
      "/pbvs/converged", 10,
      [&](std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) return;
        pbvs_converged_ = msg->data;
      });

    // Key subscription (latched) for enable/disable during APPROACH
    rclcpp::QoS key_qos(1);
    key_qos.reliable().transient_local();
    sub_key_ = create_subscription<std_msgs::msg::Char>(
      "/teleop/last_key", key_qos,
      std::bind(&OffsetNode::key_callback, this, std::placeholders::_1));

    sub_wrench_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/ur/tcp_wrench", rclcpp::SensorDataQoS(),
      std::bind(&OffsetNode::wrench_callback, this, std::placeholders::_1));

    pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/offset/cmd_twist_cf", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(8ms, std::bind(&OffsetNode::tick, this)); // ~125Hz

    RCLCPP_INFO(get_logger(), "Offset node ready. Press X to enable approach; Space to disable.");
  }

private:
  enum class State { IDLE, BASELINE, APPROACH, DONE, FAULT };
  State state_{State::IDLE};

  // IO
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_conv_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr sub_key_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool pbvs_converged_{false};

  // Teleop gating
  bool haptics_enabled_{false}; // require X to run

  // Wrench stream
  bool have_wrench_{false};
  bool printed_first_wrench_{false};
  geometry_msgs::msg::WrenchStamped last_wrench_;
  rclcpp::Time last_wrench_rx_time_;

  // Timing / counters
  rclcpp::Time t_start_;       // APPROACH start time
  rclcpp::Time t_baseline_;    // BASELINE start time
  int contact_count_{0};

  // Baseline accumulation (stationary)
  bool baseline_valid_{false};
  int baseline_n_{0};
  double baseline_sum_fx_{0.0}, baseline_sum_fy_{0.0}, baseline_sum_fz_{0.0}, baseline_sum_fmag_{0.0};
  double baseline_fx_{0.0}, baseline_fy_{0.0}, baseline_fz_{0.0}, baseline_fmag_{0.0};

  // Params
  double f_contact_{10.0};
  int debounce_n_{25};
  double v0_{0.010}, vmin_{0.005}, tau_{1.0}, timeout_s_{30.0};
  double contact_grace_s_{0.25};
  double baseline_window_s_{0.25};

  // -----------------------------
  // Callbacks
  // -----------------------------
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (!msg) return;

    if (!printed_first_wrench_) {
      printed_first_wrench_ = true;
      RCLCPP_INFO(get_logger(),
        "First wrench received on /ur/tcp_wrench. frame_id='%s'",
        msg->header.frame_id.c_str());
    }

    last_wrench_ = *msg;
    last_wrench_rx_time_ = now();
    have_wrench_ = true;
  }

  void key_callback(const std_msgs::msg::Char::SharedPtr msg)
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

  // -----------------------------
  // Core loop
  // -----------------------------
  void tick()
  {
    const auto now_t = now();

    // Hard gate: if disabled, always publish zero and stay IDLE.
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

        // Only proceed once PBVS says we're converged and we have a live wrench stream
        if (pbvs_converged_) {
          if (!have_wrench_) {
            // wait quietly; do not fault here
            return;
          }

          const double wrench_age_s = (now_t - last_wrench_rx_time_).seconds();
          if (wrench_age_s > 0.25) {
            fault("Wrench stream stale in IDLE (>0.25s).");
            return;
          }

          // NEW: enter BASELINE to measure stationary wrench at the exact APPROACH orientation
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

        if (!have_wrench_) {
          fault("No wrench received during BASELINE.");
          return;
        }

        const double wrench_age_s = (now_t - last_wrench_rx_time_).seconds();
        if (wrench_age_s > 0.25) {
          fault("Wrench stream stale during BASELINE (>0.25s).");
          return;
        }

        // Accumulate baseline samples
        const auto & f = last_wrench_.wrench.force;
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

        if (!have_wrench_) {
          // small grace at very beginning in case stream arrives slightly late
          if ((now_t - t_start_).seconds() < 0.5) {
            publishZero(now_t);
            return;
          }
          fault("No wrench received.");
          return;
        }

        // Stale wrench stream check
        const double wrench_age_s = (now_t - last_wrench_rx_time_).seconds();
        if (wrench_age_s > 0.25) {
          fault("Wrench stream stale (>0.25s).");
          return;
        }

        // Timeout check
        const double t = (now_t - t_start_).seconds();
        if (t > timeout_s_) {
          fault("Approach timeout.");
          return;
        }

        // Smooth ramp-up from 0 -> v0_ (base speed). No sharp upward edge; no high initial velocity.
        const double v_target = v0_;
        double v = 0.0;
        if (tau_ > 1e-6) {
          v = v_target * (1.0 - std::exp(-t / tau_));
        } else {
          v = v_target;
        }
        v = std::clamp(v, 0.0, v_target);

        // Contact detection:
        // - Uses SAME metric as before (force magnitude) but with a measured baseline added.
        // - During contact_grace_s_, we ignore contact to avoid start transient false stops.
        if (t < contact_grace_s_) {
          contact_count_ = 0;
        } else {
          const auto & f = last_wrench_.wrench.force;
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

  // -----------------------------
  // Helpers
  // -----------------------------
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
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
