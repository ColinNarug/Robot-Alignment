#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ur_rtde/rtde_receive_interface.h>  // from ur_rtde
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class RtdeWrenchPublisher : public rclcpp::Node
{
public:
  RtdeWrenchPublisher()
  : Node("wrench")
  {
    constexpr const char* ROBOT_IP = "192.168.1.101";

    pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/ur/tcp_wrench", rclcpp::SensorDataQoS());

    try 
    {
      rtde_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(ROBOT_IP, 125.0);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(this->get_logger(), "RTDE connection failed to %s: %s", ROBOT_IP, e.what());
      throw;
    }
    timer_ = this->create_wall_timer(8ms, std::bind(&RtdeWrenchPublisher::tick, this));
    RCLCPP_INFO(this->get_logger(),
      "wrench node started. Publishing /ur/tcp_wrench from RTDE (robot_ip=%s)",
      ROBOT_IP);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_;

  void tick()
  {
    // Returns [Fx, Fy, Fz, Tx, Ty, Tz]
    const std::vector<double> w = rtde_->getActualTCPForce();
    if (w.size() != 6)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "RTDE wrench vector size is not equal to 6 (actual: %zu)", w.size());
      return;
    }

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";  // UR reports in base frame (common convention)
    msg.wrench.force.x  = w[0];
    msg.wrench.force.y  = w[1];
    msg.wrench.force.z  = w[2];
    msg.wrench.torque.x = w[3];
    msg.wrench.torque.y = w[4];
    msg.wrench.torque.z = w[5];
    pub_->publish(msg);
  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtdeWrenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
