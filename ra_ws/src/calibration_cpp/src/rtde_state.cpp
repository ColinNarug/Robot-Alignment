#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ur_rtde/rtde_receive_interface.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class RtdeStateNode : public rclcpp::Node
{
public:
  RtdeStateNode() : Node("rtde_state")
  {
    robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.1.101");
    rate_hz_  = this->declare_parameter<double>("rate_hz", 125.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "inertial");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ur/tcp_pose", rclcpp::SensorDataQoS());

    try 
    {
      rtde_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_, rate_hz_);
    } 
    catch (const std::exception& e) 
    {
      RCLCPP_FATAL(this->get_logger(), "RTDEReceive connect failed to %s: %s", robot_ip_.c_str(), e.what());
      throw;
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RtdeStateNode::tick, this));

    RCLCPP_INFO(this->get_logger(), "rtde_state running (robot_ip=%s rate=%.1fHz)", robot_ip_.c_str(), rate_hz_);
  }

private:
  std::string robot_ip_;
  double rate_hz_{125.0};
  std::string frame_id_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_;

  static geometry_msgs::msg::Quaternion rvec_to_quat(double rx, double ry, double rz)
  {
    cv::Mat rvec(3,1,CV_64F);
    rvec.at<double>(0)=rx; rvec.at<double>(1)=ry; rvec.at<double>(2)=rz;
    cv::Mat R(3,3,CV_64F);
    cv::Rodrigues(rvec, R);

    // Convert rotation matrix to quaternion (w,x,y,z)
    const double tr = R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2);
    geometry_msgs::msg::Quaternion q{};
    if (tr > 0.0) 
    {
      double S = std::sqrt(tr + 1.0) * 2.0;
      q.w = 0.25 * S;
      q.x = (R.at<double>(2,1) - R.at<double>(1,2)) / S;
      q.y = (R.at<double>(0,2) - R.at<double>(2,0)) / S;
      q.z = (R.at<double>(1,0) - R.at<double>(0,1)) / S;
    } 
    else 
    {
      // Fallback branches
      if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) 
      {
        double S = std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2)) * 2.0;
        q.w = (R.at<double>(2,1) - R.at<double>(1,2)) / S;
        q.x = 0.25 * S;
        q.y = (R.at<double>(0,1) + R.at<double>(1,0)) / S;
        q.z = (R.at<double>(0,2) + R.at<double>(2,0)) / S;
      } 
      else if (R.at<double>(1,1) > R.at<double>(2,2)) 
      {
        double S = std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2)) * 2.0;
        q.w = (R.at<double>(0,2) - R.at<double>(2,0)) / S;
        q.x = (R.at<double>(0,1) + R.at<double>(1,0)) / S;
        q.y = 0.25 * S;
        q.z = (R.at<double>(1,2) + R.at<double>(2,1)) / S;
      } 
      else 
      {
        double S = std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1)) * 2.0;
        q.w = (R.at<double>(1,0) - R.at<double>(0,1)) / S;
        q.x = (R.at<double>(0,2) + R.at<double>(2,0)) / S;
        q.y = (R.at<double>(1,2) + R.at<double>(2,1)) / S;
        q.z = 0.25 * S;
      }
    }
    return q;
  }

  void tick()
  {
    // TCP pose: [x,y,z,rx,ry,rz] 
    const std::vector<double> p = rtde_->getActualTCPPose();
    if (p.size() == 6)
    {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.pose.position.x = p[0];
      msg.pose.position.y = p[1];
      msg.pose.position.z = p[2];
      msg.pose.orientation = rvec_to_quat(p[3], p[4], p[5]);
      pose_pub_->publish(msg);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtdeStateNode>());
  rclcpp::shutdown();
  return 0;
}