#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    // Subscribes to the same topic published by d435i_camera.cpp:
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/color/image_raw", rclcpp::SensorDataQoS(), // topic name and QoS queue size
      std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      RCLCPP_INFO(this->get_logger(), "Image callback triggered! Image resolution: %d x %d", msg->width, msg->height);
      
      // Convert ROS2 image message to OpenCV format:
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;

      // Resize image for display:
      cv::Mat displayed_image;
      double scale_factor = 1.0;
      cv::resize(image, displayed_image, cv::Size(), scale_factor, scale_factor);

      cv::imshow("Live Camera Feed", displayed_image); // Show image in window

      // WaitKey needed for GUI thread (ESC = 27):
      if (cv::waitKey(1) == 27) {
        RCLCPP_INFO(this->get_logger(), "ESC pressed. Shutting down.");
        rclcpp::shutdown();
      }
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; // Declare shared pointer to subscription
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}