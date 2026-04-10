#include <rclcpp/rclcpp.hpp> // Include the ROS node api
#include <sensor_msgs/msg/image.hpp> // Include the image message type
#include <ament_index_cpp/get_package_share_directory.hpp> // Include package share path lookup
#include <opencv2/core/persistence.hpp> // Include yaml file storage support

#include <algorithm> // Include max helper functions
#include <chrono> // Include timer duration support
#include <filesystem> // Include filesystem path support
#include <functional> // Include function binding support
#include <string> // Include string support
#include <unordered_set> // Include set support for unique paths
#include <vector> // Include vector support

namespace fs = std::filesystem; // Create a short filesystem alias

class TemplateCameraNode : public rclcpp::Node { // Define the template camera node
public:
  explicit TemplateCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("template_camera_node", options) { // Initialize the node with the original interface name
    width_ = this->declare_parameter<int>("width", 1920); // Store the requested image width
    height_ = this->declare_parameter<int>("height", 1080); // Store the requested image height
    fps_ = this->declare_parameter<int>("fps", 30); // Store the requested publish rate
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_link"); // Store the frame id
    qos_depth_ = this->declare_parameter<int>("qos_depth", 1); // Store the qos queue depth
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort"); // Store the qos reliability mode
    encoding_ = this->declare_parameter<std::string>("encoding", "bgr8"); // Store the image encoding label
    serial_ = this->declare_parameter<std::string>("serial", ""); // Store the optional serial value
    active_camera_filename_ =
      this->declare_parameter<std::string>("active_camera_filename", "active_camera.yaml"); // Store the active camera file name
    config_subdir_ = this->declare_parameter<std::string>("config_subdir", "config"); // Store the config subdirectory name
    mirror_packages_ = this->declare_parameter<std::vector<std::string>>(
      "mirror_packages", std::vector<std::string>{"dev_cpp", "dev_py"}); // Store the package mirror list
    const int depth = std::max(1, qos_depth_); // Clamp the qos depth to a valid value
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth))); // Build the image qos profile
    qos.durability_volatile(); // Force volatile durability for streaming data
    if (qos_reliability_ == "reliable") { // Select reliable transport when requested
      qos.reliable(); // Apply reliable delivery
    } else { // Use the streaming default otherwise
      qos.best_effort(); // Apply best effort delivery
    }
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", qos); // Create the image publisher
    resolved_serial_ = resolve_template_serial(); // Resolve a serial value without hardware access
    write_active_camera_yaml(resolved_serial_, width_, height_); // Update the active camera yaml in the dev workspace
    const auto period = std::chrono::milliseconds(std::max(1, 1000 / std::max(1, fps_))); // Convert the frame rate into a timer period
    timer_ = this->create_wall_timer(period, std::bind(&TemplateCameraNode::publish_blank_image, this)); // Create the periodic publish timer
    RCLCPP_INFO(
      this->get_logger(),
      "template camera node started serial=%s width=%d height=%d fps=%d",
      resolved_serial_.c_str(),
      width_,
      height_,
      fps_); // Report the template startup state
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_; // Store the image publisher handle
  rclcpp::TimerBase::SharedPtr timer_; // Store the periodic timer handle
  int width_{1920}; // Store the image width parameter
  int height_{1080}; // Store the image height parameter
  int fps_{30}; // Store the publish rate parameter
  std::string frame_id_{"camera_link"}; // Store the image frame id
  int qos_depth_{1}; // Store the qos depth parameter
  std::string qos_reliability_{"best_effort"}; // Store the qos reliability parameter
  std::string encoding_{"bgr8"}; // Store the image encoding parameter
  std::string serial_; // Store the requested serial string
  std::string resolved_serial_; // Store the resolved template serial string
  std::string active_camera_filename_{"active_camera.yaml"}; // Store the active camera file name
  std::string config_subdir_{"config"}; // Store the config subdirectory name
  std::vector<std::string> mirror_packages_; // Store the mirrored package names

  static std::string infer_workspace_root_from_share(const std::string& share_dir) { // Derive the workspace root from a package share path
    const std::string needle = "/install/"; // Match the install path segment
    const auto pos = share_dir.find(needle); // Find the install segment position
    if (pos == std::string::npos) { // Handle paths that do not contain an install segment
      return ""; // Return an empty root when the workspace cannot be inferred
    }
    return share_dir.substr(0, pos); // Return the portion before the install segment
  }

  static std::vector<std::string> unique_dirs(const std::vector<std::string>& input) { // Remove duplicate directory strings
    std::unordered_set<std::string> seen; // Track directories that were already recorded
    std::vector<std::string> output; // Store the unique directory list
    output.reserve(input.size()); // Reserve space for the unique directory list
    for (const auto& item : input) { // Visit each candidate directory
      if (item.empty()) { // Ignore empty directory strings
        continue; // Move to the next candidate directory
      }
      if (seen.insert(item).second) { // Keep the directory only once
        output.push_back(item); // Append the unseen directory
      }
    }
    return output; // Return the deduplicated directory list
  }

  std::vector<std::string> compute_active_camera_target_dirs() const { // Build the dev workspace target directories
    std::vector<std::string> dirs; // Store the candidate directory list
    std::string ws_root; // Store the inferred workspace root
    try { // Attempt to resolve the dev cpp share path
      const std::string share = ament_index_cpp::get_package_share_directory("dev_cpp"); // Load the dev cpp share path
      ws_root = infer_workspace_root_from_share(share); // Infer the workspace root from the share path
      dirs.push_back(share + "/" + config_subdir_); // Add the dev cpp runtime config directory
    } catch (...) { // Ignore lookup failures for the current package
    }
    for (const auto& pkg : mirror_packages_) { // Visit each mirrored package name
      if (pkg == "dev_cpp") { // Skip the package that was already added
        continue; // Move to the next package name
      }
      try { // Attempt to resolve the mirrored package share path
        const std::string share = ament_index_cpp::get_package_share_directory(pkg); // Load the mirrored package share path
        dirs.push_back(share + "/" + config_subdir_); // Add the mirrored runtime config directory
      } catch (...) { // Ignore lookup failures for optional mirrored packages
      }
    }
    if (!ws_root.empty()) { // Add workspace visible paths when the root was found
      dirs.push_back(ws_root + "/config"); // Add the workspace level config directory
      for (const auto& pkg : mirror_packages_) { // Visit each mirrored package name
        dirs.push_back(ws_root + "/src/" + pkg + "/" + config_subdir_); // Add the source config directory
        dirs.push_back(
          ws_root + "/install/" + pkg + "/share/" + pkg + "/" + config_subdir_); // Add the installed config directory
      }
    }
    return unique_dirs(dirs); // Return the deduplicated target directories
  }

  std::string resolve_template_serial() const { // Resolve a serial string without touching hardware
    if (!serial_.empty()) { // Prefer the requested serial when one was provided
      return serial_; // Return the configured serial string
    }
    return "template_camera"; // Return the fallback template serial name
  }

  bool write_active_camera_yaml(const std::string& serial, int width, int height) { // Write the active camera yaml file
    const auto dirs = compute_active_camera_target_dirs(); // Collect the candidate output directories
    if (dirs.empty()) { // Handle the case where no output path was found
      RCLCPP_WARN(this->get_logger(), "no dev workspace config directory found for active camera yaml"); // Report the missing path state
      return false; // Report that the write could not be completed
    }
    bool any_ok = false; // Track whether any write attempt succeeds
    for (const auto& dir : dirs) { // Visit each target directory
      try { // Attempt to create the directory and write the yaml file
        fs::create_directories(dir); // Create the output directory tree
        const std::string path = dir + "/" + active_camera_filename_; // Build the output yaml path
        cv::FileStorage fsw(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML); // Open the yaml file for writing
        if (!fsw.isOpened()) { // Skip this path when the file could not be opened
          continue; // Move to the next target directory
        }
        fsw << "serial" << serial; // Write the serial field
        fsw << "width" << width; // Write the width field
        fsw << "height" << height; // Write the height field
        fsw.release(); // Close the yaml file
        any_ok = true; // Record that a write succeeded
      } catch (const std::exception&) { // Ignore standard exceptions during the write path
      } catch (...) { // Ignore unknown exceptions during the write path
      }
    }
    return any_ok; // Return the overall write result
  }

  void publish_blank_image() { // Publish a blank image that preserves the camera interface

















  ///////////////////////////////////////////////////////
  /*
  Main development zone.
  Use camera API to grab images from the camera you're using
  */
  ///////////////////////////////////////////////////////

















    sensor_msgs::msg::Image msg; // Create the outgoing image message
    msg.header.stamp = this->now(); // Stamp the image with the current time
    msg.header.frame_id = frame_id_; // Assign the configured frame id
    msg.width = static_cast<uint32_t>(width_); // Assign the requested image width
    msg.height = static_cast<uint32_t>(height_); // Assign the requested image height
    msg.encoding = encoding_; // Assign the configured encoding label
    msg.is_bigendian = false; // Use little endian image storage
    const uint32_t bytes_per_pixel = 3; // Use three bytes per pixel for the template image
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
      static_cast<uint32_t>(width_) * bytes_per_pixel); // Compute the row step in bytes
    msg.data.assign(static_cast<size_t>(height_) * static_cast<size_t>(msg.step), 0u); // Fill the image with zero bytes
    pub_->publish(msg); // Publish the blank image message
  }
};

int main(int argc, char** argv) { // Start the ROS process for the template camera node
  rclcpp::init(argc, argv); // Initialize ROS
  rclcpp::NodeOptions options; // Create the node options object
  options.use_intra_process_comms(true); // Enable intra process transport
  auto node = std::make_shared<TemplateCameraNode>(options); // Create the template camera node
  rclcpp::spin(node); // Spin the node callbacks
  rclcpp::shutdown(); // Shut down ROS
  return 0; // Return a successful process code
}
