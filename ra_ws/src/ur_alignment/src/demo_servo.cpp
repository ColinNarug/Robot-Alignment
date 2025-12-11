#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpPose.h>
#include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include <string>                    // String Operations
#include "rclcpp/rclcpp.hpp"         // ROS2 CPP API
#include "std_msgs/msg/string.hpp"   // ROS2 String Message
#include "sensor_msgs/msg/image.hpp" // ROS2 Image Message
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>
#include <Eigen/Geometry>
#include <visp3/vision/vpPose.h>
#include <ament_index_cpp/get_package_share_directory.hpp> // File Paths for .yamls

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_UR_RTDE) && VISP_NAMESPACE_NAME
//! [Macro defined]
using namespace VISP_NAMESPACE_NAME;
#endif

// ROS 2 Node Class Definition:
class DemoServoNode : public rclcpp::Node
{
public:
  // Constructor
  DemoServoNode(int argc, char **argv)
      : Node("demo_servo")
  {
    RCLCPP_INFO(this->get_logger(), "DemoServoNode initialized.");

    // Subscribe to Camera node's topic. Image extraction by image_callback:
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&DemoServoNode::image_callback, this, std::placeholders::_1));
        

    for (int i = 1; i < argc; i++)
    {
      if (std::string(argv[i]) == "--example" && i + 1 < argc)
      { // TODO define and describe arguments (relative transformation, flange type)
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
      {
        std::cout << "Usage: " << argv[0]
                  << " [--example <brief explanation> (default: default_value)]";
      }
      std::cout << " [--verbose,-v] [--help,-h]" << std::endl;
      std::exit(EXIT_SUCCESS);
    }
  }

  // Main Loop | This is on its own thread:
  void run_servo_loop()
  {
    std::cout << "Top of run_servo_loop" << std::endl;
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11; //apriltags
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS; //apriltags
    double tagSize = 0.0221; //apriltags
    std::string robot_ip = "192.168.1.101"; //ur16e
    std::string package_path = ament_index_cpp::get_package_share_directory("uralignment_cpp"); //ur16e
    std::string eMc_filename = package_path + "/config/ur_eMc.yaml"; //ur16e
    std::string cdMo_filename = package_path + "/config/ur_cdMo.yaml";
    std::string camParam_filename = package_path + "/config/camera_intrinsics.yaml";
    float quad_decimate = 1.0; //ur16e
    int nThreads = 1; //ur16e
    bool display_tag = false; //ur16e
    int color_id = -1; //ur16e
    unsigned int thickness = 2; //ur16e
    bool align_frame = false; //ur16e
    bool opt_verbose = true;
    bool opt_plot = true; // displays
    bool opt_adaptive_gain = false; // ur16e
    bool opt_task_sequencing = false; // ur16e
    double convergence_threshold_t = 0.00001; // Value in [m] // ur16e
    double convergence_threshold_tu = 0.001;  // Value in [deg] // ur16e

#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)) //diplays
    bool display_off = true; //diplays
    std::cout << "Warning: There is no 3rd party(X11, GDI or openCV) to dislay images..." << std::endl; //diplays
#else
    bool display_off = false; //diplays
#endif

    // Create an instance of vpRobotUniversalRobots Class named 'robot'
    vpRobotUniversalRobots robot; //ur16e

    try
    {
      std::cout << "Top of 'try' in run_servo_loop" << std::endl; //ur16e
      //! [Robot connection] //ur16e
      std::cout << "Attempt to establish connection..." << std::endl; //ur16e
      robot.connect(robot_ip); //ur16e
      std::cout << "WARNING: This program will move the robot! " //ur16e
                << "Please make sure to have the user stop button at hand!" << std::endl //ur16e
                << "Press Enter to continue..." << std::endl; //ur16e
      std::cin.ignore(); //ur16e
      //! [Robot connection]

      // Get camera extrinsics:
      vpPoseVector ePc; //ur16e
      ePc.loadYAML(eMc_filename, ePc); //ur16e
      vpHomogeneousMatrix eMc(ePc); //ur16e
      std::cout << "Read extrinsic parameters from file. eMc:\n" //ur16e
                << eMc << "\n"; //ur16e

      // Camera Intrinsics:
      vpCameraParameters cam;  //diplays //apriltags
      cam.initPersProjWithoutDistortion(  //diplays //apriltags
          907.7258031, // px - Focal Length
          906.8582153, // py - Focal Length
          657.2998502, // u0 - Princicple Point
          358.9750977  // v0 - Principle Point
      );

      // Build Desired Object relative to Camera Fram [H.T. Matrix] from [Pose Vector]:
      vpPoseVector cdPo; //displays //ur_e
      cdPo.loadYAML(cdMo_filename, cdPo); //displays //ur_e
      vpHomogeneousMatrix cdMo(cdPo); //displays //ur_e

      std::cout << cam << std::endl;  //diplays //apriltags
      std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl; //apriltags
      std::cout << "tagFamily: " << tagFamily << std::endl; //apriltags
      std::cout << "nThreads: " << nThreads << std::endl; //apriltags
      std::cout << "Z aligned: " << align_frame << std::endl; //apriltags

      // Define ViSP image buffers matching RealSense camera resolution:
      vpImage<vpRGBa> I_color(540, 960); //diplays  //apriltags
      vpImage<unsigned char> I(540, 960); //diplays  //apriltags

      vpDisplay *d1 = nullptr;  //diplays
      if (!display_off)  //diplays
      {
#ifdef VISP_HAVE_X11  //diplays
        d1 = new vpDisplayX(I_color, 100, 30, "Pose from Homography");  //diplays
#endif  //diplays
      }

      //! [Create AprilTag detector]
      vpDetectorAprilTag detector(tagFamily); //diplays
      //! [Create AprilTag detector]

      //! [AprilTag detector settings]
      detector.setAprilTagQuadDecimate(quad_decimate); //diplays
      detector.setAprilTagPoseEstimationMethod(poseEstimationMethod); //diplays
      detector.setAprilTagNbThreads(nThreads); //diplays
      detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness); //diplays
      detector.setZAlignedWithCameraAxis(align_frame); //diplays
      //! [AprilTag detector settings]

      /* Servo
          cdMc: Desired Camera relative to current camera frame
          cMo: Object relative to camera frame
          oMo: Flip Logic
      */ 
      vpHomogeneousMatrix cdMc, cMo, oMo;  //diplays //ur_e
      std::cout << "Declared cdMc, cMo, oMo" << std::endl;  //diplays
      cdMc = cdMo * cMo.inverse();  //diplays //ur_e
      vpFeatureTranslation t(vpFeatureTranslation::cdMc);  //diplays
      vpFeatureThetaU tu(vpFeatureThetaU::cdRc);  //diplays
      t.buildFrom(cdMc);  //diplays //ur_e
      tu.buildFrom(cdMc);  //diplays //ur_e

      vpFeatureTranslation td(vpFeatureTranslation::cdMc);  //diplays
      vpFeatureThetaU tud(vpFeatureThetaU::cdRc);  //diplays

      vpServo task;  //diplays
      task.addFeature(t, td);  //diplays
      task.addFeature(tu, tud);  //diplays
      task.setServo(vpServo::EYEINHAND_CAMERA);  //diplays //ur
      task.setInteractionMatrixType(vpServo::CURRENT);  //diplays //ur

      // TODO tune this
      if (opt_adaptive_gain)  //ur16e
      {
        vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30 //ur16e
        task.setLambda(lambda); //ur16e
      }
      else //ur16e
      {
        task.setLambda(0.2); // TODO tune it //ur16e
      }

      vpPlot *plotter = nullptr;  //diplays
      int iter_plot = 0;  //diplays

      if (opt_plot)  //diplays
      {
        plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
        plotter->setTitle(0, "Visual features error");
        plotter->setTitle(1, "Camera velocities");
        plotter->initGraph(0, 6);
        plotter->initGraph(1, 6);
        plotter->setLegend(0, 0, "error_feat_tx");
        plotter->setLegend(0, 1, "error_feat_ty");
        plotter->setLegend(0, 2, "error_feat_tz");
        plotter->setLegend(0, 3, "error_feat_theta_ux");
        plotter->setLegend(0, 4, "error_feat_theta_uy");
        plotter->setLegend(0, 5, "error_feat_theta_uz");
        plotter->setLegend(1, 0, "vc_x");
        plotter->setLegend(1, 1, "vc_y");
        plotter->setLegend(1, 2, "vc_z");
        plotter->setLegend(1, 3, "wc_x");
        plotter->setLegend(1, 4, "wc_y");
        plotter->setLegend(1, 5, "wc_z");
      }
      std::cout << "After Plotter" << std::endl;
      bool final_quit = false; //ur16e
      bool has_converged = false; //ur16e
      bool send_velocities = false; //ur16e
      bool servo_started = false; //ur16e
      std::vector<vpImagePoint> *traj_vip = nullptr; // To memorize point trajectory // displays

      static double t_init_servo = vpTime::measureTimeMs(); // TODO benchmarking time as in detection //ur_e

      robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame //ur16e
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL); //ur16e

      std::cout << "Before While Loop" << std::endl;
      while (!has_converged && !final_quit)
      {
        std::cout << "Top of While Loop" << std::endl;
        double t_start = vpTime::measureTimeMs();

        // Wait for a new image:
        if (!new_image_available_) //apriltags
        {
          std::cout << "NO NEW IMAGE AVAILABLE!!!!!!!!!!!!!!!!!!!" << std::endl; //apriltags
          std::this_thread::sleep_for(std::chrono::milliseconds(1)); //apriltags
          continue; // No pass until new_image_available = true //apriltags
        }

        // Get latest image from ROS subscriber and convert to vpImage:
        cv::Mat current_image; //apriltags
        // Lock the mutex to ensure thread-safe access to shared image resources (i.e. image_callback temp. loss access to image_mutex_)
        std::lock_guard<std::mutex> lock(image_mutex_); //apriltags
        current_image = latest_image_.clone(); // Copy and Use latest_image_ from image_callback() //apriltags
        new_image_available_ = false; //apriltags

        if (latest_image_.empty()) //apriltags
        {
          RCLCPP_WARN(this->get_logger(), "Waiting for camera image..."); //apriltags
          continue; //apriltags
        }

        // Convert OpenCV BGR image to vpImage<vpRGBa>:
        for (int i = 0; i < current_image.rows; ++i) //apriltags //displays
        {
          for (int j = 0; j < current_image.cols; ++j) //apriltags //displays
          {
            const cv::Vec3b &pixel = current_image.at<cv::Vec3b>(i, j); //apriltags //displays
            I_color[i][j] = vpRGBa(pixel[2], pixel[1], pixel[0]); // BGR to RGBa //apriltags //displays
          }
        }

        vpDisplay::display(I_color); // displays //apriltags
        vpImageConvert::convert(I_color, I); // displays //apriltags

        std::stringstream ss; // displays
        ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit."; // displays
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red); // displays

        vpColVector v_c(6); // This will store velocity commands //ur16e

        // Tag Detection: 
        std::vector<vpHomogeneousMatrix> cMo_vec; //apriltags
        detector.detect(I, tagSize, cam, cMo_vec); //apriltags
        std::vector<int> tags_id = detector.getTagsId(); //apriltags
        std::map<int, int> tags_index; //apriltags
        tags_index[1] = 1; // blue up  //apriltags
        tags_index[2] = 2; // white left  //apriltags
        tags_index[3] = 3; // brown down  //apriltags
        tags_index[4] = 0; // black right  //apriltags

        // Model definition:
        std::vector<vpColVector> modelHoles(4); //apriltags
        for (int i = 0; i < modelHoles.size(); ++i) //apriltags
        {
          modelHoles[i].resize(3); // to avoid segmentation fault //apriltags
        }
        modelHoles[0] = {0.06512, 0.0, 0.0}; // TODO parametrize this //apriltags
        modelHoles[1] = {0.0, 0.06512, 0.0}; //apriltags
        modelHoles[2] = {-0.06512, 0.0, 0.0}; //apriltags
        modelHoles[3] = {0.0, -0.06512, 0.0}; //apriltags

        // Display camera pose for each tag and get tracked holes:
        std::vector<vpColVector> trackedHoles(4); //apriltags
        std::vector<bool> detectedMask(4, false); //apriltags
        std::vector<vpColVector> filteredModelHoles, filteredTrackedHoles; //apriltags
        filteredModelHoles.clear(); //apriltags
        filteredTrackedHoles.clear(); //apriltags
        for (int i = 0; i < trackedHoles.size(); ++i) //apriltags
        {
          trackedHoles[i].resize(3); // to avoid segmentation fault //apriltags
        }
        if (cMo_vec.size() >= 3 && cMo_vec.size() <= 4) //apriltags
        {
          for (size_t i = 0; i < cMo_vec.size(); i++) //apriltags
          {
            // vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3); // For each AprilTag
            vpTranslationVector t; //apriltags
            cMo_vec[i].extract(t); //apriltags

            int modelIndex = tags_index[tags_id[i]]; //apriltags
            trackedHoles[modelIndex] = {t[0], t[1], t[2]}; // routine //apriltags
            detectedMask[modelIndex] = true; //apriltags

            // Push the corresponding model and tracked holes into filtered vectors:
            filteredModelHoles.push_back(modelHoles[modelIndex]); //apriltags
            filteredTrackedHoles.push_back(trackedHoles[modelIndex]); //apriltags
          }
          // Now pass the filtered vectors to findPose:
          vpHomogeneousMatrix cMo = findPose(filteredModelHoles, filteredTrackedHoles); //apriltags

          cMo = findPose(filteredModelHoles, filteredTrackedHoles); // TODO write algorithm directly for column vectors //apriltags
          if (cMo == vpHomogeneousMatrix()) //apriltags
          {
            std::cerr << "WARNING: cMo is identity. Pose estimation failed or returned invalid transform." << std::endl; //apriltags

            // Print input data for debugging:
            std::cerr << "Filtered Model Holes:" << std::endl; //apriltags
            for (const auto &pt : filteredModelHoles) //apriltags
              std::cerr << pt.t() << std::endl; //apriltags

            std::cerr << "Filtered Tracked Holes:" << std::endl; //apriltags
            for (const auto &pt : filteredTrackedHoles) //apriltags
              std::cerr << pt.t() << std::endl; //apriltags
          }

          // To save current cMo in .yaml:
          vpPoseVector cPo(cMo); //apriltags
          std::stringstream ss_cPo; //apriltags
          std::cout << "cPo: " << cPo.t() << std::endl; //apriltags
          ss_cPo << package_path + "/data/ur_pose_cPo.yaml"; //apriltags
          cPo.saveYAML(ss_cPo.str(), cPo); // Save target pose in camera frame

          static bool first_time = true; //ur16e
          if (first_time) //ur16e
          {
            // Introduce security wrt tag positioning in order to avoid PI rotation (flip logic):
            std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2); //ur16e
            v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI); //ur16e
            std::cout << "First v_oMo[1]:\n" //ur16e
                      << v_oMo[1] << std::endl; //ur16e
            for (size_t i = 0; i < 2; i++) //ur16e
            {
              v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse(); //ur16e
            }
            if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) //ur16e
            {
              oMo = v_oMo[0]; //ur16e
            }
            else //ur16e
            {
              std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl; //ur16e
              oMo = v_oMo[1]; // Introduce PI rotation //ur16e
            }
          }
          // Update visual features:
          cdMc = cdMo * oMo * cMo.inverse(); //ur16e //displays
          t.buildFrom(cdMc); //ur16e //displays
          tu.buildFrom(cdMc); //ur16e //displays

          if (opt_task_sequencing) //ur16e
          {
            if (!servo_started) //ur16e
            {
              if (send_velocities) //ur16e
              {
                servo_started = true; //ur16e
              }
              t_init_servo = vpTime::measureTimeMs(); //ur16e
            }
            v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.); //ur16e
          }
          else //ur16e
          {
            v_c = task.computeControlLaw(); // Calculated Velocity Commands //ur16e
          }
          std::cout << "v_c: " << v_c.t() << std::endl; //ur16e

          // Display desired and current pose features:
          vpDisplay::displayFrame(I_color, cdMo * oMo, cam, 0.06512, vpColor::yellow, 2); //displays
          vpDisplay::displayFrame(I_color, cMo, cam, 0.06512, vpColor::none, 3); //displays
          // TODO display trajectory of Pose

          if (opt_plot) //displays
          {
            plotter->plot(0, iter_plot, task.getError()); //displays
            plotter->plot(1, iter_plot, v_c); //displays
            iter_plot++; //displays
          }


          vpTranslationVector cd_t_c = cdMc.getTranslationVector(); //ur16e
          vpThetaUVector cd_tu_c = cdMc.getThetaUVector(); //ur16e
          double error_tr = sqrt(cd_t_c.sumSquare()); //ur16e
          double error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare())); //ur16e

          ss.str(""); //displays
          ss << "error_t: " << error_tr; //displays
          vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red); //displays
          ss.str(""); //displays
          ss << "error_tu: " << error_tu; //displays
          vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red); //displays

          if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) //ur16e
          {
            has_converged = true; //ur16e
            std::cout << "Servo task has converged" << std::endl; //ur16e
            ; //ur16e
            vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red); //ur16e
          }

          if (first_time) //ur16e
          {
            first_time = false; //ur16e
          }
        } // end if (cMo_vec.size() == 4)

        else //ur16e
        {
          v_c = 0; //ur16e
        }

        if (!send_velocities) //ur16e
        {
          v_c = 0; //ur16e
        }
        
        robot.setVelocity(vpRobot::CAMERA_FRAME, v_c); // Send to the robot //ur16e

        ss.str("");
        ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";

        vpDisplay::displayText(I_color, 40, 20, ss.str(), vpColor::red); //displays
        vpDisplay::flush(I_color); //displays

        vpMouseButton::vpMouseButtonType button; //ur16e
        if (vpDisplay::getClick(I_color, button, false)) //ur16e
        {
          switch (button) //ur16e
          {
          case vpMouseButton::button1: //ur16e
            send_velocities = !send_velocities; //ur16e
            break; //ur16e

          case vpMouseButton::button3: //ur16e
            final_quit = true; //ur16e
            v_c = 0; //ur16e
            {
              vpPoseVector fPe; //ur16e
              robot.getPosition(vpRobot::END_EFFECTOR_FRAME, fPe); //ur16e
              std::stringstream ss_fPe; //ur16e
              std::cout << fPe; //ur16e
              ss_fPe << package_path + "/data/ur_pose_fPe.yaml"; //ur16e
              fPe.saveYAML(ss_fPe.str(), fPe); // Save object pose in robot frame //ur16e
            }
            break; //ur16e

          default: //ur16e
            break; //ur16e
          }
        }
        vpDisplay::flush(I_color);
        std::cout << "Bottom of While Loop" << std::endl;
      }

      if (!display_off) //displays
      {
        delete d1; //displays
      }

    }
    catch (const vpException &e) //ur16e
    {
      std::cerr << "Catch an exception: " << e.getMessage() << std::endl; //ur16e
    }
    catch (const std::exception &e) //ur16e
    {
      std::cout << "ur_rtde exception: " << e.what() << std::endl; //ur16e
      std::exit(EXIT_FAILURE); //ur16e
    }
    std::cout << "After Catches" << std::endl; //ur16e
  }

  // Declare class member variables here:

  // Kabsch.cpp Start: ======================================================================
  //apriltags
  Eigen::Affine3d Find3DAffineTransformSameScale(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out)
  {
    // Default output
    Eigen::Affine3d A;
    A.linear() = Eigen::Matrix3d::Identity(3, 3);
    A.translation() = Eigen::Vector3d::Zero();

    if (in.cols() != out.cols())
      throw "Find3DAffineTransform(): input data mis-match";

    // Find the centroids then shift to the origin
    Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
    Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
    for (int col = 0; col < in.cols(); col++)
    {
      in_ctr += in.col(col);
      out_ctr += out.col(col);
    }
    in_ctr /= in.cols();
    out_ctr /= out.cols();
    for (int col = 0; col < in.cols(); col++)
    {
      in.col(col) -= in_ctr;
      out.col(col) -= out_ctr;
    }

    // SVD
    Eigen::MatrixXd Cov = in * out.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Find the rotation
    double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > 0)
      d = 1.0;
    else
      d = -1.0;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(2, 2) = d;
    Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

    // The final transform
    A.linear() = R;
    A.translation() = (out_ctr - R * in_ctr);

    return A;
  }
  // VisP wrapper
  //apriltags
  vpHomogeneousMatrix findPose(std::vector<vpColVector> in, std::vector<vpColVector> out)
  {
    // Create an Eigen Matrix to hold the points
    Eigen::Matrix3Xd inputPoints(3, in.size()), outputPoints(3, out.size());

    // Fill the Eigen matrix with the coordinates from vpColVector
    for (size_t i = 0; i < in.size(); ++i)
    {
      inputPoints(0, i) = in[i][0];
      inputPoints(1, i) = in[i][1];
      inputPoints(2, i) = in[i][2];
    }
    for (size_t i = 0; i < out.size(); ++i)
    {
      outputPoints(0, i) = out[i][0];
      outputPoints(1, i) = out[i][1];
      outputPoints(2, i) = out[i][2];
    }

    Eigen::Affine3d A = Find3DAffineTransformSameScale(inputPoints, outputPoints);

    vpHomogeneousMatrix cMo;
    cMo[0][0] = A.linear()(0, 0);
    cMo[0][1] = A.linear()(0, 1);
    cMo[0][2] = A.linear()(0, 2);
    cMo[0][3] = A.translation()(0);
    cMo[1][0] = A.linear()(1, 0);
    cMo[1][1] = A.linear()(1, 1);
    cMo[1][2] = A.linear()(1, 2);
    cMo[1][3] = A.translation()(1);
    cMo[2][0] = A.linear()(2, 0);
    cMo[2][1] = A.linear()(2, 1);
    cMo[2][2] = A.linear()(2, 2);
    cMo[2][3] = A.translation()(2);
    return cMo;
  }
  // Kabsch.cpp End: ========================================================================

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Declare shared pointer to subscription //apriltags
  cv::Mat latest_image_; // Hold latest image //apriltags
  std::mutex image_mutex_; // Declare mutex for latest_image_ so it's used by only one thread at a time //apriltags
  std::atomic<bool> new_image_available_{false}; // For new velocity calc using only new image //apriltags
  // Runs when new image message received from subscription:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) //apriltags
  {
    try
    {
      RCLCPP_INFO(this->get_logger(), "Image callback triggered! Image resolution: %d x %d", msg->width, msg->height);

      // Convert ROS2 image message to OpenCV-compatible cv::Mat:
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
      std::lock_guard<std::mutex> lock(image_mutex_); // Locks the image_mutex_ from run_servo_loop()
      latest_image_ = cv_ptr->image.clone(); // Copy image to latest_image_
      new_image_available_ = true; 
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

private:
};

int main(int argc, char *argv[])
{
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_UR_RTDE)
  rclcpp::init(argc, argv); // Initialize ROS2

  auto node = std::make_shared<DemoServoNode>(argc, argv); // Create Custom Node

  rclcpp::executors::MultiThreadedExecutor executor; // Create multithreaded executor

  executor.add_node(node); // Register the node

  // Launch control loop in a dedicated thread:
  std::thread servo_thread(&DemoServoNode::run_servo_loop, node.get());

  executor.spin(); // ROS2 to handle callbacks concurrently using thread pool

  servo_thread.join(); // Start spinning for callbacks

  rclcpp::shutdown();

  return EXIT_SUCCESS;

#else
  (void)argc;
  (void)argv;
  std::cout << "ViSP demo_servo.cpp cannot run due to missing dependencies:\n";

#ifndef VISP_HAVE_APRILTAG
  std::cout << " - Apriltag support is missing. Please configure and build ViSP with AprilTag.\n";
#endif

#ifndef VISP_HAVE_UR_RTDE
  std::cout << " - UR RTDE (Universal Robots) support is missing. Build ViSP with libur_rtde.\n";
#endif

#ifndef VISP_HAVE_REALSENSE2
  std::cout << " - RealSense2 support is missing. Install librealsense2 and rebuild ViSP.\n";
#endif

  return EXIT_SUCCESS;

#endif
}