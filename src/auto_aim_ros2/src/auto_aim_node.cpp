#include <auto_aim_interfaces/msg/armor_detection.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/robot_ctrl.hpp>
#include <auto_aim_interfaces/msg/vision.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <list>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "auto_aim_ros2/fire_controller.hpp"

namespace auto_aim_ros2
{

  class AutoAimNode : public rclcpp::Node
  {
  public:
    AutoAimNode()
        : Node("auto_aim_node")
    {
      const auto project_root = declare_parameter<std::string>(
          "project_root", "/home/nvidia/game_26_current/sp_vision_25");
      const auto config_parameter =
          declare_parameter<std::string>("config_path", "configs/school.yaml");
      const auto image_topic = declare_parameter<std::string>("image_topic", "/image_raw");
      const auto camera_info_topic =
          declare_parameter<std::string>("camera_info_topic", "/camera_info");
      const auto vision_topic = declare_parameter<std::string>("vision_topic", "/Vision_data");
      const auto armors_topic =
          declare_parameter<std::string>("armors_topic", "/auto_aim/armors");
      const auto debug_topic =
          declare_parameter<std::string>("debug_image_topic", "/auto_aim/debug_image");
      const auto count_topic =
          declare_parameter<std::string>("detection_count_topic", "/auto_aim/detection_count");
      const auto control_topic =
          declare_parameter<std::string>("control_topic", "/Robot_ctrl_data");
      const auto control_preview_topic = declare_parameter<std::string>(
          "control_preview_topic", "/auto_aim/control_preview");
      publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
      enable_control_output_ = declare_parameter<bool>("enable_control_output", false);
      require_auto_aim_mode_ = declare_parameter<bool>("require_auto_aim_mode", true);
      vision_timeout_s_ = declare_parameter<double>("vision_timeout_s", 0.2);
      small_armor_width_m_ = declare_parameter<double>("small_armor_width_m", 0.135);
      big_armor_width_m_ = declare_parameter<double>("big_armor_width_m", 0.230);
      armor_height_m_ = declare_parameter<double>("armor_height_m", 0.056);
      max_pnp_reprojection_error_px_ =
          declare_parameter<double>("max_pnp_reprojection_error_px", 8.0);
      max_control_yaw_error_rad_ =
          declare_parameter<double>("max_control_yaw_error_deg", 15.0) * CV_PI / 180.0;
      max_control_pitch_error_rad_ =
          declare_parameter<double>("max_control_pitch_error_deg", 10.0) * CV_PI / 180.0;
      max_control_step_rad_ =
          declare_parameter<double>("max_control_step_deg", 1.0) * CV_PI / 180.0;
      control_target_filter_alpha_ =
          declare_parameter<double>("control_target_filter_alpha", 0.25);
      control_yaw_deadband_rad_ =
          declare_parameter<double>("control_yaw_deadband_deg", 0.25) * CV_PI / 180.0;
      control_pitch_deadband_rad_ =
          declare_parameter<double>("control_pitch_deadband_deg", 0.20) * CV_PI / 180.0;
      enable_fire_ = declare_parameter<bool>("enable_fire", false);
      fire_hold_s_ = declare_parameter<double>("fire_hold_s", 0.15);
      fire_interval_s_ = declare_parameter<double>("fire_interval_s", 0.5);
      fire_shot_period_s_ = declare_parameter<double>("fire_shot_period_s", 0.1);
      fire_burst_count_ = declare_parameter<int>("fire_burst_count", 3);
      fire_yaw_tolerance_rad_ =
          declare_parameter<double>("fire_yaw_tolerance_deg", 0.50) * CV_PI / 180.0;
      fire_pitch_tolerance_rad_ =
          declare_parameter<double>("fire_pitch_tolerance_deg", 0.50) * CV_PI / 180.0;
      fire_max_yaw_velocity_deg_s_ =
          declare_parameter<double>("fire_max_yaw_velocity_deg_s", 5.0);
      fire_max_pitch_velocity_deg_s_ =
          declare_parameter<double>("fire_max_pitch_velocity_deg_s", 5.0);
      fire_max_reprojection_error_px_ =
          declare_parameter<double>("fire_max_reprojection_error_px", 3.0);
      if (!std::isfinite(max_control_yaw_error_rad_) || max_control_yaw_error_rad_ <= 0.0 ||
          !std::isfinite(max_control_pitch_error_rad_) || max_control_pitch_error_rad_ <= 0.0 ||
          !std::isfinite(max_control_step_rad_) || max_control_step_rad_ <= 0.0 ||
          !std::isfinite(control_target_filter_alpha_) || control_target_filter_alpha_ <= 0.0 ||
          control_target_filter_alpha_ > 1.0 ||
          !std::isfinite(control_yaw_deadband_rad_) || control_yaw_deadband_rad_ < 0.0 ||
          !std::isfinite(control_pitch_deadband_rad_) || control_pitch_deadband_rad_ < 0.0 ||
          !std::isfinite(fire_hold_s_) || fire_hold_s_ < 0.0 ||
          !std::isfinite(fire_interval_s_) || fire_interval_s_ < 0.0 ||
          !std::isfinite(fire_shot_period_s_) || fire_shot_period_s_ <= 0.0 ||
          fire_burst_count_ <= 0 ||
          !std::isfinite(fire_yaw_tolerance_rad_) || fire_yaw_tolerance_rad_ <= 0.0 ||
          !std::isfinite(fire_pitch_tolerance_rad_) || fire_pitch_tolerance_rad_ <= 0.0 ||
          !std::isfinite(fire_max_yaw_velocity_deg_s_) || fire_max_yaw_velocity_deg_s_ < 0.0 ||
          !std::isfinite(fire_max_pitch_velocity_deg_s_) ||
          fire_max_pitch_velocity_deg_s_ < 0.0 ||
          !std::isfinite(fire_max_reprojection_error_px_) ||
          fire_max_reprojection_error_px_ <= 0.0)
      {
        throw std::runtime_error("invalid control/filter/fire parameter");
      }
      fire_controller_ = std::make_unique<FireController>(FireControllerConfig{
          fire_hold_s_, fire_interval_s_, fire_shot_period_s_, fire_burst_count_});
      if (enable_fire_ && !enable_control_output_)
      {
        RCLCPP_WARN(
            get_logger(), "enable_fire=true has no effect while enable_control_output=false");
      }

      const std::filesystem::path root(project_root);
      if (!std::filesystem::is_directory(root))
      {
        throw std::runtime_error("sp_vision project_root does not exist: " + root.string());
      }

      std::filesystem::path config_path(config_parameter);
      if (config_path.is_relative())
        config_path = root / config_path;
      if (!std::filesystem::is_regular_file(config_path))
      {
        throw std::runtime_error("auto aim config does not exist: " + config_path.string());
      }

      // Upstream model paths are relative to the sp_vision project root.  This
      // process contains only the detector node, so changing its working
      // directory cannot affect the camera or serial ROS processes.
      std::filesystem::current_path(root);
      std::filesystem::create_directories(root / "logs");
      detector_ = std::make_unique<auto_aim::YOLO>(config_path.string(), false);
      solver_ = std::make_unique<auto_aim::Solver>(config_path.string());
      tracker_ = std::make_unique<auto_aim::Tracker>(config_path.string(), *solver_);
      aimer_ = std::make_unique<auto_aim::Aimer>(config_path.string());
      load_camera_model_from_config(config_path.string());

      armors_publisher_ = create_publisher<auto_aim_interfaces::msg::Armors>(armors_topic, 10);
      count_publisher_ = create_publisher<std_msgs::msg::Int32>(count_topic, 10);
      debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(
          debug_topic, rclcpp::SensorDataQoS());

      control_preview_publisher_ =
          create_publisher<auto_aim_interfaces::msg::RobotCtrl>(control_preview_topic, 10);
      control_publisher_ =
          create_publisher<auto_aim_interfaces::msg::RobotCtrl>(control_topic, 10);

      vision_subscription_ = create_subscription<auto_aim_interfaces::msg::Vision>(
          vision_topic, rclcpp::QoS(10).best_effort(),
          std::bind(&AutoAimNode::vision_callback, this, std::placeholders::_1));
      camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_topic, rclcpp::SensorDataQoS(),
          std::bind(&AutoAimNode::camera_info_callback, this, std::placeholders::_1));
      image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          image_topic, rclcpp::SensorDataQoS(),
          std::bind(&AutoAimNode::image_callback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Config: %s", config_path.c_str());
      RCLCPP_INFO(get_logger(), "Subscribing to image topic: %s", image_topic.c_str());
      RCLCPP_INFO(get_logger(), "Subscribing to camera info topic: %s", camera_info_topic.c_str());
      if (enable_control_output_ && enable_fire_)
      {
        RCLCPP_WARN(
            get_logger(),
            "Gimbal control and FIRE are ENABLED: burst=%d pulses, hold=%.2f s, "
            "inter-burst interval=%.2f s",
            fire_burst_count_, fire_hold_s_, fire_interval_s_);
      }
      else if (enable_control_output_)
      {
        RCLCPP_WARN(
            get_logger(),
            "Gimbal control output is ENABLED; fire_command remains forced to zero.");
      }
      else
      {
        RCLCPP_WARN(
            get_logger(),
            "Safe preview mode: commands publish only on /auto_aim/control_preview; "
            "/Robot_ctrl_data and firing are disabled.");
      }
    }

  private:
    struct CameraModel
    {
      bool valid = false;
      uint32_t width = 0;
      uint32_t height = 0;
      cv::Mat camera_matrix;
      cv::Mat distortion_coefficients;
    };

    struct PoseEstimate
    {
      bool valid = false;
      cv::Vec3d position_camera{0.0, 0.0, 0.0};
      double distance_m = 0.0;
      double yaw_rad = 0.0;
      double pitch_rad = 0.0;
      double reprojection_error_px = 0.0;
    };

    struct VisionSnapshot
    {
      bool received = false;
      bool fresh = false;
      bool quaternion_valid = false;
      auto_aim_interfaces::msg::Vision message;
      Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    };

    static std::string armor_label(const auto_aim::Armor &armor)
    {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(2) << armor.confidence << " ";
      if (static_cast<std::size_t>(armor.color) < auto_aim::COLORS.size())
      {
        stream << auto_aim::COLORS[armor.color] << " ";
      }
      if (static_cast<std::size_t>(armor.name) < auto_aim::ARMOR_NAMES.size())
      {
        stream << auto_aim::ARMOR_NAMES[armor.name];
      }
      return stream.str();
    }

    void load_camera_model_from_config(const std::string &config_path)
    {
      try
      {
        const auto yaml = YAML::LoadFile(config_path);
        const auto matrix_values = yaml["camera_matrix"].as<std::vector<double>>();
        const auto distortion_values = yaml["distort_coeffs"].as<std::vector<double>>();
        if (matrix_values.size() != 9 || distortion_values.size() < 4)
        {
          throw std::runtime_error("camera_matrix/distort_coeffs have invalid lengths");
        }

        CameraModel model;
        model.valid = true;
        model.camera_matrix = cv::Mat(3, 3, CV_64F);
        std::copy(
            matrix_values.begin(), matrix_values.end(), model.camera_matrix.ptr<double>());
        model.distortion_coefficients = cv::Mat(
            1, static_cast<int>(distortion_values.size()), CV_64F);
        std::copy(
            distortion_values.begin(), distortion_values.end(),
            model.distortion_coefficients.ptr<double>());

        solver_->set_camera_model(model.camera_matrix, model.distortion_coefficients);
        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_model_ = std::move(model);
        RCLCPP_INFO(
            get_logger(), "Loaded fallback camera model from %s; waiting for /camera_info",
            config_path.c_str());
      }
      catch (const std::exception &error)
      {
        RCLCPP_WARN(
            get_logger(), "No usable fallback camera model in %s: %s",
            config_path.c_str(), error.what());
      }
    }

    void vision_callback(const auto_aim_interfaces::msg::Vision::ConstSharedPtr message)
    {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      latest_vision_ = *message;
      has_vision_ = true;
      latest_vision_received_at_ = std::chrono::steady_clock::now();
    }

    VisionSnapshot current_vision()
    {
      VisionSnapshot snapshot;
      {
        std::lock_guard<std::mutex> lock(vision_mutex_);
        snapshot.received = has_vision_;
        if (!has_vision_)
          return snapshot;
        snapshot.message = latest_vision_;
        snapshot.fresh = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - latest_vision_received_at_)
                             .count() <=
                         vision_timeout_s_;
      }

      snapshot.quaternion = Eigen::Quaterniond(
          snapshot.message.quaternion[0], snapshot.message.quaternion[1],
          snapshot.message.quaternion[2], snapshot.message.quaternion[3]);
      const double squared_norm = snapshot.quaternion.squaredNorm();
      snapshot.quaternion_valid =
          std::isfinite(squared_norm) && squared_norm > 1e-6 &&
          std::abs(squared_norm - 1.0) <= 0.1;
      if (snapshot.quaternion_valid)
        snapshot.quaternion.normalize();
      return snapshot;
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
    {
      if (
          message->width == 0 || message->height == 0 || message->d.size() < 4 ||
          !std::isfinite(message->k[0]) || !std::isfinite(message->k[4]) ||
          message->k[0] <= 0.0 || message->k[4] <= 0.0)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Ignoring invalid camera_info message");
        return;
      }

      CameraModel model;
      model.valid = true;
      model.width = message->width;
      model.height = message->height;
      model.camera_matrix = cv::Mat(
                                3, 3, CV_64F, const_cast<double *>(message->k.data()))
                                .clone();
      model.distortion_coefficients = cv::Mat(
          1, static_cast<int>(message->d.size()), CV_64F);
      std::copy(
          message->d.begin(), message->d.end(),
          model.distortion_coefficients.ptr<double>());

      bool first_live_model = false;
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        first_live_model = !has_live_camera_info_;
        camera_model_ = std::move(model);
        has_live_camera_info_ = true;
      }
      const auto active_model = current_camera_model();
      solver_->set_camera_model(
          active_model.camera_matrix, active_model.distortion_coefficients);
      if (first_live_model)
      {
        RCLCPP_INFO(
            get_logger(), "Camera model ready: %ux%u fx=%.2f fy=%.2f",
            message->width, message->height, message->k[0], message->k[4]);
      }
    }

    CameraModel current_camera_model()
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      CameraModel model;
      model.valid = camera_model_.valid;
      model.width = camera_model_.width;
      model.height = camera_model_.height;
      model.camera_matrix = camera_model_.camera_matrix.clone();
      model.distortion_coefficients = camera_model_.distortion_coefficients.clone();
      return model;
    }

    std::optional<PoseEstimate> estimate_pose(
        const auto_aim::Armor &armor, const CameraModel &camera_model) const
    {
      if (!camera_model.valid || armor.points.size() != 4)
        return std::nullopt;

      const double width_m =
          armor.type == auto_aim::ArmorType::big ? big_armor_width_m_ : small_armor_width_m_;
      const double half_width = width_m / 2.0;
      const double half_height = armor_height_m_ / 2.0;
      const std::vector<cv::Point3f> object_points{
          {static_cast<float>(-half_width), static_cast<float>(-half_height), 0.0F},
          {static_cast<float>(half_width), static_cast<float>(-half_height), 0.0F},
          {static_cast<float>(half_width), static_cast<float>(half_height), 0.0F},
          {static_cast<float>(-half_width), static_cast<float>(half_height), 0.0F}};

      cv::Vec3d rotation_vector;
      cv::Vec3d translation_vector;
      try
      {
        if (!cv::solvePnP(
                object_points, armor.points, camera_model.camera_matrix,
                camera_model.distortion_coefficients, rotation_vector, translation_vector,
                false, cv::SOLVEPNP_IPPE))
        {
          return std::nullopt;
        }

        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(
            object_points, rotation_vector, translation_vector,
            camera_model.camera_matrix, camera_model.distortion_coefficients,
            reprojected_points);

        double reprojection_error = 0.0;
        for (std::size_t index = 0; index < armor.points.size(); ++index)
        {
          reprojection_error += cv::norm(armor.points[index] - reprojected_points[index]);
        }
        reprojection_error /= static_cast<double>(armor.points.size());

        PoseEstimate estimate;
        estimate.position_camera = translation_vector;
        estimate.distance_m = cv::norm(translation_vector);
        estimate.yaw_rad = std::atan2(translation_vector[0], translation_vector[2]);
        estimate.pitch_rad = std::atan2(
            -translation_vector[1],
            std::hypot(translation_vector[0], translation_vector[2]));
        estimate.reprojection_error_px = reprojection_error;
        estimate.valid =
            translation_vector[2] > 0.0 && std::isfinite(estimate.distance_m) &&
            std::isfinite(reprojection_error) &&
            reprojection_error <= max_pnp_reprojection_error_px_;
        return estimate;
      }
      catch (const cv::Exception &)
      {
        return std::nullopt;
      }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
    {
      const auto started = std::chrono::steady_clock::now();

      cv_bridge::CvImageConstPtr image;
      try
      {
        image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
      }
      catch (const cv_bridge::Exception &error)
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "cv_bridge conversion failed: %s", error.what());
        return;
      }

      std::list<auto_aim::Armor> detections;
      try
      {
        detections = detector_->detect(image->image, frame_count_++);
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "Armor detection failed: %s", error.what());
        return;
      }

      auto output = auto_aim_interfaces::msg::Armors();
      output.header = message->header;
      output.detections.reserve(detections.size());

      auto camera_model = current_camera_model();
      if (
          camera_model.valid && camera_model.width != 0 && camera_model.height != 0 &&
          (camera_model.width != message->width || camera_model.height != message->height))
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "camera_info resolution %ux%u does not match image %ux%u; PnP disabled",
            camera_model.width, camera_model.height, message->width, message->height);
        camera_model.valid = false;
      }
      else if (!camera_model.valid)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Waiting for valid /camera_info; PnP disabled");
      }

      cv::Mat debug_image;
      if (publish_debug_image_)
        debug_image = image->image.clone();
      std::size_t valid_pose_count = 0;
      std::list<auto_aim::Armor> tracking_detections;

      for (const auto &armor : detections)
      {
        auto detection = auto_aim_interfaces::msg::ArmorDetection();
        detection.color = static_cast<uint8_t>(armor.color);
        detection.number = static_cast<uint8_t>(armor.name);
        detection.armor_type = static_cast<uint8_t>(armor.type);
        detection.confidence = static_cast<float>(armor.confidence);
        detection.center.x = armor.center.x;
        detection.center.y = armor.center.y;
        detection.center.z = 0.0F;

        const auto corner_count = std::min<std::size_t>(armor.points.size(), detection.corners.size());
        std::vector<cv::Point> contour;
        contour.reserve(corner_count);
        for (std::size_t index = 0; index < corner_count; ++index)
        {
          detection.corners[index].x = armor.points[index].x;
          detection.corners[index].y = armor.points[index].y;
          detection.corners[index].z = 0.0F;
          contour.emplace_back(cvRound(armor.points[index].x), cvRound(armor.points[index].y));
        }

        const auto pose = estimate_pose(armor, camera_model);
        if (pose.has_value())
        {
          detection.pose_valid = pose->valid;
          detection.position_camera.x = pose->position_camera[0];
          detection.position_camera.y = pose->position_camera[1];
          detection.position_camera.z = pose->position_camera[2];
          detection.distance_m = static_cast<float>(pose->distance_m);
          detection.yaw_rad = static_cast<float>(pose->yaw_rad);
          detection.pitch_rad = static_cast<float>(pose->pitch_rad);
          detection.reprojection_error_px =
              static_cast<float>(pose->reprojection_error_px);
          if (pose->valid)
          {
            ++valid_pose_count;
            tracking_detections.emplace_back(armor);
          }
        }

        if (publish_debug_image_ && contour.size() >= 3)
        {
          cv::polylines(debug_image, contour, true, cv::Scalar(0, 255, 0), 2);
          cv::putText(
              debug_image, armor_label(armor),
              cv::Point(cvRound(armor.center.x), cvRound(armor.center.y)),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
          if (pose.has_value())
          {
            std::ostringstream pose_label;
            pose_label << std::fixed << std::setprecision(2)
                       << "d=" << pose->distance_m << "m "
                       << "y=" << pose->yaw_rad * 180.0 / CV_PI << "deg "
                       << "p=" << pose->pitch_rad * 180.0 / CV_PI << "deg "
                       << "e=" << pose->reprojection_error_px << "px";
            cv::putText(
                debug_image, pose_label.str(),
                cv::Point(cvRound(armor.center.x), cvRound(armor.center.y) + 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                pose->valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
          }
        }
        output.detections.emplace_back(std::move(detection));
      }

      const auto vision = current_vision();
      if (vision.quaternion_valid)
        solver_->set_R_gimbal2world(vision.quaternion);

      std::list<auto_aim::Target> targets;
      io::Command command{false, false, 0.0, 0.0};
      try
      {
        targets = tracker_->track(tracking_detections, started);
        const double bullet_speed =
            vision.received && std::isfinite(vision.message.shoot_speed) &&
                    vision.message.shoot_speed > 1.0F
                ? vision.message.shoot_speed
                : 30.0;
        command = aimer_->aim(targets, started, bullet_speed);
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "Tracking/aiming failed: %s", error.what());
      }

      const auto tracker_state = tracker_->state();
      const bool command_finite = std::isfinite(command.yaw) && std::isfinite(command.pitch);
      const bool target_locked =
          tracker_state == "tracking" && command.control && command_finite;
      const bool mode_allowed =
          !require_auto_aim_mode_ ||
          (vision.received && vision.message.mode == auto_aim_interfaces::msg::Vision::AUTO_AIM_MODE);
      // The MCU reports feedback in degrees, while the aimer and ROS control
      // messages use radians.  The serial node converts ROS commands back to
      // MCU wire degrees.
      const double current_yaw_rad =
          vision.received ? vision.message.yaw * CV_PI / 180.0 : 0.0;
      const double current_pitch_rad =
          vision.received ? vision.message.pitch * CV_PI / 180.0 : 0.0;
      const double raw_yaw_error_rad =
          std::remainder(command.yaw - current_yaw_rad, 2.0 * CV_PI);
      const double raw_pitch_error_rad = command.pitch - current_pitch_rad;
      const bool target_within_control_window =
          command_finite && vision.received &&
          std::abs(raw_yaw_error_rad) <= max_control_yaw_error_rad_ &&
          std::abs(raw_pitch_error_rad) <= max_control_pitch_error_rad_;
      const bool actuation_allowed =
          target_locked && vision.fresh && vision.quaternion_valid && mode_allowed &&
          target_within_control_window;

      // Detector corners, PnP and the EKF all contribute a little noise even
      // when the target is motionless.  Filter the absolute target angles so
      // that the gimbal controller does not chase every frame of that noise.
      if (target_locked && target_within_control_window)
      {
        if (!filtered_target_valid_)
        {
          filtered_target_yaw_rad_ = command.yaw;
          filtered_target_pitch_rad_ = command.pitch;
          filtered_target_valid_ = true;
        }
        else
        {
          filtered_target_yaw_rad_ += control_target_filter_alpha_ * std::remainder(
              command.yaw - filtered_target_yaw_rad_, 2.0 * CV_PI);
          filtered_target_pitch_rad_ +=
              control_target_filter_alpha_ * (command.pitch - filtered_target_pitch_rad_);
        }
      }
      else
      {
        filtered_target_valid_ = false;
      }

      const double control_target_yaw_rad =
          filtered_target_valid_ ? filtered_target_yaw_rad_ : command.yaw;
      const double control_target_pitch_rad =
          filtered_target_valid_ ? filtered_target_pitch_rad_ : command.pitch;
      const double yaw_error_rad =
          std::remainder(control_target_yaw_rad - current_yaw_rad, 2.0 * CV_PI);
      const double pitch_error_rad = control_target_pitch_rad - current_pitch_rad;

      double target_reprojection_error_px = std::numeric_limits<double>::infinity();
      if (!targets.empty())
      {
        const auto tracked_number = static_cast<uint8_t>(targets.front().name);
        for (const auto &detection : output.detections)
        {
          if (detection.pose_valid && detection.number == tracked_number)
          {
            target_reprojection_error_px = std::min(
                target_reprojection_error_px,
                static_cast<double>(detection.reprojection_error_px));
          }
        }
      }
      const bool fire_aim_aligned =
          std::abs(yaw_error_rad) <= fire_yaw_tolerance_rad_ &&
          std::abs(pitch_error_rad) <= fire_pitch_tolerance_rad_;
      const bool fire_velocity_safe =
          vision.received && std::isfinite(vision.message.yaw_vel) &&
          std::isfinite(vision.message.pitch_vel) &&
          std::abs(vision.message.yaw_vel) <= fire_max_yaw_velocity_deg_s_ &&
          std::abs(vision.message.pitch_vel) <= fire_max_pitch_velocity_deg_s_;
      const bool fire_pose_valid =
          std::isfinite(target_reprojection_error_px) &&
          target_reprojection_error_px <= fire_max_reprojection_error_px_;
      const bool fire_eligible =
          enable_fire_ && enable_control_output_ && actuation_allowed &&
          fire_aim_aligned && fire_velocity_safe && fire_pose_valid;
      const int fire_command =
          fire_controller_->update(fire_eligible, std::chrono::steady_clock::now());

      if (enable_fire_)
      {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Fire gate: eligible=%s aim=%s velocity=%s pose=%s "
            "error=(%.2f,%.2f)deg velocity=(%.2f,%.2f)deg/s reprojection=%.2fpx pulse=%d",
            fire_eligible ? "yes" : "no", fire_aim_aligned ? "pass" : "reject",
            fire_velocity_safe ? "pass" : "reject", fire_pose_valid ? "pass" : "reject",
            yaw_error_rad * 180.0 / CV_PI, pitch_error_rad * 180.0 / CV_PI,
            vision.received ? vision.message.yaw_vel : 0.0,
            vision.received ? vision.message.pitch_vel : 0.0,
            target_reprojection_error_px, fire_command);
      }

      if (target_locked && vision.fresh && vision.quaternion_valid && mode_allowed &&
          !target_within_control_window)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Control rejected: target jump yaw=%.2f deg pitch=%.2f deg exceeds safety window",
            raw_yaw_error_rad * 180.0 / CV_PI, raw_pitch_error_rad * 180.0 / CV_PI);
      }

      auto control_preview = auto_aim_interfaces::msg::RobotCtrl();
      control_preview.header = message->header;
      control_preview.yaw =
          command_finite ? static_cast<float>(control_target_yaw_rad) : 0.0F;
      control_preview.pitch =
          command_finite ? static_cast<float>(control_target_pitch_rad) : 0.0F;
      control_preview.yaw_vel = 0.0F;
      control_preview.yaw_acc = 0.0F;
      control_preview.pitch_vel = 0.0F;
      control_preview.pitch_acc = 0.0F;
      control_preview.target_lock = target_locked ? auto_aim_interfaces::msg::RobotCtrl::TARGET_LOCKED : auto_aim_interfaces::msg::RobotCtrl::TARGET_UNLOCKED;
      control_preview.fire_command = fire_command;
      control_preview_publisher_->publish(control_preview);

      if (target_locked && vision.received)
      {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Control preview: feedback=(%.2f, %.2f) deg target=(%.2f, %.2f) deg "
            "error=(%.2f, %.2f) deg safety=%s fire=%d",
            current_yaw_rad * 180.0 / CV_PI, current_pitch_rad * 180.0 / CV_PI,
            control_target_yaw_rad * 180.0 / CV_PI,
            control_target_pitch_rad * 180.0 / CV_PI,
            yaw_error_rad * 180.0 / CV_PI, pitch_error_rad * 180.0 / CV_PI,
            target_within_control_window ? "PASS" : "REJECT", fire_command);
      }

      if (enable_control_output_)
      {
        auto control = control_preview;
        if (actuation_allowed)
        {
          if (!control_hold_valid_)
          {
            held_control_yaw_rad_ = current_yaw_rad;
            held_control_pitch_rad_ = current_pitch_rad;
            yaw_in_deadband_ = false;
            pitch_in_deadband_ = false;
            control_hold_valid_ = true;
          }

          // Schmitt-trigger deadbands prevent repeated enter/exit near the
          // threshold.  Once settled, retain the last absolute setpoint rather
          // than replacing it with each noisy measurement.
          constexpr double deadband_exit_scale = 1.5;
          if (yaw_in_deadband_)
          {
            yaw_in_deadband_ =
                std::abs(yaw_error_rad) <= control_yaw_deadband_rad_ * deadband_exit_scale;
          }
          else if (std::abs(yaw_error_rad) <= control_yaw_deadband_rad_)
          {
            yaw_in_deadband_ = true;
          }
          if (pitch_in_deadband_)
          {
            pitch_in_deadband_ =
                std::abs(pitch_error_rad) <= control_pitch_deadband_rad_ * deadband_exit_scale;
          }
          else if (std::abs(pitch_error_rad) <= control_pitch_deadband_rad_)
          {
            pitch_in_deadband_ = true;
          }

          if (!yaw_in_deadband_)
          {
            const double yaw_step = std::clamp(
                yaw_error_rad, -max_control_step_rad_, max_control_step_rad_);
            held_control_yaw_rad_ = current_yaw_rad + yaw_step;
          }
          if (!pitch_in_deadband_)
          {
            const double pitch_step = std::clamp(
                pitch_error_rad, -max_control_step_rad_, max_control_step_rad_);
            held_control_pitch_rad_ = current_pitch_rad + pitch_step;
          }
          control.yaw = static_cast<float>(held_control_yaw_rad_);
          control.pitch = static_cast<float>(held_control_pitch_rad_);
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 500,
              "Control active: feedback=(%.2f, %.2f) deg target=(%.2f, %.2f) deg "
              "sent=(%.2f, %.2f) deg hold=(%s,%s)",
              current_yaw_rad * 180.0 / CV_PI, current_pitch_rad * 180.0 / CV_PI,
              control_target_yaw_rad * 180.0 / CV_PI,
              control_target_pitch_rad * 180.0 / CV_PI,
              control.yaw * 180.0 / CV_PI, control.pitch * 180.0 / CV_PI,
              yaw_in_deadband_ ? "yes" : "no", pitch_in_deadband_ ? "yes" : "no");
        }
        else
        {
          // Hold the measured pose while unlocked.  If firmware fails to gate
          // angle fields on target_lock, this is still a no-motion command.
          control.target_lock = auto_aim_interfaces::msg::RobotCtrl::TARGET_UNLOCKED;
          control.yaw = static_cast<float>(current_yaw_rad);
          control.yaw_vel = 0.0F;
          control.yaw_acc = 0.0F;
          control.pitch = static_cast<float>(current_pitch_rad);
          control.pitch_vel = 0.0F;
          control.pitch_acc = 0.0F;
          control_hold_valid_ = false;
          yaw_in_deadband_ = false;
          pitch_in_deadband_ = false;
        }
        control.fire_command = actuation_allowed ? fire_command : 0;
        control_publisher_->publish(control);
      }

      const auto elapsed = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
      output.latency_ms = static_cast<float>(elapsed);

      armors_publisher_->publish(output);
      std_msgs::msg::Int32 count;
      count.data = static_cast<int32_t>(detections.size());
      count_publisher_->publish(count);

      if (publish_debug_image_)
      {
        std::ostringstream status;
        status << "tracker=" << tracker_state
               << " preview=" << (target_locked ? "LOCK" : "UNLOCK")
               << " output=" << (actuation_allowed && enable_control_output_ ? "ACTIVE" : "SAFE")
               << " fire=" << fire_command;
        cv::putText(
            debug_image, status.str(), cv::Point(20, 36), cv::FONT_HERSHEY_SIMPLEX,
            0.8, cv::Scalar(0, 255, 255), 2);
        auto debug_message = cv_bridge::CvImage(
                                 message->header, sensor_msgs::image_encodings::BGR8, debug_image)
                                 .toImageMsg();
        debug_publisher_->publish(*debug_message);
      }

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "detections=%zu valid_poses=%zu tracker=%s latency=%.1f ms "
          "camera_info=%s vision_fresh=%s control=%s fire=%d",
          detections.size(), valid_pose_count, tracker_state.c_str(), elapsed,
          camera_model.valid ? "yes" : "no",
          vision.fresh ? "yes" : "no",
          actuation_allowed && enable_control_output_ ? "active" : "safe",
          fire_command);
    }

    std::unique_ptr<auto_aim::YOLO> detector_;
    std::unique_ptr<auto_aim::Solver> solver_;
    std::unique_ptr<auto_aim::Tracker> tracker_;
    std::unique_ptr<auto_aim::Aimer> aimer_;
    bool publish_debug_image_ = true;
    bool enable_control_output_ = false;
    bool require_auto_aim_mode_ = true;
    int frame_count_ = 0;
    double small_armor_width_m_ = 0.135;
    double big_armor_width_m_ = 0.230;
    double armor_height_m_ = 0.056;
    double max_pnp_reprojection_error_px_ = 8.0;
    double vision_timeout_s_ = 0.2;
    double max_control_yaw_error_rad_ = 15.0 * CV_PI / 180.0;
    double max_control_pitch_error_rad_ = 10.0 * CV_PI / 180.0;
    double max_control_step_rad_ = 1.0 * CV_PI / 180.0;
    double control_target_filter_alpha_ = 0.25;
    double control_yaw_deadband_rad_ = 0.25 * CV_PI / 180.0;
    double control_pitch_deadband_rad_ = 0.20 * CV_PI / 180.0;
    bool filtered_target_valid_ = false;
    double filtered_target_yaw_rad_ = 0.0;
    double filtered_target_pitch_rad_ = 0.0;
    bool control_hold_valid_ = false;
    bool yaw_in_deadband_ = false;
    bool pitch_in_deadband_ = false;
    double held_control_yaw_rad_ = 0.0;
    double held_control_pitch_rad_ = 0.0;
    bool enable_fire_ = false;
    double fire_hold_s_ = 0.15;
    double fire_interval_s_ = 0.5;
    double fire_shot_period_s_ = 0.1;
    int fire_burst_count_ = 3;
    double fire_yaw_tolerance_rad_ = 0.50 * CV_PI / 180.0;
    double fire_pitch_tolerance_rad_ = 0.50 * CV_PI / 180.0;
    double fire_max_yaw_velocity_deg_s_ = 5.0;
    double fire_max_pitch_velocity_deg_s_ = 5.0;
    double fire_max_reprojection_error_px_ = 3.0;
    std::unique_ptr<FireController> fire_controller_;

    std::mutex camera_mutex_;
    CameraModel camera_model_;
    bool has_live_camera_info_ = false;

    std::mutex vision_mutex_;
    bool has_vision_ = false;
    auto_aim_interfaces::msg::Vision latest_vision_;
    std::chrono::steady_clock::time_point latest_vision_received_at_{};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Vision>::SharedPtr vision_subscription_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr
        control_preview_publisher_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr control_publisher_;
  };

} // namespace auto_aim_ros2

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<auto_aim_ros2::AutoAimNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("auto_aim_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
