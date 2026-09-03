#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "serial_main.h"
// #include "robot_status.h"
// #include "robot_struct.h"
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"

namespace rm_auto_aim
{

class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_ctrl", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    subscription_ = this->create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data", 10,
      std::bind(&RobotCtrlSub::robotCtrlSend, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "-- RobotCtrlSub Node Started --");
  }

private:
  // void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::SharedPtr msg)
  // void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl &msg) const 
  void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr& msg)
  {
		constexpr float RAD_TO_DEG = 57.29577951308232F;
		if (!std::isfinite(msg->yaw) || !std::isfinite(msg->yaw_vel) ||
			!std::isfinite(msg->yaw_acc) || !std::isfinite(msg->pitch) ||
			!std::isfinite(msg->pitch_vel) || !std::isfinite(msg->pitch_acc)) {
			RCLCPP_ERROR_THROTTLE(
				this->get_logger(), *this->get_clock(), 1000,
				"Rejected non-finite robot control command");
			return;
		}

		io::RobotCtrlData data;
		// ROS auto-aim commands are radians.  The Hero MCU serial protocol,
		// like the team's original gimbal_hj implementation, expects degrees.
		data.yaw = msg->yaw * RAD_TO_DEG;
		data.yaw_vel = msg->yaw_vel * RAD_TO_DEG;
		data.yaw_acc = msg->yaw_acc * RAD_TO_DEG;
		data.pitch = msg->pitch * RAD_TO_DEG;
		data.pitch_vel = msg->pitch_vel * RAD_TO_DEG;
		data.pitch_acc = msg->pitch_acc * RAD_TO_DEG;
		const auto requested_lock = static_cast<int>(msg->target_lock);
		const bool locked = requested_lock == 1 || requested_lock == io::TARGET_LOCKED;
		data.target_lock = locked ? io::TARGET_LOCKED : io::TARGET_UNLOCKED;
		data.fire_command = locked && msg->fire_command ? 1 : 0;
		if (locked) {
			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), 500,
				"Serial control: yaw=%.2f deg pitch=%.2f deg lock=%d fire=%d",
				data.yaw, data.pitch, static_cast<int>(data.target_lock),
				static_cast<int>(data.fire_command));
		}
		if (!serial.SenderMain(data)) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(), *this->get_clock(), 1000, "Failed to send robot control frame");
		}
    // std::cout<<"---------- ROBOT CTRL SEND ----  "<<" yaw: "<<(double)msg->yaw<<std::endl;
  }
//   void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr& msg)
// {
//   RCLCPP_INFO(this->get_logger(), "--- into robotCtrlSend ---");
//   // RCLCPP_INFO(this->get_logger(), "[RECV] vx: %.2f, vy: %.2f, vw: %.2f", msg->vx, msg->vy, msg->vw);
//   // RCLCPP_INFO(this->get_logger(), "[RECV] yaw: %.2f, pitch: %.2f", msg->yaw, msg->pitch);
//   // RCLCPP_INFO(this->get_logger(), "[RECV] fire: %d, lock: %d", msg->fire_command, msg->target_lock);
// }

  SerialMain serial;
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr subscription_;
};


}  // namespace rm_auto_aim
// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rm_auto_aim::RobotCtrlSub>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)
