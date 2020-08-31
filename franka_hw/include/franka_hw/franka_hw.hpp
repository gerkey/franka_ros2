#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <rclcpp/node.hpp>

namespace franka_hw {

class FrankaHW : public hardware_interface::RobotHardware {
 public:
  virtual hardware_interface::return_type init();
  virtual hardware_interface::return_type read();
  virtual hardware_interface::return_type write();
 private:
  static const int NUM_JOINTS = 7;
  double pos_[NUM_JOINTS];
  double vel_[NUM_JOINTS];
  double eff_[NUM_JOINTS];
  hardware_interface::JointStateHandle joint_state_handles_[NUM_JOINTS];
  double cmd_[NUM_JOINTS];
  hardware_interface::JointCommandHandle joint_command_handles_[NUM_JOINTS];
};

}  // namespace franka_hw
