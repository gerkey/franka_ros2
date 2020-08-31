#include "franka_hw/franka_hw.hpp"

namespace franka_hw {

hardware_interface::return_type
FrankaHW::init()
{
  auto joint_names = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7"
  };
  size_t i = 0;
  for (auto & joint_name : joint_names) {
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    joint_state_handles_[i] = state_handle;
    if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::return_type::OK) {
      throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
    }

    hardware_interface::JointCommandHandle command_handle(joint_name, &cmd_[i]);
    joint_command_handles_[i] = command_handle;
    if (register_joint_command_handle(&joint_command_handles_[i]) !=
      hardware_interface::return_type::OK)
    {
      throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
    }
    ++i;
  }
  return(hardware_interface::return_type::OK);
}

hardware_interface::return_type
FrankaHW::read()
{
  // do robot specific stuff to update the pos_, vel_, eff_ arrays
  return(hardware_interface::return_type::OK);
}

hardware_interface::return_type
FrankaHW::write()
{
  // do robot specific stuff to apply the command values from cmd_ to the robot
  return(hardware_interface::return_type::OK);
}

}  // namespace franka_hw
