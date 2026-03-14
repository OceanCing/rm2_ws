//
// Created by ocean_cing on 2026/2/12.
//

#pragma once

#include "rm2_chassis_controllers/chassis_base.h"
#include <control_toolbox/pid_ros.hpp>

namespace rm2_chassis_controllers
{
class OmniController : public ChassisBase
{
public:
  OmniController() = default;
  hardware_interface::CallbackReturn on_init() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
private:
  void moveJoint(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override;
  geometry_msgs::msg::Twist odometry() override;

  joint_manager::JointManager joint_manager_;
  std::vector<std::shared_ptr<control_toolbox::PidROS>> pids_;
  double K = 0.;  // Feedforward gain
  Eigen::MatrixXd chassis2joints_;
};

} // namespace rm2_chassis_controllers