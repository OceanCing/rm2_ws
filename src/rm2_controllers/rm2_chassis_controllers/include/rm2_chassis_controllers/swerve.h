//
// Created by ocean_cing on 2026/2/12.
//

#pragma once

#include "rm2_chassis_controllers/chassis_base.h"

#include <rm2_common/eigen_types.h>
#include <rm2_msgs/msg/power_heat_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rm2_chassis_controllers
{
class SwerveController : public ChassisBase
{
private:
  struct ModuleGroup
  {
    std::vector<Vec2<double>> position;
    std::vector<double> pivot_offset, pivot_buffer_threshold, pivot_effort_threshold, pivot_position_error_threshold,
      pivot_max_reduce_cnt, wheel_radius;
    joint_manager::JointManager pivot_joints;
    joint_manager::JointManager wheel_joints;
    size_t size;
  };
  std::vector<std::shared_ptr<control_toolbox::PidROS>> pivot_pids_;
  std::vector<std::shared_ptr<control_toolbox::PidROS>> wheel_pids_;
public:
  SwerveController() = default;
  hardware_interface::CallbackReturn on_init() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  void moveJoint(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override;
  void powerManagerCallback(const rm2_msgs::msg::PowerHeatData::ConstSharedPtr data);
  bool isPivotBlock(const double& cur_effort, const double& position_error, const ModuleGroup& module_group, const size_t& index);
  void reduceTargetPosition(double& target_pos, const double& position_error, const ModuleGroup& module_group, const size_t& index);
  geometry_msgs::msg::Twist odometry() override;

  ModuleGroup modules_;
  rclcpp::Subscription<rm2_msgs::msg::PowerHeatData>::SharedPtr power_manager_sub_;
  int chassis_power_buffer_ = 60;
  int pivot_block_cnt_ = 0;
};

} // namespace rm2_chassis_controllers

