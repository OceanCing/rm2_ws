//
// Created by ocean_cing on 2026/2/9.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_component.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rm2_common/hardware_handle/robot_state_handle.h>
#include <rm2_common/filters/filters.h>
#include <rm2_common/math_utilities.h>
#include <rm2_common/ori_tool.h>
#include <rm2_msgs/msg/chassis_cmd.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <unordered_map>
#include "rm2_common/joint_manager.h"

namespace rm2_chassis_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ChassisBase : public controller_interface::ControllerInterface
{
protected:
  struct Command
  {
    geometry_msgs::msg::Twist cmd_vel;
    rm2_msgs::msg::ChassisCmd cmd_chassis;
    int64_t stamp_ns;
  };

  // This struct has been deprecated
  struct JointGroup
  {
    std::vector<std::string> joint_names;
    std::vector<size_t> cmd_index;
    std::vector<size_t> pos_index;
    std::vector<size_t> vel_index;
    std::vector<size_t> eff_index;
    std::vector<std::shared_ptr<control_toolbox::PidROS>> pids;

    void reset()
    {
      cmd_index.clear();
      pos_index.clear();
      vel_index.clear();
      eff_index.clear();
    }

    void reserve(size_t n)
    {
      cmd_index.reserve(n);
      pos_index.reserve(n);
      vel_index.reserve(n);

      eff_index.reserve(n);
    }
  };

  enum
  {
    RAW,
    FOLLOW,
    TWIST
  };
public:
  ChassisBase() = default;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
protected:
  // this method has been deprecated
  template<typename T>
  std::unordered_map<std::string, size_t> buildInterfaceIndexMap(T&& interfaces) {
    std::unordered_map<std::string, size_t> interface_index_map;
    interface_index_map.reserve(interfaces.size());
    for (size_t i = 0; i < interfaces.size(); ++i) {
      interface_index_map[interfaces[i].get_name()] = i;
    }
    return interface_index_map;
  }
  
  // this method has been deprecated
  void buildJointsIndex(JointGroup& joints,const std::unordered_map<std::string, size_t>& command_map,
    const std::unordered_map<std::string, size_t>& state_map)
  {
    for (const auto& joint_name : joints.joint_names)
    {
      joints.cmd_index.push_back(command_map.at(joint_name + "/" + hardware_interface::HW_IF_EFFORT));
      joints.pos_index.push_back(state_map.at(joint_name + "/" + hardware_interface::HW_IF_POSITION));
      joints.vel_index.push_back(state_map.at(joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
      joints.eff_index.push_back(state_map.at(joint_name + "/" + hardware_interface::HW_IF_EFFORT));
    }
  }

  void buildJointsPids(JointGroup& joints)
  {
    joints.pids.clear();
    joints.pids.reserve(joints.joint_names.size());
    for (const auto& joint_name : joints.joint_names)
    {
      // Whether pid initialize in activate? Whether ptr use this API to initialize?
      auto pid = std::make_shared<control_toolbox::PidROS>(this->get_node(), joint_name + ".pid", "~/" + joint_name, false);
      pid->initialize_from_ros_parameters();
      joints.pids.push_back(pid);
    }
  }

  void raw();

  void follow(const rclcpp::Time& /*time*/, const rclcpp::Duration& period);

  void twist(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void moveJoint(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;

  virtual geometry_msgs::msg::Twist odometry() = 0;

  void updateOdom(const rclcpp::Time& time, const rclcpp::Duration& period);

  void recovery();

  void tfVelToBase(const std::string& from);

  void powerLimit();

  void cmdChassisCallback(const rm2_msgs::msg::ChassisCmd::ConstSharedPtr msg);

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void slamCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void localizationCallback(const geometry_msgs::msg::TransformStamped::ConstSharedPtr msg);

  /* void initialize_parameters();
   * The reading of parameters in ros2 is different, this function was no use in ros2
  */

  rm2_control::RobotStateHandle robot_state_handle_;
  /* This vector of interface will be filled by controller manager in active stage, not on_init or on_configure
   * And change effort_command_interface to command_interface_ to adapt to ROS2 structure
   * controller_interface_base already have this two member
   * std::vector<hardware_interface::LoanedCommandInterface> command_interface_;
   * std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
   */

  // JointGroup wheel_joints_;
  // JointGroup pivot_joints_;   // If the Omni chassis
  // JointGroup knee_joints_;  // If the Legged chassis
  // JointGroup hip_joints_;   // If the Legged chassis

  joint_manager::JointManager* power_limit_joints_{ nullptr };
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> slam_rt_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TransformStamped> localization_rt_buffer_;

  rm2_common::TfRtBroadcaster brcst4global_map2robot_odom_{};
  rm2_common::TfRtBroadcaster brcst4robot_odom2robot_base_{};
  rm2_common::TfRtBroadcaster brcst4global_map2camera_init_{};

  geometry_msgs::msg::TransformStamped global_map2robot_odom_{};
  geometry_msgs::msg::TransformStamped robot_odom2robot_base_{};
  geometry_msgs::msg::TransformStamped robot_base2lidar_base_{};
  geometry_msgs::msg::TransformStamped global_map2camera_init_{};

  tf2::Transform T_global_map2robot_odom_{};
  tf2::Transform T_robot_odom_2robot_base_{};
  tf2::Transform T_lidar_odom2lidar_base_{};
  tf2::Transform T_robot_base2lidar_base_{};
  tf2::Transform T_global_map2lidar_odom_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rm2_msgs::msg::ChassisCmd>::SharedPtr cmd_chassis_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr localization_sub_;

  std::unique_ptr<RampFilter<double>> ramp_x_{ nullptr };
  std::unique_ptr<RampFilter<double>> ramp_y_{ nullptr };
  std::unique_ptr<RampFilter<double>> ramp_w_{ nullptr };

  double publish_rate_{ 100.0 };
  bool publish_map_tf_{ false };
  bool publish_odom_tf_{ false };
  bool enable_uphill_acceleration_{ false };

  double velocity_coeff_{ 0.0 };
  double effort_coeff_{ 0.0 };
  double power_offset_{ 0.0 };
  double roll_ = 0., pitch_ = 0., yaw_ = 0.;
  double pitch_angle_threshold_ = 0., scale_ = 0.;

  double wheel_radius_{ 0.0 };
  double twist_angular_{ M_PI / 6 };
  double max_odom_vel_{ 10.0 };
  double timeout_{ 0.1 };

  bool odom_initialized_{ false };
  bool last_publish_time_initialized_{ false };
  bool slam_updated_{ false };
  bool localization_updated_{ false };
  bool state_changed_{ true };
  int state_{ RAW };

  std::string follow_source_frame_{};
  std::string command_source_frame_{};
  std::string global_map_frame_id_{ "map" };
  std::string robot_odom_frame_id_{ "odom" };
  std::string robot_base_frame_id_{ "base_link" };
  std::string lidar_base_frame_id_{ "livox_frame" };
  std::string slam_topic_{ "/Odometry" };
  std::string localization_topic_{ "/hdl_global_localization/result" };

  rclcpp::Time last_publish_time_{};
  geometry_msgs::msg::Vector3 vel_cmd_{};   // x, y

  Command cmd_struct_;
  std::shared_ptr<control_toolbox::PidROS> pid_follow_;
};

} // namespace rm2_chassis_controllers