//
// Created by ocean_cing on 2026/2/9.
//

#include "rm2_chassis_controllers/chassis_base.h"
#include <angles/angles/angles.h>
#include <rm2_common/robot_state_manager.h>

#include <chrono>

namespace rm2_chassis_controllers
{
controller_interface::CallbackReturn ChassisBase::on_init()
{
  // getNode
  // param declare on init
  // param readding kept on configeration
  try
  {
    publish_rate_ = get_node()->declare_parameter<double>("publish_rate", 100.0);
    publish_map_tf_ = get_node()->declare_parameter<bool>("publish_map_tf", false);
    publish_odom_tf_ = get_node()->declare_parameter<bool>("publish_odom_tf", false);
    slam_topic_ = get_node()->declare_parameter<std::string>("slam_topic", "");
    localization_topic_ = get_node()->declare_parameter<std::string>("localization_topic", "");

    velocity_coeff_ = get_node()->declare_parameter<double>("power.vel_coeff", 0.0);
    effort_coeff_ = get_node()->declare_parameter<double>("power.effort_coeff", 0.0);
    power_offset_ = get_node()->declare_parameter<double>("power.power_offset", 0.0);

    wheel_radius_ = get_node()->declare_parameter<double>("wheel_radius", 0.0);
    twist_angular_ = get_node()->declare_parameter<double>("twist_angular", M_PI / 6);
    max_odom_vel_ = get_node()->declare_parameter<double>("max_odom_vel", 10);
    timeout_ = get_node()->declare_parameter<double>("timeout", 0.1);

    enable_uphill_acceleration_ = get_node()->declare_parameter<bool>("enable_uphill_acceleration", false);
    if (enable_uphill_acceleration_)
    {
      pitch_angle_threshold_ = get_node()->declare_parameter<double>("pitch_angle_threshold", -0.25);
      scale_ = get_node()->declare_parameter<double>("scale", 1.0);
    }

  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Chassis parameters initialization failed: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisBase::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Should make sure the initialization of pid_follow?
  pid_follow_ = std::make_shared<control_toolbox::PidROS>(get_node(), "pid_follow");
  pid_follow_->initialize_from_ros_parameters();

  // How to assign Qos?
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1),
    [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg)
    {
      this->cmdVelCallback(msg);
    });

  cmd_chassis_sub_ = get_node()->create_subscription<rm2_msgs::msg::ChassisCmd>("/cmd_chassis", rclcpp::QoS(1),
    [this](const rm2_msgs::msg::ChassisCmd::ConstSharedPtr msg)
    {
      this->cmdChassisCallback(msg);
    });

  slam_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(slam_topic_, rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
      this->slamCallback(msg);
    });

  localization_sub_ = get_node()->create_subscription<geometry_msgs::msg::TransformStamped>(localization_topic_, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
    {
      this->localizationCallback(msg);
    });

  ramp_x_ = std::make_unique<RampFilter<double>>(0, 0.001);
  ramp_y_ = std::make_unique<RampFilter<double>>(0, 0.001);
  ramp_w_ = std::make_unique<RampFilter<double>>(0, 0.001);

  cmd_struct_.stamp_ns = get_node()->get_clock()->now().nanoseconds();

  if (publish_map_tf_)
  {
    // Whether we use node time or rclcpp time?
    global_map2robot_odom_.header.stamp = get_node()->get_clock()->now();
    global_map2robot_odom_.header.frame_id = global_map_frame_id_;
    global_map2robot_odom_.child_frame_id = robot_odom_frame_id_;
    global_map2robot_odom_.transform.rotation.w = 1.0;
    brcst4global_map2robot_odom_.init(get_node());
    brcst4global_map2robot_odom_.sendTransform(global_map2robot_odom_);

    global_map2camera_init_.header.stamp = get_node()->get_clock()->now();
    global_map2camera_init_.header.frame_id = global_map_frame_id_;
    global_map2camera_init_.child_frame_id = "camera_init";
    global_map2camera_init_.transform.rotation.w = 1.0;
    brcst4global_map2camera_init_.init(get_node());
    brcst4global_map2camera_init_.sendTransform(global_map2camera_init_);
  }

  if (publish_odom_tf_)
  {
    robot_odom2robot_base_.header.stamp = get_node()->get_clock()->now();
    RCLCPP_INFO(get_node()->get_logger(), "type: %d", get_node()->get_clock()->get_clock_type());
    robot_odom2robot_base_.header.frame_id = robot_odom_frame_id_;
    robot_odom2robot_base_.child_frame_id = robot_base_frame_id_;
    global_map2robot_odom_.transform.rotation.w = 1.0;
    brcst4robot_odom2robot_base_.init(get_node());
    brcst4robot_odom2robot_base_.sendTransform(robot_odom2robot_base_);

  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisBase::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (auto& cmd : command_interfaces_)
  {
    if (!cmd.set_value(0.0))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface in (namespace: %s) cannot set value", get_node()->get_name());
      return CallbackReturn::ERROR;
    }
  }

  // I think it could be optimized
  auto tf_buffer = rm2_common::RobotStateManager::instance().getBuffer();
  robot_state_handle_ = rm2_control::RobotStateHandle("robot_state", tf_buffer.get());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisBase::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (auto& cmd : command_interfaces_)
  {
    if (!cmd.set_value(0.0))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface in (namespace: %s) cannot set value", get_node()->get_name());
      return CallbackReturn::ERROR;
    }
  }

  pid_follow_->reset();
  ramp_x_->clear();
  ramp_y_->clear();
  ramp_w_->clear();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ChassisBase::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  if (!last_publish_time_initialized_)
  {
    last_publish_time_ = time;
    last_publish_time_initialized_ = true;
  }

  // How to update?
  rm2_msgs::msg::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis;
  geometry_msgs::msg::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel;

  auto stamp = rclcpp::Time(cmd_rt_buffer_.readFromRT()->stamp_ns, time.get_clock_type());

  if ((time - stamp).seconds() > timeout_)
  {
    vel_cmd_.x = 0.;
    vel_cmd_.y = 0.;
    vel_cmd_.z = 0.;
  }
  else
  {
    ramp_x_->setAcc(cmd_chassis.accel.linear.x);
    ramp_y_->setAcc(cmd_chassis.accel.linear.y);
    ramp_x_->input(cmd_vel.linear.x);
    ramp_y_->input(cmd_vel.linear.y);
    vel_cmd_.x = ramp_x_->output();
    vel_cmd_.y = ramp_y_->output();
    vel_cmd_.z = cmd_vel.angular.z;
  }

  if (cmd_rt_buffer_.readFromRT()->cmd_chassis.follow_source_frame.empty())
  {
    follow_source_frame_ = "yaw";
  }
  else
  {
    follow_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis.follow_source_frame;
  }
  if (cmd_rt_buffer_.readFromRT()->cmd_chassis.command_source_frame.empty())
  {
    command_source_frame_ = "yaw";
  }
  else
  {
    command_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis.command_source_frame;
  }

  if (state_ != cmd_chassis.mode)
  {
    state_ = cmd_chassis.mode;
    state_changed_ = true;
  }

  updateOdom(time, period);

  switch (state_)
  {
  case RAW:
    raw();
    break;
  case FOLLOW:
    follow(time, period);
    break;
  case TWIST:
    twist(time, period);
    break;
  default:
    RCLCPP_WARN(get_node()->get_logger(), "[Chassis] Unknown state");
    break;
  }

  ramp_w_->setAcc(cmd_chassis.accel.angular.z);
  ramp_w_->input(vel_cmd_.z);
  vel_cmd_.z = ramp_w_->output();

  moveJoint(time, period);
  powerLimit();

  auto end_time = std::chrono::high_resolution_clock::now();
  uint64_t execution_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  
  static auto last_print_time = std::chrono::high_resolution_clock::now();
  static uint64_t total_execution_time = 0;
  static uint32_t execution_count = 0;
  static uint64_t max_execution_time = 0;

  total_execution_time += execution_time;
  execution_count++;
  if (execution_time > max_execution_time)
  {
    max_execution_time = execution_time;
  }

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - last_print_time).count();
  if (elapsed >= 1000)
  {
    RCLCPP_INFO(get_node()->get_logger(), "[Chassis] Real-Time Stats (1s): Freq = %u Hz, Avg Time = %lu us, Max Time = %lu us",
                execution_count, total_execution_time / execution_count, max_execution_time);
    
    execution_count = 0;
    total_execution_time = 0;
    max_execution_time = 0;
    last_print_time = end_time;
  }

  return controller_interface::return_type::OK;
}

void ChassisBase::raw()
{
  if (state_changed_)
  {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Chassis] Enter RAW");

    recovery();
  }
  tfVelToBase(command_source_frame_);
}

void ChassisBase::follow(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Chassis] Enter FOLLOW");

    recovery();
    pid_follow_->reset();
  }
  tfVelToBase(command_source_frame_);

  try
  {
    double roll, pitch, yaw;
    quatToRPY(robot_state_handle_.lookupTransform(
      robot_base_frame_id_, follow_source_frame_, rclcpp::Time(0)).transform.rotation,
      roll, pitch, yaw);
    double follow_error = angles::shortest_angular_distance(yaw, 0);
    pid_follow_->compute_command(-follow_error, period);
    vel_cmd_.z = pid_follow_->get_current_cmd() + cmd_rt_buffer_.readFromRT()->cmd_chassis.follow_vel_des;
  }
  catch (tf2::TransformException& ex)
  {
    // RCLCPP_WARN(get_node()->get_logger(), "%s", ex.what());
  }
}

void ChassisBase::twist(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Chassis] Enter TWIST");

    recovery();
    pid_follow_->reset();
  }
  tfVelToBase(command_source_frame_);

  try
  {
    double roll, pitch, yaw;
    quatToRPY(robot_state_handle_.lookupTransform(
      robot_base_frame_id_, follow_source_frame_, rclcpp::Time(0)).transform.rotation,
      roll, pitch, yaw);
    double angle[4] = { -0.785, 0.785, 2.355, -2.355 };
    double off_set = 0.0;
    for (double i : angle)
    {
      if (std::abs(angles::shortest_angular_distance(yaw, i)) < 0.79)
      {
        off_set = i;
        break;
      }
    }
    double follow_error =
      angles::shortest_angular_distance(yaw, twist_angular_ * sin(2 * M_PI * time.seconds()) + off_set);

    pid_follow_->compute_command(-follow_error, period);  // The actual output is opposite to the calculated value
    vel_cmd_.z = pid_follow_->get_current_cmd();
  }
  catch (tf2::TransformException& ex)
  {
    //RCLCPP_WARN(get_node()->get_logger(), "%s", ex.what());
  }
}

void ChassisBase::updateOdom(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (!std::isfinite(period.seconds()) || period.seconds() <= 0.0)
  {
    return;
  }
  if (publish_map_tf_)
  {
    if (!odom_initialized_)
    {
      try
      {
        geometry_msgs::msg::TransformStamped global_map2lidar_odom =
          robot_state_handle_.lookupTransform(robot_base_frame_id_, lidar_base_frame_id_, rclcpp::Time(0));
        T_global_map2lidar_odom_.setOrigin(tf2::Vector3(global_map2lidar_odom.transform.translation.x,
                                                        global_map2lidar_odom.transform.translation.y,
                                                        global_map2lidar_odom.transform.translation.z));
        T_global_map2lidar_odom_.setRotation(tf2::Quaternion(global_map2lidar_odom.transform.rotation.x,
                                                               global_map2lidar_odom.transform.rotation.y,
                                                               global_map2lidar_odom.transform.rotation.z,
                                                               global_map2lidar_odom.transform.rotation.w));
        odom_initialized_ = true;
        global_map2camera_init_.transform = global_map2lidar_odom.transform;
      }
      catch (...)
      {
        //RCLCPP_WARN(get_node()->get_logger(), "Failed to init robot_odom2lidar_odom.");
      }
    }

    if (localization_updated_)
    {
      try
      {
        localization_updated_ = false;
        const auto& localization = localization_rt_buffer_.readFromRT();
        T_global_map2lidar_odom_.setOrigin(tf2::Vector3(localization->transform.translation.x,
                                                        localization->transform.translation.y,
                                                        localization->transform.translation.z));
        T_global_map2lidar_odom_.setRotation(tf2::Quaternion(localization->transform.rotation.x,
                                                               localization->transform.rotation.y,
                                                               localization->transform.rotation.z,
                                                               localization->transform.rotation.w));
        global_map2camera_init_.transform = tf2::toMsg(T_global_map2lidar_odom_);
      }
      catch (...)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to update localization offset.");
      }
    }

    if (slam_updated_)
    {
      try
      {
        slam_updated_ = false;
        const auto& slam = slam_rt_buffer_.readFromRT();
        T_lidar_odom2lidar_base_.setOrigin(tf2::Vector3(slam->pose.pose.position.x,
                                                        slam->pose.pose.position.y,
                                                        slam->pose.pose.position.z));
        T_lidar_odom2lidar_base_.setRotation(tf2::Quaternion(slam->pose.pose.orientation.x,
                                                               slam->pose.pose.orientation.y,
                                                               slam->pose.pose.orientation.z,
                                                               slam->pose.pose.orientation.w));

        robot_base2lidar_base_ =
          robot_state_handle_.lookupTransform(robot_base_frame_id_, lidar_base_frame_id_, rclcpp::Time(0));
        T_robot_base2lidar_base_.setOrigin(tf2::Vector3(robot_base2lidar_base_.transform.translation.x,
                                                        robot_base2lidar_base_.transform.translation.y,
                                                        robot_base2lidar_base_.transform.translation.z));
        T_robot_base2lidar_base_.setRotation(tf2::Quaternion(robot_base2lidar_base_.transform.rotation.x,
                                                               robot_base2lidar_base_.transform.rotation.y,
                                                               robot_base2lidar_base_.transform.rotation.z,
                                                               robot_base2lidar_base_.transform.rotation.w));

        T_robot_odom_2robot_base_.setOrigin(tf2::Vector3(robot_odom2robot_base_.transform.translation.x,
                                                         robot_odom2robot_base_.transform.translation.y,
                                                         robot_odom2robot_base_.transform.translation.z));
        T_robot_odom_2robot_base_.setRotation(tf2::Quaternion(robot_odom2robot_base_.transform.rotation.x,
                                                                robot_odom2robot_base_.transform.rotation.y,
                                                                robot_odom2robot_base_.transform.rotation.z,
                                                                robot_odom2robot_base_.transform.rotation.w));

        T_global_map2robot_odom_ = T_global_map2lidar_odom_ * T_lidar_odom2lidar_base_ *
                                   T_robot_base2lidar_base_.inverse() * T_robot_odom_2robot_base_.inverse();

        global_map2robot_odom_.transform = tf2::toMsg(T_global_map2robot_odom_);
      }
      catch (...)
      {
        //RCLCPP_WARN(get_node()->get_logger(), "Failed to update global_map2robot_odom.");
      }
    }
    global_map2robot_odom_.header.stamp = time;
    global_map2camera_init_.header.stamp = time;
  }

  geometry_msgs::msg::Twist vel_base = odometry();  // on base_link frame
  geometry_msgs::msg::Vector3 linear_vel_odom, angular_vel_odom;

  try
  {
    robot_odom2robot_base_ =
      robot_state_handle_.lookupTransform(robot_odom_frame_id_, robot_base_frame_id_, rclcpp::Time(0));
    tf2::Quaternion q;
    tf2::fromMsg(robot_odom2robot_base_.transform.rotation, q);
    tf2::Matrix3x3(q).getEulerYPR(yaw_, pitch_, roll_);
  }
  catch (tf2::TransformException& ex)
  {
    if (publish_odom_tf_)
    {
      brcst4robot_odom2robot_base_.sendTransform(robot_odom2robot_base_);  // For some reason, the sendTransform in init sometime not work
    }
    RCLCPP_WARN(get_node()->get_logger(), "%s", ex.what());
    return;
  }

  robot_odom2robot_base_.header.stamp = time;

  // integral vel to pos and angle
  tf2::doTransform(vel_base.linear, linear_vel_odom, robot_odom2robot_base_);
  tf2::doTransform(vel_base.angular, angular_vel_odom, robot_odom2robot_base_);

  double length =
    std::sqrt(std::pow(linear_vel_odom.x, 2) + std::pow(linear_vel_odom.y, 2) + std::pow(linear_vel_odom.z, 2));
  if (length < max_odom_vel_)
  {
    // avoid nan vel
    robot_odom2robot_base_.transform.translation.x += linear_vel_odom.x * period.seconds();
    robot_odom2robot_base_.transform.translation.y += linear_vel_odom.y * period.seconds();
    robot_odom2robot_base_.transform.translation.z += linear_vel_odom.z * period.seconds();
  }
  
  length = std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
  if (length > 0.001)
  {
    // avoid nan quat
    tf2::Quaternion odom2base_quat, trans_quat;
    tf2::fromMsg(robot_odom2robot_base_.transform.rotation, odom2base_quat);
    trans_quat.setRotation(tf2::Vector3(angular_vel_odom.x / length,
                                            angular_vel_odom.y / length,
                                            angular_vel_odom.z / length),
                                         length * period.seconds());
    odom2base_quat = trans_quat * odom2base_quat;
    odom2base_quat.normalize();
    robot_odom2robot_base_.transform.rotation = tf2::toMsg(odom2base_quat);
  }

  if (!robot_state_handle_.setTransform(robot_odom2robot_base_, "rm2_chassis_controllers"))
  {
    //RCLCPP_WARN(get_node()->get_logger(), "Failed to set transform.");
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + rclcpp::Duration::from_seconds(1.0 / publish_rate_) < time)
  {
    if (publish_map_tf_)
    {
      brcst4global_map2robot_odom_.sendTransform(global_map2robot_odom_);
      brcst4global_map2camera_init_.sendTransform(global_map2camera_init_);
    }

    if (publish_odom_tf_)
    {
      brcst4robot_odom2robot_base_.sendTransform(robot_odom2robot_base_);
    }

    last_publish_time_ = time;
  }
}

void ChassisBase::recovery()
{
  ramp_x_->clear(vel_cmd_.x);
  ramp_y_->clear(vel_cmd_.y);
  ramp_w_->clear(vel_cmd_.z);
}

void ChassisBase::tfVelToBase(const std::string& from)
{
  try
  {
   tf2::doTransform(vel_cmd_, vel_cmd_, robot_state_handle_.lookupTransform("base_link", from, rclcpp::Time(0)));
  }
  catch (tf2::TransformException& ex)
  {
    //RCLCPP_WARN(get_node()->get_logger() ,"%s", ex.what());
  }
}

void ChassisBase::powerLimit()
{
  if (!power_limit_joints_) return;
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis.power_limit;
  // Three coefficients of a quadratic equation in one variable
  double a = 0., b = 0., c = 0.;
  // Whether we must use get_optional()?
  for (const auto& joint : *power_limit_joints_)
  {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    a += square(cmd_effort);
    b += std::abs(cmd_effort * real_vel);
    c += square(real_vel);
  }
  a *= effort_coeff_;
  c = c * velocity_coeff_ - power_offset_ - power_limit;
  // Root formula for quadratic equation in one variable
  double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;

  for (auto& joint : *power_limit_joints_)
  {
    if (pitch_ < pitch_angle_threshold_ && enable_uphill_acceleration_)
    {

      if (joint.getName().find("back") != std::string::npos)
      {
        (void)joint.setCommand(zoom_coeff > 1 ?
          joint.getCommand() :
          joint.getCommand() * zoom_coeff * scale_);
      }
      if (joint.getName().find("front") != std::string::npos)
      {
        (void)joint.setCommand(zoom_coeff > 1 ?
          joint.getCommand() :
          joint.getCommand() * zoom_coeff * scale_);
      }
    }
    else
    {
      (void)joint.setCommand(zoom_coeff > 1 ?
        joint.getCommand() :
        joint.getCommand() * zoom_coeff * scale_);
    }
  }
}

void ChassisBase::cmdChassisCallback(const rm2_msgs::msg::ChassisCmd::ConstSharedPtr msg)
{
  cmd_struct_.cmd_chassis = *msg;
  cmd_struct_.stamp_ns = get_node()->get_clock()->now().nanoseconds();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void ChassisBase::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  cmd_struct_.cmd_vel = *msg;
  cmd_struct_.stamp_ns = get_node()->get_clock()->now().nanoseconds();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void ChassisBase::slamCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  slam_rt_buffer_.writeFromNonRT(*msg);
  slam_updated_ = true;
}

void ChassisBase::localizationCallback(const geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
  localization_rt_buffer_.writeFromNonRT(*msg);
  localization_updated_ = true;
}

} // namespace rm2_chassis_controllers