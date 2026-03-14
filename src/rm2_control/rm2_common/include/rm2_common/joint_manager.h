//
// Created by Nesc on 2026/3/11.
//

#pragma once

#include <optional>
#include <controller_interface/controller_interface.hpp>
#include <unordered_map>

namespace joint_manager 
{
class JointHandle
{
public:
  explicit JointHandle(const std::string& name);
  ~JointHandle() = default;

  void bind_interfaces(
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos,
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel,
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort,
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd);

  void read();
  
  inline double getVelocity() const;
  inline double getPosition() const;
  inline double getEffort() const;
  inline double getCommand() const;
  inline std::string getName() const;
  
  void setCommand(double command);

private:
  std::string name_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_interface;
  double cached_pos = 0.0;
  double cached_vel = 0.0;
  double cached_effort = 0.0;
};

class JointManager
{
public:
  JointManager() = default;
  explicit JointManager(std::vector<std::string> names);
  ~JointManager() = default;

  void init(std::vector<std::string> names);
  void add_joint(std::string name);
  
  size_t size() const;
  std::vector<std::string> get_names() const;
  
  void read_all();
  void write_all(const std::vector<double>& commands);

  // todo: avoid copy
  // todo：make it more elegrant
  void bind_all(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces);

  std::vector<std::string> get_command_interface_names() const;
  std::vector<std::string> get_state_interface_names() const;
  
  auto begin() { return joints_.begin(); }
  auto end() { return joints_.end(); }
  auto begin() const { return joints_.begin(); }
  auto end() const { return joints_.end(); }

  JointHandle& operator[](size_t index);
  const JointHandle& operator[](size_t index) const;

private:
  std::vector<JointHandle> joints_;
  std::vector<std::string> allowed_command_types_ = {"effort"};
  std::vector<std::string> allowed_state_types_ = {"position", "velocity", "effort"};
  // Maybe we should use yaml to set allowed command types
};

inline JointHandle::JointHandle(const std::string& name) : name_(name) {}

inline void JointHandle::bind_interfaces(
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos,
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel,
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort,
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd) 
{
  pos_state = pos;
  vel_state = vel;
  effort_state = effort;
  cmd_interface = cmd;
}

inline void JointHandle::read() 
{
  if (pos_state) cached_pos = pos_state->get().get_value();
  if (vel_state) cached_vel = vel_state->get().get_value();
  if (effort_state) cached_effort = effort_state->get().get_value();
}

inline double JointHandle::getVelocity() const {return cached_vel;}
inline double JointHandle::getPosition() const {return cached_pos;}
inline double JointHandle::getEffort() const {return cached_effort;}
inline std::string JointHandle::getName() const {return name_;}

inline double JointHandle::getCommand() const
{
  if (cmd_interface)
  {
    return cmd_interface->get().get_value();
  }
  return 0.0;
}
 
inline void JointHandle::setCommand(double command)
{
  if (cmd_interface) 
  {
    cmd_interface->get().set_value(command);
  }
}

inline JointManager::JointManager(std::vector<std::string> names)
{
  for (const auto& name : names)
  {
    joints_.emplace_back(name);
  }
}

inline void JointManager::init(std::vector<std::string> names)
{
  joints_.clear();
  for (const auto& name : names)
  {
    joints_.emplace_back(name);
  }
}

inline void JointManager::add_joint(std::string name) {joints_.emplace_back(name);}

inline size_t JointManager::size() const { return joints_.size(); }

inline std::vector<std::string> JointManager::get_names() const {
  std::vector<std::string> names;
  names.reserve(joints_.size());
  for (const auto& joint : joints_) { names.push_back(joint.getName()); }
  return names;
}

inline void JointManager::read_all() {
  for (auto & joint : joints_) 
  {
    joint.read();
  }
}

inline void JointManager::write_all(const std::vector<double>& commands) 
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].setCommand(commands[i]);
  }
}

inline void JointManager::bind_all(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
              std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces)
{
  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_map;
  for (auto& it : state_interfaces) {
    state_map.emplace(it.get_name(), std::ref(it));
  }

  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_map;
  for (auto& it : command_interfaces) {
    cmd_map.emplace(it.get_name(), std::ref(it));
  }

  for (auto & joint : joints_) 
  {
    const std::string name = joint.getName();
    auto get_state = [&](const std::string& type) -> std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> {
      auto it = state_map.find(name + "/" + type);
      return (it != state_map.end()) ? std::make_optional(it->second) : std::nullopt;
    };

    auto get_cmd = [&](const std::string& type) -> std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> {
      auto it = cmd_map.find(name + "/" + type);
      return (it != cmd_map.end()) ? std::make_optional(it->second) : std::nullopt;
    };

    // todo yaml
    joint.bind_interfaces(
      get_state("position"),
      get_state("velocity"),
      get_state("effort"),
      get_cmd("effort")
    );
  }
}

inline std::vector<std::string> JointManager::get_command_interface_names() const
{
  std::vector<std::string> names;
  for (const auto & joint : joints_) 
  {
    for (const auto & type : allowed_command_types_) 
    {
      names.push_back(joint.getName() + "/" + type);
    }
  }
  return names;
}

inline std::vector<std::string> JointManager::get_state_interface_names() const 
{
  std::vector<std::string> names;
  for (const auto & joint : joints_) 
  {
    for (const auto & type : allowed_state_types_) 
    {
      names.push_back(joint.getName() + "/" + type);
    }
  }
  return names;
}

inline JointHandle& JointManager::operator[](size_t index) 
{
  return joints_.at(index); 
}

inline const JointHandle& JointManager::operator[](size_t index) const
{
  return joints_.at(index); 
}
}