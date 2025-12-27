#include <twist_mux/twist_mux.h>
#include <twist_mux/twist_mux_diagnostics.h>
#include <twist_mux/twist_mux_diagnostics_status.h>
#include <twist_mux/utils.h>
#include <twist_mux/xmlrpc_helpers.h>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_twist Old velocity
 * @param new_twist New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const geometry_msgs::msg::Twist& old_twist, const geometry_msgs::msg::Twist& new_twist)
{
  const auto old_linear_x = std::abs(old_twist.linear.x);
  const auto new_linear_x = std::abs(new_twist.linear.x);

  const auto old_angular_z = std::abs(old_twist.angular.z);
  const auto new_angular_z = std::abs(new_twist.angular.z);

  return (old_linear_x  < new_linear_x ) or
         (old_angular_z < new_angular_z);
}

namespace twist_mux
{

// Fix constructor signature to match header
TwistMux::TwistMux(const std::string& node_name)
  : Node(node_name)
{
  /// Get topics and locks:
  velocity_hs_ = std::make_shared<velocity_topic_container>();
  lock_hs_     = std::make_shared<lock_topic_container>();
  getTopicHandles("topics", *velocity_hs_);
  getTopicHandles("locks", *lock_hs_);

  /// Publisher for output topic:
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 1);

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>(shared_from_this());
  status_      = std::make_shared<status_type>();
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs     = lock_hs_;

  /// Timer for diagnostics:
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(DIAGNOSTICS_PERIOD * 1000)),
    std::bind(&TwistMux::updateDiagnostics, this)
  );
}

TwistMux::~TwistMux()
{}

void TwistMux::updateDiagnostics()
{
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

// Fix method signature to match header
void TwistMux::publishTwist(const geometry_msgs::msg::Twist::SharedPtr& msg)
{
  cmd_pub_->publish(*msg);
}

template<typename T>
void TwistMux::getTopicHandles(const std::string& param_name, std::list<T>& topic_hs)
{
  try
  {
    // ROS 2 parameter handling
    this->declare_parameter(param_name, rclcpp::ParameterValue(rclcpp::PARAMETER_NOT_SET));
    
    rclcpp::Parameter param = this->get_parameter(param_name);
    
    if (param.get_type() == rclcpp::PARAMETER_NOT_SET)
    {
      RCLCPP_WARN(this->get_logger(), "Parameter '%s' not found", param_name.c_str());
      return;
    }

    // For ROS 2, we'll need to adapt the parameter parsing
    // This is a simplified version - you may need to adjust based on your parameter structure
    auto param_array = param.as_string_array();
    
    for (const auto& param_str : param_array)
    {
      // Parse individual topic configuration from string
      // This will need to be adapted based on your parameter format
      std::string name, topic;
      double timeout;
      int priority;
      
      // Example parsing - adjust as needed
      parseTopicConfig(param_str, name, topic, timeout, priority);
      
      topic_hs.emplace_back(shared_from_this(), name, topic, timeout, priority, this);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(), "Error parsing params: %s", e.what());
  }
}

void TwistMux::parseTopicConfig(const std::string& config_str, std::string& name, 
                                std::string& topic, double& timeout, int& priority)
{
  // This is a placeholder - implement based on your parameter format
  // Example format: "name:topic:timeout:priority"
  std::istringstream iss(config_str);
  std::string token;
  
  if (std::getline(iss, name, ':') &&
      std::getline(iss, topic, ':') &&
      std::getline(iss, token, ':'))
  {
    timeout = std::stod(token);
    if (std::getline(iss, token, ':'))
    {
      priority = std::stoi(token);
    }
  }
}

int TwistMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto& lock_h : *lock_hs_)
  {
    if (lock_h.isLocked())
    {
      auto tmp = lock_h.getPriority();
      if (priority < tmp)
      {
        priority = tmp;
      }
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Priority = %d", static_cast<int>(priority));

  return priority;
}

bool TwistMux::hasPriority(const VelocityTopicHandle& twist)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string velocity_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto& velocity_h : *velocity_hs_)
  {
    if (not velocity_h.isMasked(lock_priority))
    {
      const auto velocity_priority = velocity_h.getPriority();
      if (priority < velocity_priority)
      {
        priority = velocity_priority;
        velocity_name = velocity_h.getName();
      }
    }
  }

  return twist.getName() == velocity_name;
}

} // namespace twist_mux