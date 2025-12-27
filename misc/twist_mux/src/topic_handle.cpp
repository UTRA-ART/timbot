#include <twist_mux/twist_mux_diagnostics_status.h>
#include <twist_mux/twist_mux.h>
#include <algorithm>

namespace twist_mux
{

// VelocityTopicHandle implementation
VelocityTopicHandle::VelocityTopicHandle(rclcpp::Node::SharedPtr node, const std::string& name, 
                                         const std::string& topic, double timeout, priority_type priority, TwistMux* mux)
  : TopicHandle_(node, name, topic, timeout, priority, mux)
{
  subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    topic, 1, 
    std::bind(&VelocityTopicHandle::callback, this, std::placeholders::_1)
  );
}

bool VelocityTopicHandle::isMasked(priority_type lock_priority) const
{
  return lock_priority >= priority_;
}

void VelocityTopicHandle::callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  msg_ = *msg;
  stamp_ = node_->get_clock()->now();
  
  // Check if this topic has priority and publish if so
  if (mux_ && mux_->hasPriority(*this))
  {
    mux_->publishTwist(msg);
  }
}

// LockTopicHandle implementation
LockTopicHandle::LockTopicHandle(rclcpp::Node::SharedPtr node, const std::string& name, 
                                 const std::string& topic, double timeout, priority_type priority, TwistMux* mux)
  : TopicHandle_(node, name, topic, timeout, priority, mux)
{
  subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic, 1,
    std::bind(&LockTopicHandle::callback, this, std::placeholders::_1)
  );
}

bool LockTopicHandle::isLocked() const
{
  if (hasExpired())
  {
    return false;
  }
  
  return msg_.data;
}

void LockTopicHandle::callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  msg_ = *msg;
  stamp_ = node_->get_clock()->now();
}

} // namespace twist_mux