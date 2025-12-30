/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *********************************************************************/

 #ifndef TOPIC_HANDLE_H
 #define TOPIC_HANDLE_H
 
 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/bool.hpp>
 #include <geometry_msgs/msg/twist.hpp>
 
 #include <twist_mux/utils.h>  // This should be first for clamp function
 
 #include <memory>
 #include <string>
 #include <vector>
 
 namespace twist_mux
 {
 
 // Forward declaration
 class TwistMux;
 
 template<typename T>
 class TopicHandle_
 {
 public:
   typedef int priority_type;
 
   TopicHandle_(rclcpp::Node::SharedPtr node, const std::string& name, const std::string& topic, 
                double timeout, priority_type priority, TwistMux* mux)
     : node_(node)
     , name_(name)
     , topic_(topic)
     , timeout_(timeout)
     , priority_(twist_mux::clamp(priority, priority_type(0), priority_type(255)))  // Use namespace
     , mux_(mux)
     , stamp_(0, 0, RCL_ROS_TIME)
   {
     RCLCPP_INFO_STREAM(node_->get_logger(),
       "Topic handler '" << name_ << "' subscribed to topic '" << topic_ <<
       "': timeout = " << ((timeout_) ? std::to_string(timeout_) + "s" : "None") <<
       ", priority = " << static_cast<int>(priority_)
     );
   }
 
   virtual ~TopicHandle_() = default;
 
   bool hasExpired() const
   {
     return (timeout_ > 0.0) and
            ((node_->get_clock()->now() - stamp_).seconds() > timeout_);
   }
 
   const std::string& getName() const { return name_; }
   const std::string& getTopic() const { return topic_; }
   const double& getTimeout() const { return timeout_; }
   const priority_type& getPriority() const { return priority_; }
   const rclcpp::Time& getStamp() const { return stamp_; }
   const T& getMessage() const { return msg_; }
 
 protected:
   rclcpp::Node::SharedPtr node_;
   std::string name_;
   std::string topic_;
   typename rclcpp::Subscription<T>::SharedPtr subscriber_;
   double timeout_;
   priority_type priority_;
   TwistMux* mux_;
   rclcpp::Time stamp_;
   T msg_;
 };
 
 class VelocityTopicHandle : public TopicHandle_<geometry_msgs::msg::Twist>
 {
 public:
   VelocityTopicHandle(rclcpp::Node::SharedPtr node, const std::string& name, 
                       const std::string& topic, double timeout, priority_type priority, TwistMux* mux);
 
   bool isMasked(priority_type lock_priority) const;
   void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
 };
 
 class LockTopicHandle : public TopicHandle_<std_msgs::msg::Bool>
 {
 public:
   LockTopicHandle(rclcpp::Node::SharedPtr node, const std::string& name, 
                   const std::string& topic, double timeout, priority_type priority, TwistMux* mux);
 
   bool isLocked() const;
   void callback(const std_msgs::msg::Bool::SharedPtr msg);
 };
 
 } // namespace twist_mux
 
 #endif // TOPIC_HANDLE_H