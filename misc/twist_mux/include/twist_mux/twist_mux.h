/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *********************************************************************/

 #ifndef TWIST_MUX_H
 #define TWIST_MUX_H
 
 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/bool.hpp>
 #include <geometry_msgs/msg/twist.hpp>
 
 #include <twist_mux/utils.h>
 #include <twist_mux/twist_mux_diagnostics_status.h>  // Include full definition
 
 #include <list>
 #include <memory>
 #include <string>
 
 namespace twist_mux
 {
 
 // Forward declarations:
 class TwistMuxDiagnostics;
 class VelocityTopicHandle;
 class LockTopicHandle;
 
 /**
  * @brief The TwistMux class implements a top-level twist multiplexer module
  * that prioritizes different velocity command topic inputs according to locks.
  */
 class TwistMux : public rclcpp::Node
 {
 public:
 
   // Modern C++ type aliases (C++11 and later)
   using velocity_topic_container = std::list<VelocityTopicHandle>;
   using lock_topic_container = std::list<LockTopicHandle>;
 
   explicit TwistMux(const std::string& node_name = "twist_mux");
   virtual ~TwistMux();  // Remove = default
 
   bool hasPriority(const VelocityTopicHandle& twist);
 
   void publishTwist(const geometry_msgs::msg::Twist::SharedPtr& msg);
 
   void updateDiagnostics();
 
 protected:
 
   using diagnostics_type = TwistMuxDiagnostics;
   using status_type = TwistMuxDiagnosticsStatus;
 
   rclcpp::TimerBase::SharedPtr diagnostics_timer_;
 
   static constexpr double DIAGNOSTICS_PERIOD = 1.0;
 
   /**
    * @brief velocity_hs_ Velocity topics' handles.
    */
   std::shared_ptr<velocity_topic_container> velocity_hs_;
   std::shared_ptr<lock_topic_container>     lock_hs_;
 
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
 
   geometry_msgs::msg::Twist last_cmd_;
 
   template<typename T>
   void getTopicHandles(const std::string& param_name, std::list<T>& topic_hs);
 
   int getLockPriority();
 
   std::shared_ptr<diagnostics_type> diagnostics_;
   std::shared_ptr<status_type>      status_;
 
 private:
   // Helper method for parsing topic configuration
   void parseTopicConfig(const std::string& config_str, std::string& name, 
                         std::string& topic, double& timeout, int& priority);
 };
 
 } // namespace twist_mux
 
 #endif // TWIST_MUX_H