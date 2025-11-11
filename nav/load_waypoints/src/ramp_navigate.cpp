#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>


// Define the different states for ramp navigation
enum State { 
  no_ramp, 
  to_ramp, 
  on_ramp 
};

class RampNavigateNode: public rclcpp::Node
{
 public:
 RampNavigateNode() : Node("RampNavigateNode"), count_(0) {
    state = no_ramp;
    ramps_to_cross = 1;

    ramp_seg_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ramp_seg", 10,
        std::bind(&RampNavigateNode::rampFrontCallback, this, std::placeholders::_1));
    
    ramp_naving_pub = this->create_publisher<std_msgs::msg::Bool>("/ramp_naving", 5);
    ramp_routine_pub = this->create_publisher<std_msgs::msg::Bool>("/ramp_routine", 5);

  }

 private:

  // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac; <-- Find ROS2 equivalent for Action Client
  int ramps_to_cross;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ramp_seg_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ramp_naving_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ramp_routine_pub;


  State state;
  int pre_ramp_detections = 0;
  int no_ramp_period = 0; // How many times pre_ramp_detections is NOT incremented
  float slope, xmid, ymid, px, py;

  // Eigen::Matrix2d ramp2map;
  // tf::TransformListener tfListener;

  void rampFrontCallback(const geometry_msgs::msg::PoseArray::SharedPtr ramp_seg) {
    // Migrate ramp navigation logic here
    return;
  }
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RampNavigateNode>());
  rclcpp::shutdown();
  return 0;
}