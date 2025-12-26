#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// Define the different states for ramp navigation
enum State { no_ramp, to_ramp, on_ramp };

class RampNavigateNode : public rclcpp::Node {
 public:
  using NavigateToPoseAction = nav2_msgs::action::NavigateToPose; 
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

  RampNavigateNode() : Node("RampNavigateNode"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
    state = no_ramp;
    ramps_to_cross = 1;

    ac = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "/move_base");

    ramp_seg_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ramp_seg", 10,
        std::bind(&RampNavigateNode::rampFrontCallback, this,
                  std::placeholders::_1));

    ramp_naving_pub =
        this->create_publisher<std_msgs::msg::Bool>("/ramp_naving", 5);
    ramp_routine_pub =
        this->create_publisher<std_msgs::msg::Bool>("/ramp_routine", 5);
  }

 private:
  // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac; <-- Find
  // ROS2 equivalent for Action Client
  int ramps_to_cross;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr ac;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ramp_seg_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ramp_naving_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ramp_routine_pub;

  State state;
  int pre_ramp_detections = 0;
  int no_ramp_period =
      0;  // How many times pre_ramp_detections is NOT incremented
  float slope, xmid, ymid, px, py;

  Eigen::Matrix2d ramp2map;
  tf2_ros::TransformListener tfListener;
  tf2_ros::Buffer tfBuffer;
  // tf::TransformListener tfListener;

  void rampFrontCallback(
      const geometry_msgs::msg::PoseArray::SharedPtr ramp_seg) {
    // Migrate ramp navigation logic here
    if (state == on_ramp || ramps_to_cross <= 0) {
      return;
    }

    // If the length of ramp segment is not within expected range, do not proceed
    // Also make sure the ramp is detected for more than one time point (confirm it's actually there)
    if (!pass_length(ramp_seg->poses)) {
      if (pre_ramp_detections > 0) {
        no_ramp_period += 1;
        if (no_ramp_period > 3) {
          pre_ramp_detections = 0;
          no_ramp_period = 0;
        }
      }
      return;
    }
    pre_ramp_detections += 1;

    // Obtain base_link -> map transform
    geometry_msgs::msg::TransformStamped transform = tfBuffer.lookupTransform("map", "base_link", tf2::TimePointZero);
    tf2::Transform tfTransform;
    tf2::fromMsg(transform.transform, tfTransform);
    const auto& caff = tfTransform.getOrigin();

    // Find the middle point in front of ramp
    const auto& front = ramp_seg->poses.front().position;
    const auto& back = ramp_seg->poses.back().position;
    const float x_len = back.x - front.x;
    const float y_len = back.y - front.y;
    const float len = sqrt(x_len*x_len + y_len*y_len); // Length of detected ramp segment

    Eigen::Matrix2d ramp2map_;
    ramp2map_ << y_len, x_len,
                -x_len, y_len;
    ramp2map_ = ramp2map_ / len;
    Eigen::Vector2d mid(-1, 0.5 * len);
    Eigen::Vector2d midmap = ramp2map * mid + Eigen::Vector2d(front.x, front.y);

    // Make the goal closer to ramp until within proximity
    // to mitigate  error caused by calculating the front of ramp to be too far away
    const float goal_dist2 = (midmap[0] - caff.x())*(midmap[0] - caff.x()) + (midmap[1] - caff.y())*(midmap[1] - caff.y());
    if (goal_dist2 > 3.0*3.0) {
      midmap = ramp2map * Eigen::Vector2d(0, 0.5 * len) + Eigen::Vector2d(front.x, front.y);
    }

    const float mvavg = 0.5;
    const float mvavg_st = 1 - mvavg;

    ramp2map = ramp2map * mvavg_st + mvavg * ramp2map_;
    xmid = xmid * mvavg_st + mvavg * midmap[0];
    ymid = ymid * mvavg_st + mvavg * midmap[1];

    // Goal x, y in map frame
    px = xmid;
    py = ymid;

    if (state == no_ramp) {
      if (pre_ramp_detections < 10) {
        return;
      } else {
        state = to_ramp;
        pre_ramp_detections = 0;
        std_msgs::msg::Bool naving_msg;
        naving_msg.data = true;
        ramp_naving_pub->publish(naving_msg); // Send message that we are current in ramp navigation mode
    }
  }

  // Set a move base goal for the middle front of the ramp
  auto goal_msg = NavigateToPoseAction::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = px;
  goal_msg.pose.pose.position.y = py;
  goal_msg.pose.pose.orientation.w = 1.0;

  ac->wait_for_action_server();

  auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
  send_goal_options.feedback_callback = 
      std::bind(&RampNavigateNode::feedback_callback, this, 
                std::placeholders::_1, std::placeholders::_2);
  
  ac->async_send_goal(goal_msg, send_goal_options);

  const float goalerror2 = (px - caff.x()) * (px - caff.x()) + (py - caff.y()) * (py - caff.y());
  if (goalerror2 < 2.0) {
    RCLCPP_INFO(this->get_logger(), "ON RAMP: Initiating ramp crossing");
    state = on_ramp;
    cross(goal_msg, xmid, ymid, ramp2map, send_goal_options); // Continue rest of navigation across ramp

    std_msgs::msg::Bool naving_msg;
    naving_msg.data = false;
    ramp_naving_pub->publish(naving_msg);
    ramps_to_cross -= 1;
  }

}

void cross(NavigateToPoseAction::Goal goal, const float xmid, const float ymid, const Eigen::Matrix2d ramp2map, rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions send_goal_options) {
    std_msgs::msg::Bool is_on_ramp;
    is_on_ramp.data = true;
    ramp_routine_pub->publish(is_on_ramp); // Send message that we are currently crossing ramp
        
    const float ramp_traverse_dist = 8; // Total distance to traverse 
    const int traverse_count = 8; // How many goal points to set along the ramp
    const Eigen::Vector2d incr = ramp2map * Eigen::Vector2d(ramp_traverse_dist / traverse_count, 0); // Make it a tiny bit past the ramp

    float px = xmid;
    float py = ymid;

    // Set goals at repeated small increments across ramp
    for (int i = 0; i < traverse_count; i += 1) {
      //goal.target_pose.header.stamp = ros::Time::now();
      px += incr[0];
      py += incr[1];
      goal.pose.pose.position.x = px;
      goal.pose.pose.position.y = py;

      ac->async_send_goal(goal, send_goal_options);

      // wait for the result, this might not be correct
      // std::shared_future<rclcpp::client_goal_handle::WrappedResult> result = ac.async_get_result();
      // result.wait();
    }
        
    RCLCPP_INFO(this->get_logger(), "Finished Ramp Crossing");
    state = no_ramp; // After we finish crossing ramp
  }

  bool pass_length(const std::vector<geometry_msgs::msg::Pose>& seg) {
    const auto& front = seg.front().position;
    const auto& back = seg.back().position;
    const float dx = front.x - back.x;
    const float dy = front.y - back.y;
    const float dz = front.z - back.z;
    const float incline_len2 = dx*dx + dy*dy + dz*dz;
    const float min_len = 2.5;
    const float max_len = 4;
    return incline_len2 >= min_len*min_len && incline_len2 <= max_len*max_len;
  }

  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
  }
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RampNavigateNode>());
  rclcpp::shutdown();
  return 0;
}