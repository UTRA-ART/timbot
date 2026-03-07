#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <time.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
using std::placeholders::_1;

nav_msgs::msg::Odometry odom_new;
nav_msgs::msg::Odometry odom_old;

std_msgs::msg::String debug_msg;
std::stringstream ss;

// timestamps
time_t last;
time_t current;
// duration to calculate distance
long int duration = 0;

// initial pose
const double initial_x = 0.0;
const double initial_y = 0.0;
const double initial_theta = 0.00000000001;
const double PI = 3.1415926;

// Approximately 9.8inches = 24.892cm diameter (estimate)
const double WHEEL_RADIUS = 0.125;  // (in metres)
const double CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
const double WHEEL_BASE =
    0.69;  // (centre of left tire to centre of right tire)

// defines slope of rpm/(ticks per second) reading, with intercept set to 0
const double A = 0.3114;
const double TICKS_PER_METRE = 1 / A / CIRCUMFERENCE * 60;

// distance both wheels have travelled
double distance_left = 0;
double distance_right = 0;

// ticks per second for each wheel
float ticks_left = 0;
float ticks_right = 0;

// direction for each wheel
int l_direction = 0;
int r_direction = 0;

// wheel velocitiess
double rpm_right = 0;
double rpm_left = 0;
double vel_right = 0;
double vel_left = 0;

// has initial pose been received?
bool initial_pose_received = false;

using namespace std;

class Odom_Pub2 : public rclcpp::Node {
 public:
  Odom_Pub2() : Node("odom_pub2") {
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "wheel_odom/euler",
            100);  // simple odom message, orientation.z is an euler angle
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "wheel_odom/quant",
            100);  // full odom message, orientation.z is quaternion

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub =
        this->create_publisher<std_msgs::msg::String>("debug_wheel_odom", 100);

    // ticks per second from both wheels
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_vel_sub =
        this->create_subscription<std_msgs::msg::Int32>(
            "/right_wheel/ticks", 100,
            std::bind(&Odom_Pub2::right_ticks_cb, this, _1));
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_vel_sub =
        this->create_subscription<std_msgs::msg::Int32>(
            "/left_wheel/ticks", 100,
            std::bind(&Odom_Pub2::left_ticks_cb, this, _1));
    // wheel commands to get direction for both wheels
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr r_vel =
        this->create_subscription<std_msgs::msg::Bool>(
            "/right_wheel/direction", 100,
            std::bind(&Odom_Pub2::right_direction_cb, this, _1));
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr l_vel =
        this->create_subscription<std_msgs::msg::Bool>(
            "/left_wheel/direction", 100,
            std::bind(&Odom_Pub2::left_direction_cb, this, _1));
    // force set rover position
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "rover_pose/set", 1, std::bind(&Odom_Pub2::set_pose_cb, this, _1));
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub =
        this->create_subscription<std_msgs::msg::Bool>(
            "rover_pose/reset", 1,
            std::bind(&Odom_Pub2::reset_pose_cb, this, _1));
  }

  // publish odom_new as nav_msgs/odom message in quaternion form
  void publish_quat() {
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_new.pose.pose.orientation.z);

    nav_msgs::msg::Odometry quat_odom;
    quat_odom.header.stamp = odom_new.header.stamp;
    quat_odom.header.frame_id = "odom";
    quat_odom.child_frame_id = "base_link";
    quat_odom.pose.pose.position.x = odom_new.pose.pose.position.x;
    quat_odom.pose.pose.position.y = odom_new.pose.pose.position.y;
    quat_odom.pose.pose.position.z = odom_new.pose.pose.position.z;
    quat_odom.pose.pose.orientation.x = q.x();
    quat_odom.pose.pose.orientation.y = q.y();
    quat_odom.pose.pose.orientation.z = q.z();
    quat_odom.pose.pose.orientation.w = q.w();
    quat_odom.twist.twist.linear.x = odom_new.twist.twist.linear.x;
    quat_odom.twist.twist.linear.y = odom_new.twist.twist.linear.y;
    quat_odom.twist.twist.linear.z = odom_new.twist.twist.linear.z;
    quat_odom.twist.twist.angular.x = odom_new.twist.twist.angular.x;
    quat_odom.twist.twist.angular.y = odom_new.twist.twist.angular.y;
    quat_odom.twist.twist.angular.z = odom_new.twist.twist.angular.z;

    // build covariance matrix (use big number if unsure of uncertainty)
    for (int i = 0; i < 36; i++) {
      if (i == 0 || i == 7 || i == 14) {
        quat_odom.pose.covariance[i] = 0.01;  // translation accuracy +/- 0.01 m
      } else if (i == 21 || i == 28 || i == 35) {
        quat_odom.pose.covariance[i] +=
            0.1;  // rotation accuracy +/- 0.1 radian
      } else {
        quat_odom.pose.covariance[i] = 0;
      }
    }

    odom_data_pub_quat->publish(quat_odom);
  }

  // update odometry information
  void update_odom() {
    // average distance since last cycle
    double cycle_distance =
        ((r_direction * distance_right) + (l_direction * distance_left)) / 2;
    // number of radians the robot has turned since the last cycle
    double cycle_angle =
        asin(((l_direction * distance_left) - (r_direction * distance_right)) /
             WHEEL_BASE);
    // average angle during the last cycle
    double avg_angle = cycle_angle / 2 + odom_old.pose.pose.orientation.z;

    if (avg_angle > PI) {
      avg_angle -= 2 * PI;
    } else if (avg_angle < -PI) {
      avg_angle += 2 * PI;
    }

    // calculate new pose
    odom_new.pose.pose.position.x =
        odom_old.pose.pose.position.x + cos(avg_angle) * cycle_distance;
    odom_new.pose.pose.position.y =
        odom_old.pose.pose.position.y + sin(avg_angle) * cycle_distance;
    odom_new.pose.pose.orientation.z =
        cycle_angle + odom_old.pose.pose.orientation.z;

    // prevent lockup from a single bad cycle
    if (isnan(odom_new.pose.pose.position.x) ||
        isnan(odom_new.pose.pose.position.y) ||
        isnan(odom_new.pose.pose.position.z)) {
      odom_new.pose.pose.position.x = odom_old.pose.pose.position.x;
      odom_new.pose.pose.position.y = odom_old.pose.pose.position.y;
      odom_new.pose.pose.orientation.z = odom_old.pose.pose.orientation.z;
    }

    // ensure theta stays in the correct range
    if (odom_new.pose.pose.orientation.z > PI) {
      odom_new.pose.pose.orientation.z -= 2 * PI;
    } else if (odom_new.pose.pose.orientation.z < -PI) {
      odom_new.pose.pose.orientation.z += 2 * PI;
    }

    // compute velocity
    rclcpp::Time new_time = this->get_clock()->now();
    odom_new.header.stamp = new_time;
    rclcpp::Time old_time(odom_old.header.stamp);
    odom_new.twist.twist.linear.x =
        cycle_distance / (new_time - old_time).seconds();
    odom_new.twist.twist.angular.z =
        cycle_angle / (new_time - old_time).seconds();
    // save pose data for next cycle
    odom_old.pose.pose.position.x = odom_new.pose.pose.position.x;
    odom_old.pose.pose.position.y = odom_new.pose.pose.position.y;
    odom_old.pose.pose.orientation.z = odom_new.pose.pose.orientation.z;
    odom_old.header.stamp = odom_new.header.stamp;

    // publish odometry message
    odom_data_pub->publish(odom_new);
  }

 private:
  // get current ticks/second from each wheel
  void right_ticks_cb(const std_msgs::msg::Int32::SharedPtr right_ticks) {
    // ticks from sensor since last message for right wheel
    ticks_right = right_ticks->data;
    // convert ticks to metres per second
    distance_right = ticks_right / TICKS_PER_METRE;
  }

  void left_ticks_cb(const std_msgs::msg::Int32::SharedPtr left_ticks) {
    // ticks from sensor since last message for left wheel
    ticks_left = left_ticks->data;
    distance_left = ticks_left / TICKS_PER_METRE;
  }

  // get the direction (positive or negative) from each wheel
  void left_direction_cb(const std_msgs::msg::Bool::SharedPtr left_dir_msg) {
    if (left_dir_msg->data) {
      l_direction = 1;
    } else {
      l_direction = -1;
    }
  }

  void right_direction_cb(const std_msgs::msg::Bool::SharedPtr right_dir_msg) {
    if (right_dir_msg->data) {
      r_direction = 1;
    } else {
      r_direction = -1;
    }
  }

  void set_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    odom_old.pose.pose.position.x = pose_msg->pose.position.x;
    odom_old.pose.pose.position.y = pose_msg->pose.position.y;
    odom_old.pose.pose.orientation.z = pose_msg->pose.orientation.z;
  }

  void reset_pose_cb(const std_msgs::msg::Bool::SharedPtr reset_msg) {
    if (reset_msg->data) {
      odom_old.pose.pose.position.x = 0;
      odom_old.pose.pose.position.y = 0;
      odom_old.pose.pose.orientation.z = 0;
    }
  }
};

// main
int main(int argc, char** argv) {
  // set data fields of odometry message
  odom_new.header.frame_id = "odom";
  odom_new.child_frame_id = "base_link";
  odom_new.pose.pose.position.z = 0;
  odom_new.pose.pose.orientation.x = 0;
  odom_new.pose.pose.orientation.y = 0;
  odom_new.twist.twist.linear.x = 0;
  odom_new.twist.twist.linear.y = 0;
  odom_new.twist.twist.linear.z = 0;
  odom_new.twist.twist.angular.x = 0;
  odom_new.twist.twist.angular.y = 0;
  odom_new.twist.twist.angular.z = 0;
  odom_old.pose.pose.position.x = initial_x;
  odom_old.pose.pose.position.y = initial_y;
  odom_old.pose.pose.orientation.z = initial_theta;

  // initialise ros node
  rclcpp::init(argc, argv);
  auto odom_node = std::make_shared<Odom_Pub2>();
  odom_old.header.stamp = odom_node->get_clock()->now();

  // initial timestamp? not sure how to initialize the time, time taken mostly
  // in loop?
  last = time(NULL);
  current = last;

  rclcpp::Rate loop_rate(30);

  while (rclcpp::ok()) {
    // distance is updated in update_odom
    // skip the update if either time function fail and return -1
    if (last == -1 || current == -1) {
    } else {
      odom_node->update_odom();
    }

    odom_node->publish_quat();

    rclcpp::spin_some(odom_node);
    loop_rate.sleep();
    // initial time becomes last time
    last = current;
    // last time is updated
    current = time(NULL);
    // duration is the difference, used in next loop for update_odom
    // typecast to integer in order to use in calculations
    duration = static_cast<long int>(current - last);
  }

  rclcpp::shutdown();

  return 0;
}