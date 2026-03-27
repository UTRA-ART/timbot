/*
 * 2023-04-29
 * gets ticks per second from each wheel and publishes linear and angular velocity
 * from https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
 * note: tutorial is in Melodic
 * 
 * subscribed to:
 *   right_wheel/ticks (ticks since last, Int32)
 *   left_wheel/ticks
 *   right_wheel/direction (bool)
 *   left_wheel/direction
 *   rover_pose/set (geometry_msgs/PoseStamped)
 *   rover_pose/reset (bool)
 * 
 * publishes to:
 *   odom (quaternion)
 * 
 * resources
 *   covariance matrix: https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/
 *   motion model: https://www.roboticsbook.org/S52_diffdrive_actions.html
 *  
 * 2024-04-10
 * fix direction issue - direction retrieved from command messages and
 * and combined with directionless ticks/s from hall effect sensors
 * 
 * 2024-05-20
 * subscribe to wheel/ticks instead, not necessarily ticks_ps
 * 
 * 2024-06-01
 * subscribe to rover_pose to reset position without relaunching
 * 
 * 2026-02-28
 * ROS2 Port
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <sstream>

using namespace std;
using std::placeholders::_1;

class WheelOdomPub : public rclcpp::Node
{
public:
    WheelOdomPub() : Node("wheel_odom_pub")
    {
        // set data fields of odometry message
        odom_new_.header.frame_id = "odom";
        odom_new_.child_frame_id = "base_link";
        odom_new_.pose.pose.position.z = 0;
        odom_new_.pose.pose.orientation.x = 0;
        odom_new_.pose.pose.orientation.y = 0;
        odom_new_.twist.twist.linear.x = 0;
        odom_new_.twist.twist.linear.y = 0;
        odom_new_.twist.twist.linear.z = 0;
        odom_new_.twist.twist.angular.x = 0;
        odom_new_.twist.twist.angular.y = 0;
        odom_new_.twist.twist.angular.z = 0;
        odom_old_.pose.pose.position.x = INITIAL_X;
        odom_old_.pose.pose.position.y = INITIAL_Y;
        odom_old_.pose.pose.orientation.z = INITIAL_THETA;

        // publishers
        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 100);    // full odom message, orientation as quaternion
        debug_pub_ = this->create_publisher<std_msgs::msg::String>(
            "debug_wheel_odom", 100);

        // subscribers
        // ticks per second from both wheels
        right_ticks_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/right_wheel/ticks", 100,
            std::bind(&WheelOdomPub::right_ticks_cb, this, _1));
        left_ticks_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/left_wheel/ticks", 100,
            std::bind(&WheelOdomPub::left_ticks_cb, this, _1));
        // wheel commands to get direction for both wheels
        right_dir_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/right_wheel/direction", 100,
            std::bind(&WheelOdomPub::right_direction_cb, this, _1));
        left_dir_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/left_wheel/direction", 100,
            std::bind(&WheelOdomPub::left_direction_cb, this, _1));
        // force set rover position
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "rover_pose/set", 1,
            std::bind(&WheelOdomPub::set_pose_cb, this, _1));
        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "rover_pose/reset", 1,
            std::bind(&WheelOdomPub::reset_pose_cb, this, _1));

        // timer at 30 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&WheelOdomPub::timer_callback, this));
    }

private:
    // constants
    static constexpr double INITIAL_X = 0.0;
    static constexpr double INITIAL_Y = 0.0;
    static constexpr double INITIAL_THETA = 0.00000000001;
    static constexpr double PI = 3.1415926;

    // Approximately 9.8inches = 24.892cm diameter (estimate)
    static constexpr double WHEEL_RADIUS = 0.125; // (in metres)
    static constexpr double CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
    static constexpr double WHEEL_BASE = 0.69; // (centre of left tire to centre of right tire)

    // defines slope of rpm/(ticks per second) reading, with intercept set to 0
    static constexpr double A = 0.3114;
    static constexpr double TICKS_PER_METRE = 1.0 / A / CIRCUMFERENCE * 60.0;

    // member variables
    nav_msgs::msg::Odometry odom_new_;
    nav_msgs::msg::Odometry odom_old_;

    // distance both wheels have travelled
    double distance_left_ = 0;
    double distance_right_ = 0;

    // ticks for each wheel
    float ticks_left_ = 0;
    float ticks_right_ = 0;

    // direction for each wheel
    int l_direction_ = 0;
    int r_direction_ = 0;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

    // subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_dir_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_dir_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // --- callbacks ---

    void right_ticks_cb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        ticks_right_ = msg->data;
        distance_right_ = ticks_right_ / TICKS_PER_METRE;
    }

    void left_ticks_cb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        ticks_left_ = msg->data;
        distance_left_ = ticks_left_ / TICKS_PER_METRE;
    }

    void left_direction_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        l_direction_ = msg->data ? 1 : -1;
    }

    void right_direction_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        r_direction_ = msg->data ? 1 : -1;
    }

    void set_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        odom_old_.pose.pose.position.x = msg->pose.position.x;
        odom_old_.pose.pose.position.y = msg->pose.position.y;
        odom_old_.pose.pose.orientation.z = msg->pose.orientation.z;
    }

    void reset_pose_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            odom_old_.pose.pose.position.x = 0;
            odom_old_.pose.pose.position.y = 0;
            odom_old_.pose.pose.orientation.z = 0;
        }
    }

    // --- timer callback (replaces while loop) ---

    void timer_callback()
    {
        update_odom();
        publish_quat();
        RCLCPP_DEBUG(this->get_logger(), "left_dir: %d\tright_dir: %d",
                     l_direction_, r_direction_);
    }

    // --- odometry update ---

    void update_odom()
    {
        // average distance since last cycle
        double cycle_distance = ((r_direction_ * distance_right_) + (l_direction_ * distance_left_)) / 2;
        // number of radians the robot has turned since the last cycle
        double cycle_angle = asin(((l_direction_ * distance_left_) - (r_direction_ * distance_right_)) / WHEEL_BASE);
        // average angle during the last cycle
        double avg_angle = cycle_angle / 2 + odom_old_.pose.pose.orientation.z;

        if (avg_angle > PI) {
            avg_angle -= 2 * PI;
        } else if (avg_angle < -PI) {
            avg_angle += 2 * PI;
        }

        // calculate new pose
        odom_new_.pose.pose.position.x = odom_old_.pose.pose.position.x + cos(avg_angle) * cycle_distance;
        odom_new_.pose.pose.position.y = odom_old_.pose.pose.position.y + sin(avg_angle) * cycle_distance;
        odom_new_.pose.pose.orientation.z = cycle_angle + odom_old_.pose.pose.orientation.z;

        // prevent lockup from a single bad cycle
        if (isnan(odom_new_.pose.pose.position.x) || isnan(odom_new_.pose.pose.position.y)
            || isnan(odom_new_.pose.pose.position.z)) {
            odom_new_.pose.pose.position.x = odom_old_.pose.pose.position.x;
            odom_new_.pose.pose.position.y = odom_old_.pose.pose.position.y;
            odom_new_.pose.pose.orientation.z = odom_old_.pose.pose.orientation.z;
        }

        // ensure theta stays in the correct range
        if (odom_new_.pose.pose.orientation.z > PI) {
            odom_new_.pose.pose.orientation.z -= 2 * PI;
        } else if (odom_new_.pose.pose.orientation.z < -PI) {
            odom_new_.pose.pose.orientation.z += 2 * PI;
        }

        // compute velocity
        odom_new_.header.stamp = this->now();
        double dt = (rclcpp::Time(odom_new_.header.stamp) - rclcpp::Time(odom_old_.header.stamp)).seconds();
        if (dt > 0.0) {
            odom_new_.twist.twist.linear.x = cycle_distance / dt;
            odom_new_.twist.twist.angular.z = cycle_angle / dt;
        }

        // save pose data for next cycle
        odom_old_.pose.pose.position.x = odom_new_.pose.pose.position.x;
        odom_old_.pose.pose.position.y = odom_new_.pose.pose.position.y;
        odom_old_.pose.pose.orientation.z = odom_new_.pose.pose.orientation.z;
        odom_old_.header.stamp = odom_new_.header.stamp;

        // publish is handled in publish_quat()
    }

    // --- publish quaternion odometry ---

    void publish_quat()
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_new_.pose.pose.orientation.z);

        nav_msgs::msg::Odometry quat_odom;
        quat_odom.header.stamp = odom_new_.header.stamp;
        quat_odom.header.frame_id = "odom";
        quat_odom.child_frame_id = "base_link";
        quat_odom.pose.pose.position.x = odom_new_.pose.pose.position.x;
        quat_odom.pose.pose.position.y = odom_new_.pose.pose.position.y;
        quat_odom.pose.pose.position.z = odom_new_.pose.pose.position.z;
        quat_odom.pose.pose.orientation.x = q.x();
        quat_odom.pose.pose.orientation.y = q.y();
        quat_odom.pose.pose.orientation.z = q.z();
        quat_odom.pose.pose.orientation.w = q.w();
        quat_odom.twist.twist.linear.x = odom_new_.twist.twist.linear.x;
        quat_odom.twist.twist.linear.y = odom_new_.twist.twist.linear.y;
        quat_odom.twist.twist.linear.z = odom_new_.twist.twist.linear.z;
        quat_odom.twist.twist.angular.x = odom_new_.twist.twist.angular.x;
        quat_odom.twist.twist.angular.y = odom_new_.twist.twist.angular.y;
        quat_odom.twist.twist.angular.z = odom_new_.twist.twist.angular.z;

        // build covariance matrix (use big number if unsure of uncertainty)
        for (int i = 0; i < 36; i++) {
            if (i == 0 || i == 7 || i == 14) {
                quat_odom.pose.covariance[i] = 0.01;    // translation accuracy +/- 0.01 m
            } else if (i == 21 || i == 28 || i == 35) {
                quat_odom.pose.covariance[i] = 0.1;     // rotation accuracy +/- 0.1 radian
            } else {
                quat_odom.pose.covariance[i] = 0;
            }
        }

        odom_data_pub_->publish(quat_odom);
    }
};

// main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdomPub>());
    rclcpp::shutdown();
    return 0;
}