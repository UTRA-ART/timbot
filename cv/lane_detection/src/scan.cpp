#include <memory>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lane_detection/msg/float_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class LaneScan : public rclcpp::Node
{
public:
    LaneScan() : Node("lane_scan")
    {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("lane_float_topic", "/cv/lane_detections");
        this->declare_parameter<std::string>("lane_laser_topic", "/cv/lane_detections_scan");
        this->declare_parameter<std::string>("lidar_laser_topic", "/scan_modified");
        this->declare_parameter<std::string>("lidar_laser_topic_out", "/scan_merged");

        lane_float_topic_ = this->get_parameter("lane_float_topic").as_string();
        lane_laser_topic_ = this->get_parameter("lane_laser_topic").as_string();
        lidar_laser_topic_ = this->get_parameter("lidar_laser_topic").as_string();
        lidar_laser_topic_out_ = this->get_parameter("lidar_laser_topic_out").as_string();

        // Setup TF buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Wait for transform (equivalent of ROS1 waitForTransform)
        {
            const std::string target = "bottom_lidar_link";
            const std::string source = "left_camera_link_optical";
            rclcpp::Time start = this->now();
            rclcpp::Duration timeout = rclcpp::Duration::from_seconds(4.0);

            while (!tf_buffer_->canTransform(target, source, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                if ((this->now() - start) > timeout) {
                    RCLCPP_WARN(this->get_logger(), "TF %s -> %s not available after 4s", source.c_str(), target.c_str());
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        // SUBSCRIBERS

        // Listens to lane detection points from a CV node
        // Queue size 10 (max 10 messages buffered)
        lane_sub_ = this->create_subscription<lane_detection::msg::FloatArray>(
            lane_float_topic_, 10,
            std::bind(&LaneScan::laneCallback, this, std::placeholders::_1));

        // Listens to lidar scan data
        // Queue size 10 (max 10 messages buffered)
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_laser_topic_, 10,
            std::bind(&LaneScan::lidarCallback, this, std::placeholders::_1));

        // PUBLISHERS
        // Publishes processed lane scan data as a LaserScan message
        out_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            lane_laser_topic_, 1);

        // Republishes lidar scan data with lane points merged in
        lidar_out_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            lidar_laser_topic_out_, 1);

        last_scan_found_ = false;

        RCLCPP_INFO(this->get_logger(), "LaneScan node started, listening to %s and %s",
                    lane_float_topic_.c_str(), lidar_laser_topic_.c_str());
    }

private:
    // Parameters
    std::string lane_float_topic_;
    std::string lane_laser_topic_;
    std::string lidar_laser_topic_;
    std::string lidar_laser_topic_out_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS Interfaces
    rclcpp::Subscription<lane_detection::msg::FloatArray>::SharedPtr lane_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr out_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_out_pub_;

    // State
    sensor_msgs::msg::LaserScan last_scan_msg_;
    rclcpp::Time last_scan_msg_time_;
    bool last_scan_found_;

    /// @brief Lane detection callback
    /// @param msg lane detection message
    ///
    /// Called when a new lane detection msg arrives
    void laneCallback(const lane_detection::msg::FloatArray::SharedPtr msg)
    {
        // Initialize LaserScan message
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.frame_id = "bottom_lidar_link";
        scan_msg.header.stamp = msg->header.stamp;

        // Spans from -135 to +135 degrees
        scan_msg.angle_min = -2.3561899662017822;
        scan_msg.angle_max = 2.3561899662017822;

        // 0.25 degree increments
        scan_msg.angle_increment = 0.0043673585169017315;

        scan_msg.range_min = 0.05999999865889549;
        scan_msg.range_max = 4.09499979019165;

        // Fill ranges with infinity (to mark no detection)
        for (double a = scan_msg.angle_min; a <= scan_msg.angle_max; a += scan_msg.angle_increment) {
            scan_msg.ranges.push_back(std::numeric_limits<float>::infinity());
        }

        bool point_added = false;

        if (!msg->lists.empty()) {
            const auto &list = msg->lists[0];

            for (const auto &point : list.elements) {

                // Transform point from camera frame to lidar frame
                geometry_msgs::msg::PoseStamped old_pose;
                old_pose.header.frame_id = msg->header.frame_id;
                old_pose.pose.position.x = point.x;
                old_pose.pose.position.y = point.y;
                old_pose.pose.position.z = point.z;
                old_pose.pose.orientation.w = 1.0;

                geometry_msgs::msg::PoseStamped new_pose;

                try {
                    // use tf2 buffer transform and catch exceptions
                    new_pose = tf_buffer_->transform(old_pose, new_pose, "bottom_lidar_link");
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                    continue;
                }

                // Calculate angle and distance
                double x = new_pose.pose.position.x;
                double y = new_pose.pose.position.y;
                double theta = std::atan2(y, x);
                double distance = std::sqrt(x*x + y*y);

                // Compute index - map angle to index in ranges array
                int idx = static_cast<int>((theta - scan_msg.angle_min) / scan_msg.angle_increment);
                idx = std::clamp(idx, 0, (int)scan_msg.ranges.size() - 1);

                if (scan_msg.ranges[idx] > 100.0) {
                    scan_msg.ranges[idx] = distance;
                } else {
                    scan_msg.ranges[idx] = std::min((double)scan_msg.ranges[idx], distance);
                }

                point_added = true;
            }
        }

        // Publish if at least one point was added
        if (point_added) {
            out_pub_->publish(scan_msg);
            last_scan_msg_ = scan_msg;
            last_scan_msg_time_ = this->now();
            last_scan_found_ = true;
        }
    }

    /// @brief Lidar scan callback
    /// @param lidar_scan message
    /// Checks if LIDAR scan has points < 1000 m
    /// If not, republishes last lane scan instead
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan)
    {
        bool point_found = false;

        for (float r : lidar_scan->ranges) {
            if (r < 1000.0f) {
                point_found = true;
                break;
            }
        }

        if (!point_found && last_scan_found_) {
            lidar_out_pub_->publish(last_scan_msg_);
        } else {
            lidar_out_pub_->publish(*lidar_scan);
        }
    }
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Creates a shared pointer to the LaneScan node and spins it (processes callbacks) 
    rclcpp::spin(std::make_shared<LaneScan>());

    // Shutdown ROS 2 node cleanly when done
    rclcpp::shutdown();
    return 0;
}
