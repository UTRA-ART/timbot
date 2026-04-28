#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>

#include "videocapture.hpp"

namespace
{
sl_oc::video::RESOLUTION parse_resolution(const std::string & value)
{
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "HD2K") {
    return sl_oc::video::RESOLUTION::HD2K;
  }
  if (upper == "HD1080") {
    return sl_oc::video::RESOLUTION::HD1080;
  }
  if (upper == "HD720") {
    return sl_oc::video::RESOLUTION::HD720;
  }
  return sl_oc::video::RESOLUTION::VGA;
}

sl_oc::video::FPS parse_fps(int value)
{
  if (value >= 100) {
    return sl_oc::video::FPS::FPS_100;
  }
  if (value >= 60) {
    return sl_oc::video::FPS::FPS_60;
  }
  if (value >= 30) {
    return sl_oc::video::FPS::FPS_30;
  }
  return sl_oc::video::FPS::FPS_15;
}

int parse_device_id(const std::string & value)
{
  if (value.rfind("/dev/video", 0) == 0) {
    try {
      return std::stoi(value.substr(10));
    } catch (...) {
      return -1;
    }
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return -1;
  }
}

int parse_yuv_code(const std::string & value)
{
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "UYVY") {
    return cv::COLOR_YUV2BGR_UYVY;
  }
  if (upper == "YVYU") {
    return cv::COLOR_YUV2BGR_YVYU;
  }
  return cv::COLOR_YUV2BGR_YUYV;
}
}

class ZedOpenCaptureNode : public rclcpp::Node
{
public:
  ZedOpenCaptureNode()
  : Node("zed_open_capture_node")
  {
    declare_parameter("video_device", std::string("/dev/video2"));
    declare_parameter("resolution", std::string("VGA"));
    declare_parameter("fps", 15);
    declare_parameter("yuv_format", std::string("YUYV"));
    declare_parameter("fx", 350.0);
    declare_parameter("fy", 350.0);
    declare_parameter("cx", 336.0);
    declare_parameter("cy", 188.0);
    declare_parameter("baseline", 0.12);
    declare_parameter("min_depth", 0.5);
    declare_parameter("max_depth", 20.0);
    declare_parameter("rectify", true);
    declare_parameter("k1", 0.0);
    declare_parameter("k2", 0.0);
    declare_parameter("p1", 0.0);
    declare_parameter("p2", 0.0);
    declare_parameter("k3", 0.0);
    declare_parameter("left_frame_id", std::string("left_camera_link_optical"));
    declare_parameter("left_image_topic", std::string("/zed_node/left/image"));
    declare_parameter("left_camera_info_topic", std::string("/zed_node/left/camera_info"));
    declare_parameter("depth_image_topic", std::string("/zed_node/left/depth_image"));
    declare_parameter("point_cloud_topic", std::string("/zed_node/left/points"));

    video_device_ = get_parameter("video_device").as_string();
    resolution_ = get_parameter("resolution").as_string();
    fps_ = get_parameter("fps").as_int();
    yuv_format_ = get_parameter("yuv_format").as_string();

    fx_ = static_cast<float>(get_parameter("fx").as_double());
    fy_ = static_cast<float>(get_parameter("fy").as_double());
    cx_ = static_cast<float>(get_parameter("cx").as_double());
    cy_ = static_cast<float>(get_parameter("cy").as_double());
    baseline_ = static_cast<float>(get_parameter("baseline").as_double());
    min_depth_ = static_cast<float>(get_parameter("min_depth").as_double());
    max_depth_ = static_cast<float>(get_parameter("max_depth").as_double());
    rectify_ = get_parameter("rectify").as_bool();
    k1_ = get_parameter("k1").as_double();
    k2_ = get_parameter("k2").as_double();
    p1_ = get_parameter("p1").as_double();
    p2_ = get_parameter("p2").as_double();
    k3_ = get_parameter("k3").as_double();

    depth_fx_ = fx_;
    depth_fy_ = fy_;
    depth_cx_ = cx_;
    depth_cy_ = cy_;

    left_frame_id_ = get_parameter("left_frame_id").as_string();

    left_image_topic_ = get_parameter("left_image_topic").as_string();
    left_camera_info_topic_ = get_parameter("left_camera_info_topic").as_string();
    depth_image_topic_ = get_parameter("depth_image_topic").as_string();
    point_cloud_topic_ = get_parameter("point_cloud_topic").as_string();

    left_pub_ = create_publisher<sensor_msgs::msg::Image>(left_image_topic_, rclcpp::SensorDataQoS());
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(left_camera_info_topic_,
      rclcpp::SensorDataQoS());
    auto depth_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic_, depth_qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, rclcpp::SensorDataQoS());

    yuv_code_ = parse_yuv_code(yuv_format_);

    sl_oc::video::VideoParams params;
    params.res = parse_resolution(resolution_);
    params.fps = parse_fps(fps_);
    params.verbose = sl_oc::VERBOSITY::INFO;

    cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
    int dev_id = parse_device_id(video_device_);
    if (!cap_->initializeVideo(dev_id)) {
      RCLCPP_FATAL(get_logger(), "Failed to open video device: %s", video_device_.c_str());
      throw std::runtime_error("Failed to initialize ZED Open Capture");
    }

    init_sgbm();

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, fps_)));
    timer_ = create_wall_timer(period, std::bind(&ZedOpenCaptureNode::on_timer, this));
  }

private:
  void init_sgbm()
  {
    int min_disparity = 0;
    int num_disparities = 96;  // Must be multiple of 16
    int block_size = 3;        // Must be odd

    min_disparity_ = min_disparity;
    num_disparities_ = num_disparities;
    block_size_ = block_size;

    sgbm_ = cv::StereoSGBM::create(min_disparity_, num_disparities_, block_size_);
    int p1 = 8 * block_size_ * block_size_;
    int p2 = 32 * block_size_ * block_size_;
    sgbm_->setP1(p1);
    sgbm_->setP2(p2);
    sgbm_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    sgbm_->setPreFilterCap(63);
    sgbm_->setUniquenessRatio(5);
    sgbm_->setSpeckleWindowSize(255);
    sgbm_->setSpeckleRange(1);
    sgbm_->setDisp12MaxDiff(96);
  }

  void ensure_rectification(int width, int height)
  {
    if (width <= 0 || height <= 0) {
      rect_ready_ = false;
      return;
    }
    if (rect_ready_ && rect_width_ == width && rect_height_ == height) {
      return;
    }

    cv::Mat k_left = (cv::Mat_<double>(3, 3) <<
      fx_, 0.0, cx_,
      0.0, fy_, cy_,
      0.0, 0.0, 1.0
    );
    cv::Mat k_right = k_left.clone();

    cv::Mat d_left = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
    cv::Mat d_right = d_left.clone();

    cv::Mat r = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = (cv::Mat_<double>(3, 1) << -static_cast<double>(baseline_), 0.0, 0.0);

    cv::Mat r1, r2, p1, p2, q;
    cv::stereoRectify(
      k_left, d_left, k_right, d_right,
      cv::Size(width, height),
      r, t, r1, r2, p1, p2, q,
      cv::CALIB_ZERO_DISPARITY, 0.0, cv::Size(width, height)
    );

    cv::initUndistortRectifyMap(
      k_left, d_left, r1, p1,
      cv::Size(width, height),
      CV_16SC2, left_map1_, left_map2_
    );
    cv::initUndistortRectifyMap(
      k_right, d_right, r2, p2,
      cv::Size(width, height),
      CV_16SC2, right_map1_, right_map2_
    );

    depth_fx_ = static_cast<float>(p1.at<double>(0, 0));
    depth_fy_ = static_cast<float>(p1.at<double>(1, 1));
    depth_cx_ = static_cast<float>(p1.at<double>(0, 2));
    depth_cy_ = static_cast<float>(p1.at<double>(1, 2));

    rect_r_ = {
      r1.at<double>(0, 0), r1.at<double>(0, 1), r1.at<double>(0, 2),
      r1.at<double>(1, 0), r1.at<double>(1, 1), r1.at<double>(1, 2),
      r1.at<double>(2, 0), r1.at<double>(2, 1), r1.at<double>(2, 2)
    };
    rect_p_ = {
      p1.at<double>(0, 0), p1.at<double>(0, 1), p1.at<double>(0, 2), p1.at<double>(0, 3),
      p1.at<double>(1, 0), p1.at<double>(1, 1), p1.at<double>(1, 2), p1.at<double>(1, 3),
      p1.at<double>(2, 0), p1.at<double>(2, 1), p1.at<double>(2, 2), p1.at<double>(2, 3)
    };

    rect_width_ = width;
    rect_height_ = height;
    rect_ready_ = true;
    cam_info_ready_ = false;
  }

  void on_timer()
  {
    const sl_oc::video::Frame frame = cap_->getLastFrame(100);
    if (frame.data == nullptr || frame.timestamp == last_ts_) {
      return;
    }
    last_ts_ = frame.timestamp;

    cv::Mat frame_yuv(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuv, frame_bgr, yuv_code_);

    int half_width = frame_bgr.cols / 2;
    if (half_width <= 0) {
      return;
    }

    cv::Mat left_bgr = frame_bgr(cv::Rect(0, 0, half_width, frame_bgr.rows));
    cv::Mat right_bgr = frame_bgr(cv::Rect(half_width, 0, half_width, frame_bgr.rows));

    cv::Mat left_gray;
    cv::Mat right_gray;
    cv::cvtColor(left_bgr, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_bgr, right_gray, cv::COLOR_BGR2GRAY);

    cv::Mat left_bgr_rect = left_bgr;
    cv::Mat left_gray_rect = left_gray;
    cv::Mat right_gray_rect = right_gray;
    if (rectify_) {
      ensure_rectification(left_gray.cols, left_gray.rows);
      if (rect_ready_) {
        cv::remap(left_bgr, left_bgr_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(left_gray, left_gray_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(right_gray, right_gray_rect, right_map1_, right_map2_, cv::INTER_LINEAR);
      }
    }

    cv::Mat disp_16;
    sgbm_->compute(left_gray_rect, right_gray_rect, disp_16);

    cv::Mat disp_float;
    disp_16.convertTo(disp_float, CV_32F, 1.0 / 16.0);

    cv::Mat depth(left_gray.rows, left_gray.cols, CV_32F, cv::Scalar(0.0f));
    float f_times_b = fx_ * baseline_;
    for (int r = 0; r < disp_float.rows; ++r) {
      float * disp_row = disp_float.ptr<float>(r);
      float * depth_row = depth.ptr<float>(r);
      for (int c = 0; c < disp_float.cols; ++c) {
        float d = disp_row[c];
        if (d <= 0.0f) {
          depth_row[c] = 0.0f;
          continue;
        }
        float z = f_times_b / d;
        if (z < min_depth_ || z > max_depth_) {
          depth_row[c] = 0.0f;
        } else {
          depth_row[c] = z;
        }
      }
    }

    publish_left_image(left_bgr_rect, frame.timestamp);
    publish_left_info(left_bgr_rect.cols, left_bgr_rect.rows, frame.timestamp);
    publish_depth(depth, frame.timestamp);
    publish_point_cloud(depth, frame.timestamp);
  }

  void publish_left_image(const cv::Mat & left_bgr, uint64_t ts)
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(static_cast<int64_t>(ts));
    header.frame_id = left_frame_id_;

    auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, left_bgr).toImageMsg();
    left_pub_->publish(*msg);
  }

  void publish_left_info(int width, int height, uint64_t ts)
  {
    if (!cam_info_ready_ || width != info_width_ || height != info_height_) {
      left_info_.width = static_cast<uint32_t>(width);
      left_info_.height = static_cast<uint32_t>(height);
      left_info_.distortion_model = "plumb_bob";
      left_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

      if (rectify_ && rect_ready_) {
        left_info_.k = {depth_fx_, 0.0, depth_cx_, 0.0, depth_fy_, depth_cy_, 0.0, 0.0, 1.0};
        left_info_.r = rect_r_;
        left_info_.p = rect_p_;
      } else {
        left_info_.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
        left_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        left_info_.p = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
      }

      cam_info_ready_ = true;
      info_width_ = width;
      info_height_ = height;
    }

    left_info_.header.stamp = rclcpp::Time(static_cast<int64_t>(ts));
    left_info_.header.frame_id = left_frame_id_;
    left_info_pub_->publish(left_info_);
  }

  void publish_depth(const cv::Mat & depth, uint64_t ts)
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(static_cast<int64_t>(ts));
    header.frame_id = left_frame_id_;

    auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, depth).toImageMsg();
    depth_pub_->publish(*msg);
  }

  void publish_point_cloud(const cv::Mat & depth, uint64_t ts)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(ts));
    cloud.header.frame_id = left_frame_id_;
    cloud.height = static_cast<uint32_t>(depth.rows);
    cloud.width = static_cast<uint32_t>(depth.cols);
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(depth.rows * depth.cols);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (int r = 0; r < depth.rows; ++r) {
      const float * depth_row = depth.ptr<float>(r);
      for (int c = 0; c < depth.cols; ++c, ++iter_x, ++iter_y, ++iter_z) {
        float z = depth_row[c];
        if (z <= 0.0f) {
          *iter_x = nan;
          *iter_y = nan;
          *iter_z = nan;
        } else {
          *iter_x = (static_cast<float>(c) - depth_cx_) * z / depth_fx_;
          *iter_y = (static_cast<float>(r) - depth_cy_) * z / depth_fy_;
          *iter_z = z;
        }
      }
    }

    cloud_pub_->publish(cloud);
  }

  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::shared_ptr<cv::StereoSGBM> sgbm_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::string video_device_;
  std::string resolution_;
  int fps_ = 15;
  std::string yuv_format_;
  int yuv_code_ = cv::COLOR_YUV2BGR_YUYV;

  std::string left_frame_id_;
  std::string left_image_topic_;
  std::string left_camera_info_topic_;
  std::string depth_image_topic_;
  std::string point_cloud_topic_;

  float fx_ = 350.0f;
  float fy_ = 350.0f;
  float cx_ = 336.0f;
  float cy_ = 188.0f;
  float baseline_ = 0.12f;
  float min_depth_ = 0.5f;
  float max_depth_ = 20.0f;

  bool rectify_ = false;
  double k1_ = 0.0;
  double k2_ = 0.0;
  double p1_ = 0.0;
  double p2_ = 0.0;
  double k3_ = 0.0;

  float depth_fx_ = 350.0f;
  float depth_fy_ = 350.0f;
  float depth_cx_ = 336.0f;
  float depth_cy_ = 188.0f;

  bool rect_ready_ = false;
  int rect_width_ = 0;
  int rect_height_ = 0;
  cv::Mat left_map1_;
  cv::Mat left_map2_;
  cv::Mat right_map1_;
  cv::Mat right_map2_;
  std::array<double, 9> rect_r_{};
  std::array<double, 12> rect_p_{};

  int min_disparity_ = 0;
  int num_disparities_ = 96;
  int block_size_ = 3;

  sensor_msgs::msg::CameraInfo left_info_;
  bool cam_info_ready_ = false;
  int info_width_ = 0;
  int info_height_ = 0;

  uint64_t last_ts_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ZedOpenCaptureNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Failed to start zed_open_capture_node: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
