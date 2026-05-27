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

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "videocapture.hpp"
#include "calibration.hpp" // <-- Added to use driver's auto-calibration

namespace
{
sl_oc::video::RESOLUTION parse_resolution(const std::string & value)
{
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "HD2K") return sl_oc::video::RESOLUTION::HD2K;
  if (upper == "HD1080") return sl_oc::video::RESOLUTION::HD1080;
  if (upper == "HD720") return sl_oc::video::RESOLUTION::HD720;
  return sl_oc::video::RESOLUTION::VGA;
}

sl_oc::video::FPS parse_fps(int value)
{
  if (value >= 100) return sl_oc::video::FPS::FPS_100;
  if (value >= 60) return sl_oc::video::FPS::FPS_60;
  if (value >= 30) return sl_oc::video::FPS::FPS_30;
  return sl_oc::video::FPS::FPS_15;
}

int parse_device_id(const std::string & value)
{
  if (value.rfind("/dev/video", 0) == 0) {
    try { return std::stoi(value.substr(10)); } catch (...) { return -1; }
  }
  try { return std::stoi(value); } catch (...) { return -1; }
}

int parse_yuv_code(const std::string & value)
{
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "UYVY") return cv::COLOR_YUV2BGR_UYVY;
  if (upper == "YVYU") return cv::COLOR_YUV2BGR_YVYU;
  return cv::COLOR_YUV2BGR_YUYV;
}
}

class ZedOpenCaptureNode : public rclcpp::Node
{
public:
  ZedOpenCaptureNode()
  : Node("zed_open_capture_node")
  {
    // Camera and stereo tuning parameters.
    declare_parameter("video_device", std::string("/dev/video0"));
    declare_parameter("num_disparities", 96);
    declare_parameter("block_size", 3);
    declare_parameter("p1_multiplier", 8);
    declare_parameter("p2_multiplier", 32);
    std::string video_device = get_parameter("video_device").as_string();

    // Setup ROS Publishers
    left_pub_ = create_publisher<sensor_msgs::msg::Image>(left_image_topic_, rclcpp::SensorDataQoS());
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(left_camera_info_topic_, rclcpp::SensorDataQoS());
    auto depth_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic_, depth_qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, rclcpp::SensorDataQoS());

    yuv_code_ = parse_yuv_code(yuv_format_);

    // 1. Initialize Video Capture
    sl_oc::video::VideoParams params;
    params.res = parse_resolution(resolution_);
    params.fps = parse_fps(fps_);
    params.verbose = sl_oc::VERBOSITY::INFO;

    cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
    int dev_id = parse_device_id(video_device);
    
    if (!cap_->initializeVideo(dev_id)) {
      RCLCPP_FATAL(get_logger(), "Failed to open video device: %s", video_device.c_str());
      throw std::runtime_error("Failed to initialize ZED Open Capture");
    }

    declare_parameter("aec_agc", true);
    declare_parameter("exposure", 157);
    declare_parameter("gain", 0);
    declare_parameter("auto_white_balance", true);
    declare_parameter("brightness", 0);
    declare_parameter("contrast", 32);
    declare_parameter("gamma", 100);
    declare_parameter("hue", 0);
    declare_parameter("saturation", 64);
    declare_parameter("sharpness", 3);

    cap_->resetAECAGC();
    cap_->resetAutoWhiteBalance();
    cap_->resetBrightness();
    cap_->resetContrast();
    cap_->resetGamma();
    cap_->resetHue();
    cap_->resetSaturation();
    cap_->resetSharpness();
    RCLCPP_INFO(get_logger(), "Capture params reset to device defaults");

    apply_capture_params(true);

    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool should_apply = false;
        for (const auto & param : params) {
          const auto & name = param.get_name();
          if (name == "aec_agc" || name == "exposure" || name == "gain" ||
              name == "auto_white_balance" || name == "brightness" ||
              name == "contrast" || name == "gamma" || name == "hue" ||
              name == "saturation" || name == "sharpness") {
            should_apply = true;
            break;
          }
        }

        if (should_apply) {
          try {
            apply_capture_params(false);
          } catch (const std::exception & e) {
            result.successful = false;
            result.reason = e.what();
          }
        }

        return result;
      });

    // 2. Auto-load calibration from the driver/server
    int sn = cap_->getSerialNumber();
    RCLCPP_INFO(get_logger(), "Connected to camera sn: %d", sn);

    std::string calib_file;
    if (!sl_oc::tools::downloadCalibrationFile(sn, calib_file)) {
      RCLCPP_FATAL(get_logger(), "Could not load calibration file from Stereolabs servers");
      throw std::runtime_error("Calibration download failed");
    }
    RCLCPP_INFO(get_logger(), "Calibration file found. Loading...");

    // 3. Get exact dimensions and let the driver build the rectification maps
    int w, h;
    cap_->getFrameSize(w, h);
    int half_w = w / 2;

    cv::Mat k_left, k_right;
    double baseline = 0.0;

    sl_oc::tools::initCalibration(
      calib_file, cv::Size(half_w, h), 
      left_map1_, left_map2_, right_map1_, right_map2_,
      k_left, k_right, &baseline
    );

    // 4. Save the auto-loaded intrinsics to class variables
    fx_ = static_cast<float>(k_left.at<double>(0, 0));
    fy_ = static_cast<float>(k_left.at<double>(1, 1));
    cx_ = static_cast<float>(k_left.at<double>(0, 2));
    cy_ = static_cast<float>(k_left.at<double>(1, 2));
    baseline_ = static_cast<float>(baseline / 1000.0);

    RCLCPP_INFO(get_logger(), "Auto-loaded params: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, baseline=%.4f",
                fx_, fy_, cx_, cy_, baseline_);

    init_sgbm();

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, fps_)));
    timer_ = create_wall_timer(period, std::bind(&ZedOpenCaptureNode::on_timer, this));
  }

private:
  void apply_capture_params(bool log)
  {
    const bool aec_agc = get_parameter("aec_agc").as_bool();
    const int exposure = get_parameter("exposure").as_int();
    const int gain = get_parameter("gain").as_int();
    const bool auto_white_balance = get_parameter("auto_white_balance").as_bool();
    const int brightness = get_parameter("brightness").as_int();
    const int contrast = get_parameter("contrast").as_int();
    const int gamma = get_parameter("gamma").as_int();
    const int hue = get_parameter("hue").as_int();
    const int saturation = get_parameter("saturation").as_int();
    const int sharpness = get_parameter("sharpness").as_int();

    cap_->setAECAGC(aec_agc);
    if (!aec_agc) {
      cap_->setExposure(sl_oc::video::CAM_SENS_POS::LEFT, exposure);
      cap_->setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, exposure);
      cap_->setGain(sl_oc::video::CAM_SENS_POS::LEFT, gain);
      cap_->setGain(sl_oc::video::CAM_SENS_POS::RIGHT, gain);
    }
    cap_->setAutoWhiteBalance(auto_white_balance);
    cap_->setBrightness(brightness);
    cap_->setContrast(contrast);
    cap_->setGamma(gamma);
    cap_->setHue(hue);
    cap_->setSaturation(saturation);
    cap_->setSharpness(sharpness);

    if (log) {
      RCLCPP_INFO(
        get_logger(),
        "Capture params: AEC/AGC=%s, exp=%d, gain=%d, AWB=%s, bright=%d, contrast=%d, gamma=%d, hue=%d, sat=%d, sharp=%d",
        aec_agc ? "on" : "off",
        exposure,
        gain,
        auto_white_balance ? "on" : "off",
        brightness,
        contrast,
        gamma,
        hue,
        saturation,
        sharpness);
    }
  }

  void init_sgbm()
  {
    const int min_disparity = 0;
    const int num_disparities = get_parameter("num_disparities").as_int();
    const int block_size = get_parameter("block_size").as_int();
    const int p1_multiplier = get_parameter("p1_multiplier").as_int();
    const int p2_multiplier = get_parameter("p2_multiplier").as_int();

    sgbm_ = cv::StereoSGBM::create(min_disparity, num_disparities, block_size);
    const int p1 = p1_multiplier * block_size * block_size;
    const int p2 = p2_multiplier * block_size * block_size;
    sgbm_->setP1(p1);
    sgbm_->setP2(p2);
    sgbm_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    sgbm_->setPreFilterCap(63);
    sgbm_->setUniquenessRatio(5);
    sgbm_->setSpeckleWindowSize(255);
    sgbm_->setSpeckleRange(1);
    sgbm_->setDisp12MaxDiff(96);
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
    if (half_width <= 0) return;

    cv::Mat left_bgr = frame_bgr(cv::Rect(0, 0, half_width, frame_bgr.rows));
    cv::Mat right_bgr = frame_bgr(cv::Rect(half_width, 0, half_width, frame_bgr.rows));

    cv::Mat left_gray, right_gray;
    cv::cvtColor(left_bgr, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_bgr, right_gray, cv::COLOR_BGR2GRAY);

    // Apply the auto-loaded stereo rectification maps directly
    cv::Mat left_bgr_rect, left_gray_rect, right_gray_rect;
    cv::remap(left_bgr, left_bgr_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
    cv::remap(left_gray, left_gray_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
    cv::remap(right_gray, right_gray_rect, right_map1_, right_map2_, cv::INTER_LINEAR);

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
      left_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0}; // Remapped images have 0 distortion

      // Use the auto-calculated intrinsics!
      left_info_.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
      left_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      left_info_.p = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};

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
          *iter_x = (static_cast<float>(c) - cx_) * z / fx_;
          *iter_y = (static_cast<float>(r) - cy_) * z / fy_;
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
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // --- Start configuration defaults ---
  std::string resolution_ = "VGA";
  int fps_ = 15;
  std::string yuv_format_ = "YUYV";
  int yuv_code_ = cv::COLOR_YUV2BGR_YUYV;

  std::string left_frame_id_ = "left_camera_link_optical";
  std::string left_image_topic_ = "/zed_node/left/image";
  std::string left_camera_info_topic_ = "/zed_node/left/camera_info";
  std::string depth_image_topic_ = "/zed_node/left/depth_image";
  std::string point_cloud_topic_ = "/zed_node/left/points";

  float min_depth_ = 0.5f;
  float max_depth_ = 20.0f;
  // --- End configuration defaults ---

  // These will automatically populate via driver calibration logic
  float fx_, fy_, cx_, cy_, baseline_;

  cv::Mat left_map1_, left_map2_, right_map1_, right_map2_;
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