

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

enum State { no_ramp, to_ramp, on_ramp };

class RampNavigateNode {
 public:
  void begin() {
    state = no_ramp;
    ramps_to_cross = 1;

    ramp_seg_sub = this->create_subscription<type>(
        "/ramp_seg", 10,
        std::bind(&RampNavigateNode::rampFrontCallback, this, _1));
  }

 private:
  int ramps_to_cross;
  State state;
  int pre_ramp_detections = 0;
  int no_ramp_period = 0;
  float slope, xmid, ymid, px, py;
  Eigen::Matrix2d ramp2map;

  
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv, "ramp_navigate");
}