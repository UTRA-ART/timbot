#ifndef TWIST_MUX_DIAGNOSTICS_H
#define TWIST_MUX_DIAGNOSTICS_H

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <twist_mux/twist_mux_diagnostics_status.h>  // Include full definition

#include <memory>

namespace twist_mux
{

class TwistMuxDiagnostics
{
public:
  using status_type = TwistMuxDiagnosticsStatus;

  static constexpr double MAIN_LOOP_TIME_MIN = 0.2; // [s]
  static constexpr double READING_AGE_MIN    = 3.0; // [s]

  explicit TwistMuxDiagnostics(rclcpp::Node::SharedPtr node);
  // Remove = default since we need to implement the destructor
  virtual ~TwistMuxDiagnostics();

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void update();
  void updateStatus(std::shared_ptr<const status_type> status);

private:
  static constexpr uint8_t OK    = diagnostic_msgs::msg::DiagnosticStatus::OK;
  static constexpr uint8_t WARN  = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  static constexpr uint8_t ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_;
  std::shared_ptr<status_type> status_;
};

} // namespace twist_mux

#endif // TWIST_MUX_DIAGNOSTICS_H