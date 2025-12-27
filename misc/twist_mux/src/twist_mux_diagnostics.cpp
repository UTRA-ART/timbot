#include <twist_mux/twist_mux_diagnostics.h>
#include <twist_mux/twist_mux_diagnostics_status.h>

#include <algorithm>
#include <sstream>

namespace twist_mux
{

TwistMuxDiagnostics::TwistMuxDiagnostics(rclcpp::Node::SharedPtr node)
  : node_(node)
  , diagnostic_(std::make_unique<diagnostic_updater::Updater>(node))
  , status_(std::make_shared<status_type>())
{
  diagnostic_->add("Twist mux status", this, &TwistMuxDiagnostics::diagnostics);
  diagnostic_->setHardwareID("none");
}

TwistMuxDiagnostics::~TwistMuxDiagnostics()
{
  // Destructor implementation (can be empty but needs to be defined)
}

void TwistMuxDiagnostics::update()
{
  diagnostic_->force_update();  // Use force_update() instead of update()
}

void TwistMuxDiagnostics::updateStatus(std::shared_ptr<const status_type> status)
{
  if (status)
  {
    // Copy the status data
    status_->velocity_hs = status->velocity_hs;
    status_->lock_hs     = status->lock_hs;
    status_->priority    = status->priority;
    
    status_->main_loop_time = status->main_loop_time;
    status_->reading_age    = status->reading_age;
  }
}

void TwistMuxDiagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!status_)
  {
    stat.summary(ERROR, "No status available");
    return;
  }

  // Check main loop time and reading age
  if (status_->main_loop_time > MAIN_LOOP_TIME_MIN)
    stat.summary(WARN, "Loop time too high");
  else if (status_->reading_age > READING_AGE_MIN)
    stat.summary(WARN, "Data too old");
  else
    stat.summary(OK, "OK");

  // Add velocity topic information
  if (status_->velocity_hs)
  {
    for (const auto& velocity_h : *status_->velocity_hs)
    {
      std::ostringstream ss;
      ss << velocity_h.getName() << " (" << 
            (velocity_h.isMasked(status_->priority) ? "masked" : "unmasked") << 
            ")";
      stat.add(ss.str(), velocity_h.getTopic());
    }
  }

  // Add lock topic information  
  if (status_->lock_hs)
  {
    for (const auto& lock_h : *status_->lock_hs)
    {
      std::ostringstream ss;
      ss << lock_h.getName() << " (" << 
            (lock_h.isLocked() ? "locked" : "unlocked") << 
            ")";
      stat.add(ss.str(), lock_h.getTopic());
    }
  }

  // Add summary data
  stat.add("current priority", static_cast<int>(status_->priority));
  stat.add("loop time in [sec]", status_->main_loop_time);
  stat.add("data age in [sec]", status_->reading_age);
}

} // namespace twist_mux