/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *********************************************************************/

 #ifndef TWIST_MUX_UTILS_H
 #define TWIST_MUX_UTILS_H
 
 #include <algorithm>  // For std::clamp
 
 namespace twist_mux
 {
 
 // Forward declarations
 class TwistMuxDiagnostics;
 struct TwistMuxDiagnosticsStatus;
 class VelocityTopicHandle;
 class LockTopicHandle;
 class TwistMux;
 
 /**
  * @brief Clamp a value to the range [min, max]
  */
 template<typename T>
 constexpr T clamp(const T& x, const T& min, const T& max)
 {
   return std::clamp(x, min, max);
 }
 
 } // namespace twist_mux
 
 #endif // TWIST_MUX_UTILS_H