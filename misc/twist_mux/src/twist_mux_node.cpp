/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

 #include <rclcpp/rclcpp.hpp>
 #include <twist_mux/twist_mux.h>
 #include <memory>
 
 int main(int argc, char *argv[])
 {
   // Initialize ROS 2
   rclcpp::init(argc, argv);
 
   try
   {
     // Create the twist_mux node
     auto mux = std::make_shared<twist_mux::TwistMux>();
 
     // Spin the node
     rclcpp::spin(mux);
   }
   catch (const std::exception& e)
   {
     RCLCPP_ERROR(rclcpp::get_logger("twist_mux_node"), "Exception caught: %s", e.what());
     return EXIT_FAILURE;
   }
 
   // Clean shutdown
   rclcpp::shutdown();
   return EXIT_SUCCESS;
 }

