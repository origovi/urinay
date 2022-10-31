/**
 * @file Time.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Time class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <ros/ros.h>

#include <map>
#include <stdexcept>
#include <string>

/**
 * @brief Abstract-static class made to make it easier to quantify the time
 * a specific set of operation take.
 */
class Time {
 private:
  static std::map<std::string, ros::WallTime> clocks_;

 public:
  Time() = delete;

  /**
   * @brief Creates a clock with name \a clockName.
   * 
   * @param[in] clockName 
   */
  static void tick(const std::string &clockName);

  /**
   * @brief Stops the clock with name \a clockName, returns the duration
   * and prompts the clock name and duration.
   * 
   * @param[in] clockName 
   */
  static ros::WallDuration tock(const std::string &clockName);
};