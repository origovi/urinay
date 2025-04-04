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
#include <iomanip>

/**
 * @brief Abstract-static class made to make it easier to quantify the time
 * a specific set of operation take.
 */
class Time {
 private:
  /**
   * @brief This private class helps to store all the metrics needed for
   * profiling.
   */
  struct Task {
    bool active = true;
    ros::WallTime startTime;
    ros::WallDuration total_time_active;
    uint64_t count = 0;
    ros::WallDuration last_duration, max_duration;
    Task(const ros::WallTime &_startTime)
      : startTime(_startTime) {}
    void activate(const ros::WallTime &_startTime) {
      startTime = _startTime;
      active = true;
    }
    ros::WallDuration stop(const ros::WallTime &stopTime) {
      active = false;
      last_duration = stopTime - startTime;
      total_time_active += last_duration;
      if (last_duration > max_duration) max_duration = last_duration;
      count++;
      return last_duration;
    }
  };
  static std::map<std::string, Task> tasks_;

 public:
  Time() = delete;

  /**
   * @brief When true, after every call to tock(), a line indicating the task
   * duration is printed.
   */
  static bool print_after_tock;

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

  /**
   * @brief Prints a time report that includes time metrics of all tasks.
   * The report includes:
   * - Last duration of task
   * - Max duration of task
   * - Average duration of task
   */
  static void print_report();

};