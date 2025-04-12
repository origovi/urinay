/**
 * @file Logger.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Logger class specification
 * @version 1.0
 * @date 2025-04-12
 * 
 * @copyright Copyright (c) 2025 TUfast e.V.
 */

#pragma once

#include <ros/ros.h>

#include <map>
#include <stdexcept>
#include <string>
#include <iomanip>
#include <queue>

/**
 * @brief Abstract-static class made to make it easier show a report after each
 * iteration including time statistics and important messages.
 */
class Logger {
 private:
  /**
   * @brief This private class helps to store all the metrics needed for time
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

  /**
   * @brief Map that includes all tasks that need to be timed and contains
   * all timing info.
   */
  static std::map<std::string, Task> tasks_;

  /**
   * @brief Contains the current iteration's msgs that need to be printed in
   * the next log report.
   */
  static std::queue<std::string> info_msgs_, warn_msgs_;

 public:
  Logger() = delete;

  /**
   * @brief When true, after every call to tock() or log, the time or log msg
   * will be printed.
   */
  static bool print_immediately;

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
   * @brief Appends a msg to be printed in the next log report with "info"
   * priority.
   */
  static void loginfo(const std::string &msg);

  /**
   * @brief Appends a msg to be printed in the next log report with "warning"
   * priority.
   */
  static void logwarn(const std::string &msg);

  /**
   * @brief Prints a time report that includes time metrics of all tasks and
   * log messages that were added in that iteration.
   */
  static void print_report();
};