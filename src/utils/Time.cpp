/**
 * @file Time.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Time class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Time.hpp"

std::map<std::string, Time::Task> Time::tasks_;
bool Time::print_after_tock;

void Time::tick(const std::string &clockName) {
  ros::WallTime now = ros::WallTime::now();
  std::map<std::string, Task>::iterator it = tasks_.find(clockName);
  if (it != tasks_.end()) {
    if (it->second.active) {
      ROS_WARN("[urinay] Called tick() two times with same clockName before calling tock()");
    }
    else {
      it->second.activate(now);
    }
  } else {
    tasks_.emplace(clockName, Task(now));
  }
}

ros::WallDuration Time::tock(const std::string &clockName) {
  ros::WallTime now = ros::WallTime::now();
  std::map<std::string, Task>::iterator it = tasks_.find(clockName);
  if (it == tasks_.end()) {
    ROS_ERROR("[urinay] Called tock() before calling tick()");
    return ros::WallDuration(0.0);
  } else {
    ros::WallDuration duration = it->second.stop(now);
    if (print_after_tock) ROS_INFO_STREAM("[urinay] " << clockName << " has taken: " << duration.toSec() * 1e3 << "ms");
    return duration;
  }
}

void Time::print_report() {
  std::stringstream to_print;
  // Table header
  to_print << "\033[2J\033[H";
  to_print << std::fixed << std::setprecision(2);
  to_print << std::setw(20) << std::left << "Task Name"
           << std::setw(12) << "Last (ms)"
           << std::setw(12) << "Max (ms)"
           << std::setw(12) << "Avg (ms)" << "\n";
  to_print << std::string(50, '-') << "\n";  // Separator line

  // Print each task and accumulate totals
  for (const auto& pair : tasks_) {
    const Task& chrono = pair.second;
    to_print << std::setw(20) << std::left << pair.first
             << std::setw(12) << chrono.last_duration.toSec() * 1e3
             << std::setw(12) << chrono.max_duration.toSec() * 1e3
             << std::setw(12) << (chrono.total_time_active.toSec() * 1e3) / chrono.count << "\n";
  }

  ROS_INFO_STREAM('\n' << to_print.str());
}
