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

std::map<std::string, ros::WallTime> Time::clocks_;

void Time::tick(const std::string &clockName) {
  std::map<std::string, ros::WallTime>::iterator it = clocks_.find(clockName);
  if (it != clocks_.end()) {
    ROS_WARN("[urinay] Called tick() two times with same clockName before calling tock()");
    it->second = ros::WallTime::now();
  } else {
    clocks_.emplace(clockName, ros::WallTime::now());
  }
}

ros::WallDuration Time::tock(const std::string &clockName) {
  std::map<std::string, ros::WallTime>::iterator it = clocks_.find(clockName);
  ros::WallDuration res;
  if (it == clocks_.end()) {
    ROS_ERROR("[urinay] Called tock() before calling tick()");
  } else {
    res = ros::WallTime::now() - it->second;
    ROS_INFO_STREAM("[urinay] " << it->first << " has taken: " << res.toSec() * 1e3 << "ms");
    clocks_.erase(it);
  }
  return res;
}
