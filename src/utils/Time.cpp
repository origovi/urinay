#include "utils/Time.hpp"

std::map<std::string, ros::WallTime> Time::clocks_;

void Time::tick(const std::string &clockName) {
  std::map<std::string, ros::WallTime>::iterator it = clocks_.find(clockName);
  if (it != clocks_.end()) {
    ROS_WARN("Called tick() two times with same clockName before calling tock()");
    it->second = ros::WallTime::now();
  } else {
    clocks_.emplace(clockName, ros::WallTime::now());
  }
}

ros::WallDuration Time::tock(const std::string &clockName) {
  std::map<std::string, ros::WallTime>::iterator it = clocks_.find(clockName);
  ros::WallDuration res;
  if (it == clocks_.end()) {
    ROS_ERROR("Called tock() before calling tick()");
  } else {
    res = ros::WallTime::now() - it->second;
    ROS_INFO_STREAM(it->first << " has taken: " << res.toSec() * 1e3 << "ms");
    clocks_.erase(it);
  }
  return res;
}
