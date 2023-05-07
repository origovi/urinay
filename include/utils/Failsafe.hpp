/**
 * @file Failsafe.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Failsafe class specification
 * @version 1.0
 * @date 2022-05-06
 *
 * @copyright Copyright (c) 2023 BCN eMotorsport
 */

#pragma once

#include "utils/Params.hpp"
#include "utils/constants.hpp"
#include <cmath>

/**
 * @brief A class that provides all functionality to create and manage failsafe
 * parameters.
 */
template <typename T>
class Failsafe : public T {
 private:
 public:
  Failsafe() = default;
  Failsafe(const T &x) : T(x) {}

  /**
   * @brief Sets implicit object to a general failsafe object, using its parameters.
   * The objectives are:
   * - Increase all parameters (thresholds) by a factor.
   * - Limit sight length.
   * 
   * @param params 
   * @param safetyFactor 
   * @param failsafe_max_way_horizon_size 
   */
  void initGeneral(const T &params, const double &safetyFactor, const int &failsafe_max_way_horizon_size);
};