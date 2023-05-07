/**
 * @file Failsafe.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Failsafe class member functions implementation
 * @version 1.0
 * @date 2022-05-06
 *
 * @copyright Copyright (c) 2023 BCN eMotorsport
 */

#include "utils/Failsafe.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods  ---------------------------- */

template<typename T>
void Failsafe<T>::initGeneral(const T &params, const double &safetyFactor, const int &failsafe_max_way_horizon_size) {
  *this = Failsafe<T>(params);
  this->max_way_horizon_size = failsafe_max_way_horizon_size;

  this->search_radius *= safetyFactor;
  this->max_angle_diff = std::min(this->max_angle_diff*safetyFactor, M_PI_2);
  this->edge_len_diff_factor *= safetyFactor;
  this->max_next_heuristic *= safetyFactor;
}
template void Failsafe<Params::WayComputer::Search>::initGeneral(const Params::WayComputer::Search &, const double &, const int &);