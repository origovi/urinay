/**
 * @file constants.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the constants used throughout the project.
 * @version 1.0
 * @date 2023-05-01
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <cstdint>

/**
 * @brief This number is the shifting number that will be used when computing
 * hashes for objects whose hash is defined by more than one element. Given
 * that the object whose hash has more elements is the triangle (3), this
 * number has been adjuested accordingly, i.e. 21*3 == 63, 63 < 64 bits.
 * It also means that the biggest cone ID MUST be less than 2^HASH_SHIFT_NUM-3.
 * The last "3" is because of the super triangle. We must be able to create it.
 */
constexpr uint32_t HASH_SHIFT_NUM = 21;

/**
 * @brief Minimum number of midpoints required to be able to make a loop
 * closure. No loop will be closed if its size is less than
 * \a MIN_LOOP_SIZE.
 */
constexpr uint32_t MIN_LOOP_SIZE = 25;

/**
 * @brief If two midpoints have a distance (between each other) of less than
 * \a SAME_MIDPOINT_DIST_THRESHOLD meters, it will be considered to be the same
 * point.
 */
constexpr double SAME_MIDPOINT_DIST_THRESHOLD = 0.2;

/**
 * @brief Minimum number of midpoints ahead of the car so that a failsafe is
 * not triggered.
 *  - If set to 1: the failsafe will be activated only when the car does not
 *    have any midpoints ahead.
 *  - If set to 2: the failsafe will be activated when the car has still one
 *    midpoint ahead. This option may be preferrable.
 *  - Any bigger value will trigger the failsafe sooner.
 */
constexpr uint32_t MIN_FAILSAFE_WAY_SIZE = 2;

/**
 * @brief When the car is in start position, we want the path to recalculate not
 * from car's closest midpoint but from 0 (empty Trace). To do this we need to
 * know if the car is in a starting position.
 * We will recalculate path from scratch (see Trace::trimByLocal()) basically
 * if car's outside a radius from (0,0).
 */
constexpr double CAR_HASNT_MOVED_START_RADIUS = 0.3;