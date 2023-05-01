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
 * hashes for objects whose hash is defined by more than one number. Given
 * that the object whose hash has more elements is the triangle (3), this
 * number has been adjuested accordingly, i.e. 21*3 == 63, 63 < 64 bits.
 * It also means that the biggest cone ID MUST be less than 2^HASH_SHIFT_NUM-3.
 * The last "3" is because of the super triangle. We must be able to create it.
 */
const uint64_t HASH_SHIFT_NUM = 21;