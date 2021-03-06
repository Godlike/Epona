/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_FLOATING_POINT_HPP
#define EPONA_FLOATING_POINT_HPP

#include <glm/glm.hpp>

namespace epona
{
namespace fp
{
constexpr float g_floatingPointThreshold = 1e-6f;

/**
 *  @brief  Checks if given floating point number is equal to null within a set threshold
 *
 *  @param  n   number
 *
 *  @return @c true if a number is equal to null, @c false otherwise
 */
inline bool IsZero(float n)
{
    return glm::abs(n) < g_floatingPointThreshold;
}

/**
 *  @brief  Checks if left-hand side value is equal to the right-hand side
 *          within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if numbers are equal, @c false otherwise
 *
 *  @sa IsNotEqual
 */
inline bool IsEqual(float lhs, float rhs)
{
    return glm::abs(lhs - rhs) < g_floatingPointThreshold;
}

/**
 *  @brief  Checks if left-hand side value is not equal to the right-hand side
 *          within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if numbers are equal, @c false otherwise
 *
 *  @sa IsEqual
 */
inline bool IsNotEqual(float lhs, float rhs)
{
    return !IsEqual(lhs, rhs);
}

/**
 *  @brief  Checks if left-hand side value is less than right-hand side value
 *          within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is less than rhs, @c false otherwise
 */
inline bool IsLess(float lhs, float rhs)
{
    return !IsEqual(lhs, rhs) && (lhs - rhs) < 0.0f;
}

/**
 *  @brief  Checks if left-hand side value is greater than right-hand side value
 *          within a set threshold and returns true if so
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is greater than rhs, @c false otherwise
 */
inline bool IsGreater(float lhs, float rhs)
{
    return !IsEqual(lhs, rhs) && (lhs - rhs) > 0.0f;
}

/**
 *  @brief  Checks if left-hand side value is less than or equal to right-hand
 *          side value within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is less than or equal to rhs, @c false otherwise
 */
inline bool IsLessOrEqual(float lhs, float rhs)
{
    return !IsGreater(lhs, rhs);
}

/**
 *  @brief  Checks if left-hand side value is greater than or equal to a
 *          right-hand side value within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is greater than or equal to rhs, @c false otherwise
 */
inline bool IsGreaterOrEqual(float lhs, float rhs)
{
    return !IsLess(lhs, rhs);
}
} // namespace fp
} // namespace epona
#endif // EPONA_FLOATING_POINT_HPP
