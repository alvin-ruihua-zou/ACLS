#include "compares.h"
#include <cmath>
#include <algorithm>

/**
 * @brief Compares two floating-point numbers to check if the first is less than the second.
 *
 * The comparison uses both relative and absolute tolerances to handle floating-point inaccuracies.
 *
 * @param a The first number.
 * @param b The second number.
 * @param rel_tol The relative tolerance for comparison.
 * @param abs_tol The absolute tolerance for comparison.
 * @return True if `a` is less than `b` within the specified tolerances, false otherwise.
 */
bool isLessThan(double a, double b, double rel_tol, double abs_tol)
{
    return (b - a > std::max(rel_tol * std::max(std::fabs(a), std::fabs(b)), abs_tol));
}

/**
 * @brief Compares two floating-point numbers to check if the first is greater than the second.
 *
 * The comparison uses both relative and absolute tolerances to handle floating-point inaccuracies.
 *
 * @param a The first number.
 * @param b The second number.
 * @param rel_tol The relative tolerance for comparison.
 * @param abs_tol The absolute tolerance for comparison.
 * @return True if `a` is greater than `b` within the specified tolerances, false otherwise.
 */
bool isGreaterThan(double a, double b, double rel_tol, double abs_tol)
{
    return (a - b > std::max(rel_tol * std::max(std::fabs(a), std::fabs(b)), abs_tol));
}

/**
 * @brief Checks if the first floating-point number is definitely greater than the second.
 *
 * This uses a scaled epsilon to account for the magnitude of the numbers.
 *
 * @param a The first number.
 * @param b The second number.
 * @param epsilon The scaling factor for comparison.
 * @return True if `a` is definitely greater than `b`, false otherwise.
 */
bool definitelyGreaterThan(double a, double b, double epsilon)
{
    return (a - b) > ((std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
}

/**
 * @brief Checks if the first floating-point number is definitely less than the second.
 *
 * This uses a scaled epsilon to account for the magnitude of the numbers.
 *
 * @param a The first number.
 * @param b The second number.
 * @param epsilon The scaling factor for comparison.
 * @return True if `a` is definitely less than `b`, false otherwise.
 */
bool definitelyLessThan(double a, double b, double epsilon)
{
    return (b - a) > ((std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
}