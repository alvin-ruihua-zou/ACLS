#ifndef COMPARES_H_
#define COMPARES_H_

/**
 * @brief Compares two floating-point numbers to check if the first is greater than the second.
 *
 * @param a The first number.
 * @param b The second number.
 * @param rel_tol The relative tolerance for comparison (default: 1e-9).
 * @param abs_tol The absolute tolerance for comparison (default: 1e-12).
 * @return True if `a` is greater than `b` within the specified tolerances, false otherwise.
 */
bool isGreaterThan(double a, double b, double rel_tol = 1e-9, double abs_tol = 1e-12);

/**
 * @brief Compares two floating-point numbers to check if the first is less than the second.
 *
 * @param a The first number.
 * @param b The second number.
 * @param rel_tol The relative tolerance for comparison (default: 1e-9).
 * @param abs_tol The absolute tolerance for comparison (default: 1e-12).
 * @return True if `a` is less than `b` within the specified tolerances, false otherwise.
 */
bool isLessThan(double a, double b, double rel_tol = 1e-9, double abs_tol = 1e-12);

/**
 * @brief Checks if the first floating-point number is definitely greater than the second.
 *
 * This uses a scaled epsilon to account for the magnitude of the numbers.
 *
 * @param a The first number.
 * @param b The second number.
 * @param epsilon The scaling factor for comparison (default: 1e-12).
 * @return True if `a` is definitely greater than `b`, false otherwise.
 */
bool definitelyGreaterThan(double a, double b, double epsilon = 1e-12);

/**
 * @brief Checks if the first floating-point number is definitely less than the second.
 *
 * This uses a scaled epsilon to account for the magnitude of the numbers.
 *
 * @param a The first number.
 * @param b The second number.
 * @param epsilon The scaling factor for comparison (default: 1e-12).
 * @return True if `a` is definitely less than `b`, false otherwise.
 */
bool definitelyLessThan(double a, double b, double epsilon = 1e-12);

#endif // COMPARES_H_