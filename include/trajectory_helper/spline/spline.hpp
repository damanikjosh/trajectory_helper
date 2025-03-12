#ifndef TRAJECTORY_HELPER__SPLINE_HPP
#define TRAJECTORY_HELPER__SPLINE_HPP

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>

namespace th {

/**
 * @brief A general polynomial spline interpolator using Eigen3.
 */
template<typename T>
class Spline {
public:
    std::vector<T> s, y; // Input data
    Eigen::MatrixXd coefficients; // Coefficients for each segment
    int degree;

    /**
     * @brief Constructor that allows specifying the polynomial degree.
     * @param s_vals - x-axis values (sorted)
     * @param y_vals - y-axis values
     * @param degree - Degree of the polynomial spline (default: 3 for cubic)
     */
    Spline(const std::vector<T>& s_vals, const std::vector<T>& y_vals, int degree = 3)
        : degree(degree) {
        if (s_vals.size() < degree + 1 || s_vals.size() != y_vals.size()) {
            throw std::runtime_error("Invalid input sizes for polynomial spline.");
        }

        s = s_vals;
        y = y_vals;
        size_t n = s.size() - 1; // Number of segments

        Eigen::MatrixXd A(n * (degree + 1), n * (degree + 1)); // Coefficient matrix
        Eigen::VectorXd B(n * (degree + 1)); // Right-hand side vector
        A.setZero();
        B.setZero();

        // Fill the system of equations
        int row = 0;
        for (size_t i = 0; i < n; ++i) {
            // Interpolation constraints at segment endpoints
            for (int j = 0; j <= degree; ++j) {
                A(row, i * (degree + 1) + j) = std::pow(s[i], j);
            }
            B(row++) = y[i];

            for (int j = 0; j <= degree; ++j) {
                A(row, i * (degree + 1) + j) = std::pow(s[i + 1], j);
            }
            B(row++) = y[i + 1];

            // Continuity constraints (1st derivative)
            if (i < n - 1) {
                for (int j = 1; j <= degree; ++j) {
                    A(row, i * (degree + 1) + j) = j * std::pow(s[i + 1], j - 1);
                    A(row, (i + 1) * (degree + 1) + j) = -j * std::pow(s[i + 1], j - 1);
                }
                B(row++) = 0;
            }
        }

        // Solve Ax = B
        Eigen::VectorXd solution = A.colPivHouseholderQr().solve(B);
        coefficients = Eigen::Map<Eigen::MatrixXd>(solution.data(), degree + 1, n).transpose();
    }

    /**
     * @brief Evaluate the spline at a given query point s_query.
     */
    T evaluate(T s_query) const {
        if (s_query <= s.front()) return y.front();
        if (s_query >= s.back()) return y.back();

        // Find the segment index
        size_t i = 0;
        while (i < s.size() - 1 && s_query > s[i + 1]) ++i;

        // Compute polynomial evaluation
        T result = 0;
        for (int j = 0; j <= degree; ++j) {
            result += coefficients(i, j) * std::pow(s_query, j);
        }
        return result;
    }
};

// Common type definitions
typedef Spline<int> Splinei;
typedef Spline<float> Splinef;
typedef Spline<double> Splined;

}  // namespace th

#endif  // TRAJECTORY_HELPER__SPLINE_HPP
