#ifndef TRAJECTORY_HELPER__CALC_SPLINE_LENGTHS_HPP
#define TRAJECTORY_HELPER__CALC_SPLINE_LENGTHS_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include "trajectory_helper/types.hpp"

namespace th {

/**
 * Calculate spline lengths for third order splines defining x- and y-coordinates.
 *
 * @param coeffs_x          Coefficient matrix of x splines (no_splines x 4)
 * @param coeffs_y          Coefficient matrix of y splines (no_splines x 4)
 * @param quickndirty      If true, returns lengths based on distance between first and last point
 * @param no_interp_points Number of interpolation points for length calculation
 * @return                 Vector containing the length of every spline segment
 */
std::vector<double> calc_spline_lengths(
    const std::vector<std::vector<double>>& coeffs_x,
    const std::vector<std::vector<double>>& coeffs_y,
    bool quickndirty = false,
    int no_interp_points = 15)
{
    // Input validation
    if (coeffs_x.size() != coeffs_y.size()) {
        throw std::runtime_error("Coefficient matrices must have the same length!");
    }

    for (const auto& row : coeffs_x) {
        if (row.size() != 4) throw std::runtime_error("X coefficients must have 4 columns!");
    }
    for (const auto& row : coeffs_y) {
        if (row.size() != 4) throw std::runtime_error("Y coefficients must have 4 columns!");
    }

    const size_t no_splines = coeffs_x.size();
    std::vector<double> spline_lengths(no_splines, 0.0);

    if (quickndirty) {
        // Quick calculation using endpoints
        for (size_t i = 0; i < no_splines; ++i) {
            double x_diff = coeffs_x[i][0] + coeffs_x[i][1] + coeffs_x[i][2] + coeffs_x[i][3] - coeffs_x[i][0];
            double y_diff = coeffs_y[i][0] + coeffs_y[i][1] + coeffs_y[i][2] + coeffs_y[i][3] - coeffs_y[i][0];
            spline_lengths[i] = std::sqrt(x_diff * x_diff + y_diff * y_diff);
        }
    } else {
        // Interpolation-based calculation
        std::vector<double> t_steps(no_interp_points);
        double step = 1.0 / (no_interp_points - 1);
        for (int i = 0; i < no_interp_points; ++i) {
            t_steps[i] = i * step;
        }

        // Temporary storage for spline coordinates
        std::vector<Point2f> spl_coords(no_interp_points);

        for (size_t i = 0; i < no_splines; ++i) {
            // Calculate coordinates at each interpolation point
            for (int j = 0; j < no_interp_points; ++j) {
                double t = t_steps[j];
                double t2 = t * t;
                double t3 = t2 * t;

                spl_coords[j] = Point2f(
                    coeffs_x[i][0] + coeffs_x[i][1] * t + coeffs_x[i][2] * t2 + coeffs_x[i][3] * t3,
                    coeffs_y[i][0] + coeffs_y[i][1] * t + coeffs_y[i][2] * t2 + coeffs_y[i][3] * t3
                );
            }

            // Sum up distances between consecutive points
            double length = 0.0;
            for (int j = 1; j < no_interp_points; ++j) {
                Point2f diff = spl_coords[j] - spl_coords[j-1];
                length += std::sqrt(diff.x * diff.x + diff.y * diff.y);
            }
            spline_lengths[i] = length;
        }
    }

    return spline_lengths;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__CALC_SPLINE_LENGTHS_HPP
