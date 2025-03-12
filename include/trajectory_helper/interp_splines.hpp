#ifndef TRAJECTORY_HELPER__INTERP_SPLINES_HPP
#define TRAJECTORY_HELPER__INTERP_SPLINES_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include "trajectory_helper/types.hpp"
#include "trajectory_helper/calc_spline_lengths.hpp"

namespace th {

struct InterpResult {
    std::vector<Point2f> path_interp;      // Interpolated path points
    std::vector<size_t> spline_inds;       // Indices of splines containing the points
    std::vector<double> t_values;          // Relative spline coordinate values
    std::vector<double> dists_interp;      // Total distance up to every point (optional)
};

/**
 * Interpolate points on one or more cubic splines.
 * 
 * @param coeffs_x         Coefficient matrix of x splines (no_splines x 4)
 * @param coeffs_y         Coefficient matrix of y splines (no_splines x 4)
 * @param spline_lengths   Optional pre-calculated spline lengths
 * @param incl_last_point  Include last point (t = 1.0)
 * @param stepsize_approx  Desired stepsize between points (use either this or stepnum_fixed)
 * @param stepnum_fixed    Fixed number of points per spline (use either this or stepsize_approx)
 * @return                 Interpolation results containing path points and metadata
 */
InterpResult interp_splines(
    const std::vector<std::vector<double>>& coeffs_x,
    const std::vector<std::vector<double>>& coeffs_y,
    const std::vector<double>* spline_lengths = nullptr,
    bool incl_last_point = false,
    double stepsize_approx = 0.0,
    const std::vector<int>* stepnum_fixed = nullptr)
{
    // Input validation
    if (coeffs_x.size() != coeffs_y.size()) {
        throw std::runtime_error("Coefficient matrices must have the same length!");
    }
    
    if (spline_lengths && coeffs_x.size() != spline_lengths->size()) {
        throw std::runtime_error("coeffs_x/y and spline_lengths must have the same length!");
    }

    if ((!stepsize_approx && !stepnum_fixed) || (stepsize_approx && stepnum_fixed)) {
        throw std::runtime_error("Provide either stepsize_approx or stepnum_fixed!");
    }

    if (stepnum_fixed && stepnum_fixed->size() != coeffs_x.size()) {
        throw std::runtime_error("stepnum_fixed must have an entry for every spline!");
    }

    InterpResult result;
    const size_t no_splines = coeffs_x.size();

    if (stepsize_approx > 0) {
        // Approximate equal step size interpolation
        std::vector<double> lengths = spline_lengths ? *spline_lengths : 
                                    calc_spline_lengths(coeffs_x, coeffs_y);
        
        // Calculate cumulative distances
        std::vector<double> dists_cum(no_splines);
        dists_cum[0] = lengths[0];
        for (size_t i = 1; i < no_splines; ++i) {
            dists_cum[i] = dists_cum[i-1] + lengths[i];
        }

        // Calculate number of interpolation points
        size_t no_points = static_cast<size_t>(std::ceil(dists_cum.back() / stepsize_approx)) + 1;
        result.path_interp.resize(no_points);
        result.spline_inds.resize(no_points);
        result.t_values.resize(no_points);
        result.dists_interp.resize(no_points);

        // Create equally spaced points
        double total_dist = dists_cum.back();
        for (size_t i = 0; i < no_points - 1; ++i) {
            result.dists_interp[i] = i * stepsize_approx;
            
            // Find spline containing this point
            size_t j = 0;
            while (j < no_splines && result.dists_interp[i] >= dists_cum[j]) ++j;
            result.spline_inds[i] = j;

            // Calculate t value
            double prev_dist = j > 0 ? dists_cum[j-1] : 0.0;
            result.t_values[i] = (result.dists_interp[i] - prev_dist) / lengths[j];

            // Calculate point coordinates
            double t = result.t_values[i];
            double t2 = t * t;
            double t3 = t2 * t;
            result.path_interp[i] = Point2f(
                coeffs_x[j][0] + coeffs_x[j][1] * t + coeffs_x[j][2] * t2 + coeffs_x[j][3] * t3,
                coeffs_y[j][0] + coeffs_y[j][1] * t + coeffs_y[j][2] * t2 + coeffs_y[j][3] * t3
            );
        }
    } else {
        // Fixed number of points per spline
        size_t total_points = 0;
        for (size_t i = 0; i < no_splines; ++i) {
            total_points += (*stepnum_fixed)[i];
            if (i < no_splines - 1) total_points--; // Remove overlapping points
        }

        result.path_interp.resize(total_points);
        result.spline_inds.resize(total_points);
        result.t_values.resize(total_points);

        size_t point_idx = 0;
        for (size_t i = 0; i < no_splines; ++i) {
            int points_this_spline = (*stepnum_fixed)[i];
            if (i < no_splines - 1) points_this_spline--; // Remove last point except for last spline

            for (int j = 0; j < points_this_spline; ++j) {
                double t = static_cast<double>(j) / ((*stepnum_fixed)[i] - 1);
                result.t_values[point_idx] = t;
                result.spline_inds[point_idx] = i;

                double t2 = t * t;
                double t3 = t2 * t;
                result.path_interp[point_idx] = Point2f(
                    coeffs_x[i][0] + coeffs_x[i][1] * t + coeffs_x[i][2] * t2 + coeffs_x[i][3] * t3,
                    coeffs_y[i][0] + coeffs_y[i][1] * t + coeffs_y[i][2] * t2 + coeffs_y[i][3] * t3
                );
                point_idx++;
            }
        }
    }

    // Handle last point if requested
    if (!incl_last_point && !result.path_interp.empty()) {
        result.path_interp.pop_back();
        result.spline_inds.pop_back();
        result.t_values.pop_back();
        if (!result.dists_interp.empty()) result.dists_interp.pop_back();
    }

    return result;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__INTERP_SPLINES_HPP
