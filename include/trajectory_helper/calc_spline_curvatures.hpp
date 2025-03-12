#ifndef TRAJECTORY_HELPER__CALC_SPLINE_CURVATURES_HPP
#define TRAJECTORY_HELPER__CALC_SPLINE_CURVATURES_HPP

#include <vector>
#include <cmath>
#include <stdexcept>

namespace th {

/**
 * Calculate curvatures for given spline points.
 * 
 * @param coeffs_x     Coefficient matrix of x splines (no_splines x 4)
 * @param coeffs_y     Coefficient matrix of y splines (no_splines x 4)
 * @param ind_spls     Indices of splines containing the points
 * @param t_spls       Relative spline coordinate values for each point
 * @return            Vector of curvatures at each point
 */
std::vector<double> calc_spline_curvatures(
    const std::vector<std::vector<double>>& coeffs_x,
    const std::vector<std::vector<double>>& coeffs_y,
    const std::vector<size_t>& ind_spls,
    const std::vector<double>& t_spls)
{
    // Input validation
    if (coeffs_x.size() != coeffs_y.size()) {
        throw std::runtime_error("Coefficient matrices must have the same length!");
    }
    if (ind_spls.size() != t_spls.size()) {
        throw std::runtime_error("ind_spls and t_spls must have the same length!");
    }

    std::vector<double> kappa(ind_spls.size());

    // Calculate curvature for each point
    for (size_t i = 0; i < ind_spls.size(); ++i) {
        size_t spline_idx = ind_spls[i];
        double t = t_spls[i];
        double t2 = t * t;

        // Calculate first derivatives
        double x_d = coeffs_x[spline_idx][1] + 
                    2.0 * coeffs_x[spline_idx][2] * t + 
                    3.0 * coeffs_x[spline_idx][3] * t2;

        double y_d = coeffs_y[spline_idx][1] + 
                    2.0 * coeffs_y[spline_idx][2] * t + 
                    3.0 * coeffs_y[spline_idx][3] * t2;

        // Calculate second derivatives
        double x_dd = 2.0 * coeffs_x[spline_idx][2] + 
                     6.0 * coeffs_x[spline_idx][3] * t;

        double y_dd = 2.0 * coeffs_y[spline_idx][2] + 
                     6.0 * coeffs_y[spline_idx][3] * t;

        // Calculate curvature
        double denominator = std::pow(x_d * x_d + y_d * y_d, 1.5);
        
        if (std::abs(denominator) < 1e-10) {
            kappa[i] = 0.0;  // Handle near-zero denominator case
        } else {
            kappa[i] = (x_d * y_dd - y_d * x_dd) / denominator;
        }
    }

    return kappa;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__CALC_SPLINE_CURVATURES_HPP
