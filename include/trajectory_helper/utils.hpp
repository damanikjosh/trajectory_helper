#ifndef TRAJECTORY_HELPER__UTILS_HPP
#define TRAJECTORY_HELPER__UTILS_HPP

#include <cmath>
#include <vector>
#include <algorithm>
#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track.hpp"
#include "trajectory_helper/track/track_point.hpp"

namespace th {

/**
 * A small helper to check if two doubles are "close" within a given tolerance
 */
bool isClose(double a, double b, double tol = 1e-9) {
    return std::fabs(a - b) < tol;
}

template<typename T1, typename T2>
float distance(const T1& p1, const T2& p2) {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/**
 * Checks if two 2D points (x,y) are close in both coordinates
 */
template<typename T1, typename T2>
bool pointsAreClose(const T1& p1, const T2& p2, double tol = 1e-9) {
    return (isClose(p1.x, p2.x, tol) && isClose(p1.y, p2.y, tol));
}

/**
 * Normalize heading angle to be between -π and π
 */
double normalize_psi(double psi) {
    while (psi > M_PI) psi -= 2.0 * M_PI;
    while (psi < -M_PI) psi += 2.0 * M_PI;
    return psi;
}

/**
 * Linear interpolation helper function
 * 
 * @param x   Vector of x coordinates for interpolation
 * @param y   Vector of y coordinates for interpolation
 * @param x0  x value at which to interpolate
 * @return   Interpolated y value
 */
double linear_interp(const std::vector<double>& x, 
                          const std::vector<double>& y, 
                          double x0) {
    auto it = std::lower_bound(x.begin(), x.end(), x0);
    if (it == x.begin()) return y.front();
    if (it == x.end()) return y.back();
    
    size_t i = std::distance(x.begin(), it) - 1;
    double t = (x0 - x[i]) / (x[i + 1] - x[i]);
    return y[i] + t * (y[i + 1] - y[i]);
}

size_t find_nearest_idx(const Track2f& track, const Point2f& point) {
    if (track.size() < 2) return track.empty() ? 0 : 1;
    
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;

    // Find nearest point
    for (size_t i = 0; i < track.size(); ++i) {
        double dist = std::hypot(track[i].x - point.x, track[i].y - point.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

/**
 * Calculate cumulative distances along track points
 */
template<typename T>
std::vector<T> calculate_cumulative_distances(const std::vector<TrackPoint<T>>& points) {
    std::vector<T> dists_cum(points.size(), 0);
    for (size_t i = 1; i < points.size(); ++i) {
        Point<T> diff = points[i].to_point() - points[i-1].to_point();
        dists_cum[i] = dists_cum[i-1] + std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    return dists_cum;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__UTILS_HPP