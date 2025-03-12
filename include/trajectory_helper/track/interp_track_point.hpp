#ifndef TRAJECTORY_HELPER__INTERP_POINT_HPP
#define TRAJECTORY_HELPER__INTERP_POINT_HPP

#include <iostream>

#include "trajectory_helper/track/track.hpp"
#include "trajectory_helper/track/track_point.hpp"

namespace th {

/**
 * Interpolate a single track point at a given distance using linear interpolation.
 * 
 * @param track  Track containing the points to interpolate between
 * @param dist   Distance at which to interpolate
 * @return       Interpolated track point
 */
template<typename T>
TrackPoint2<T> interp_track_point(const Track2<T>& track, T dist) {
    // Show warning if the dist is outside the track
    if (dist < track.front().s || dist > track.back().s) {
        std::cerr << "Warning: Interpolating outside track bounds." << std::endl;
    }
    // Find the two points to interpolate between
    auto s_vals = track.s();
    auto it = std::lower_bound(s_vals.begin(), s_vals.end(), dist);
    
    // Handle edge cases
    if (it == s_vals.end()) return track.back();
    if (it == s_vals.begin()) return track.front();
    
    size_t idx2 = std::distance(s_vals.begin(), it);
    size_t idx1 = idx2 - 1;
    
    const auto& p1 = track[idx1];
    const auto& p2 = track[idx2];
    
    // Calculate interpolation parameter
    T t = (dist - p1.s) / (p2.s - p1.s);
    
    TrackPoint2<T> point;
    // Interpolate all properties
    point.s = dist;  // Use exact distance
    point.x = p1.x + t * (p2.x - p1.x);
    point.y = p1.y + t * (p2.y - p1.y);
    
    if (track.has_psi()) {
        T diff = p2.psi - p1.psi;
        if (diff > T(M_PI)) diff -= T(2 * M_PI);
        if (diff < T(-M_PI)) diff += T(2 * M_PI);
        point.psi = normalize_psi(p1.psi + t * diff);
    }
    
    if (track.has_widths()) {
        point.wr = p1.wr + t * (p2.wr - p1.wr);
        point.wl = p1.wl + t * (p2.wl - p1.wl);
    }
    
    if (track.has_kappa()) {
        point.kappa = p1.kappa + t * (p2.kappa - p1.kappa);
    }

    return point;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__INTERP_POINT_HPP
