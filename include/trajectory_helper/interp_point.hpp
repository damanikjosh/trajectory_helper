#ifndef TRAJECTORY_HELPER__INTERP_POINT_HPP
#define TRAJECTORY_HELPER__INTERP_POINT_HPP

#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"

namespace th {

template<typename T>
T interpolate_property(const Track2f& track, 
                      const std::vector<double>& dists_cum,
                      double dist,
                      T TrackPoint2f::*property,
                      bool is_angle = false) {
    std::vector<T> values;
    values.reserve(track.size());
    for (const auto& p : track) {
        values.push_back(p.*property);
    }
    
    if (is_angle) {
        // Find the two points to interpolate between
        auto it = std::lower_bound(dists_cum.begin(), dists_cum.end(), dist);
        if (it == dists_cum.end()) return values.back();
        if (it == dists_cum.begin()) return values.front();
        
        size_t idx2 = std::distance(dists_cum.begin(), it);
        size_t idx1 = idx2 - 1;
        
        double t = (dist - dists_cum[idx1]) / (dists_cum[idx2] - dists_cum[idx1]);
        double angle1 = values[idx1];
        double angle2 = values[idx2];
        
        // Ensure the difference between angles is in [-π, π]
        double diff = angle2 - angle1;
        if (diff > M_PI) diff -= 2 * M_PI;
        if (diff < -M_PI) diff += 2 * M_PI;
        
        return normalize_psi(angle1 + t * diff);
    }
    
    return linear_interp(dists_cum, values, dist);
}

/**
 * Interpolate a single track point at a given distance using linear interpolation.
 * 
 * @param track      Track containing the points to interpolate between
 * @param dists_cum  Cumulative distances along the track
 * @param dist       Distance at which to interpolate
 * @return          Interpolated track point
 */
TrackPoint2f interp_point(const Track2f& track,
                         const std::vector<double>& dists_cum,
                         double dist) {
    TrackPoint2f point;
    
    point.s = interpolate_property(track, dists_cum, dist, &TrackPoint2f::s);

    // Basic properties (x, y) always present
    point.x = interpolate_property(track, dists_cum, dist, &TrackPoint2f::x);
    point.y = interpolate_property(track, dists_cum, dist, &TrackPoint2f::y);

    // Optional properties
    if (track.has_psi()) {
        point.psi = normalize_psi(interpolate_property(track, dists_cum, dist, &TrackPoint2f::psi, true));
    }
    if (track.has_widths()) {
        point.wr = interpolate_property(track, dists_cum, dist, &TrackPoint2f::wr);
        point.wl = interpolate_property(track, dists_cum, dist, &TrackPoint2f::wl);
    }
    if (track.has_kappa()) {
        point.kappa = interpolate_property(track, dists_cum, dist, &TrackPoint2f::kappa);
    }

    return point;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__INTERP_POINT_HPP
