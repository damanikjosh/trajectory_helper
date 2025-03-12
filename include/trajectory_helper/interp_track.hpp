#ifndef TRAJECTORY_HELPER__INTERP_TRACK_HPP
#define TRAJECTORY_HELPER__INTERP_TRACK_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"
#include "trajectory_helper/interp_point.hpp"

namespace th {

/**
 * Interpolate track points linearly to a new stepsize.
 * 
 * @param track      Track2f containing points and widths (unclosed)
 * @param stepsize   Desired stepsize after interpolation in m
 * @return          Interpolated track (unclosed)
 */
Track2f interp_track(const Track2f& track, double stepsize) {
    if (track.size() < 2) {
        throw std::runtime_error("Track must have at least 2 points!");
    }

    // Create closed track by adding first point at end
    Track2f track_cl = track;
    track_cl.push_back(track[0]);

    // Calculate element lengths
    std::vector<double> el_lengths_cl;
    el_lengths_cl.reserve(track_cl.size() - 1);
    
    for (size_t i = 0; i < track_cl.size() - 1; ++i) {
        Point2f diff = track_cl[i+1].to_point() - track_cl[i].to_point();
        el_lengths_cl.push_back(std::sqrt(diff.x * diff.x + diff.y * diff.y));
    }

    // Calculate cumulative distances
    std::vector<double> dists_cum_cl(track_cl.size(), 0.0);
    for (size_t i = 1; i < track_cl.size(); ++i) {
        dists_cum_cl[i] = dists_cum_cl[i-1] + el_lengths_cl[i-1];
    }

    // Calculate number of interpolation points
    size_t no_points_interp_cl = static_cast<size_t>(std::ceil(dists_cum_cl.back() / stepsize)) + 1;
    std::vector<double> dists_interp_cl(no_points_interp_cl);
    double total_dist = dists_cum_cl.back();
    
    for (size_t i = 0; i < no_points_interp_cl; ++i) {
        dists_interp_cl[i] = i * total_dist / (no_points_interp_cl - 1);
    }

    // Create and interpolate new track
    Track2f track_interp;
    track_interp.resize(no_points_interp_cl - 1);  // Remove last point to keep unclosed

    // Interpolate all track points
    for (size_t i = 0; i < track_interp.size(); ++i) {
        track_interp[i] = interp_point(track_cl, dists_cum_cl, dists_interp_cl[i]);
    }

    return track_interp;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__INTERP_TRACK_HPP
