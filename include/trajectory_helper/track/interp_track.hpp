#ifndef TRAJECTORY_HELPER__INTERP_TRACK_HPP
#define TRAJECTORY_HELPER__INTERP_TRACK_HPP

#include <vector>
#include <cmath>
#include <stdexcept>

#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track.hpp"
#include "trajectory_helper/track/track_point.hpp"
#include "trajectory_helper/track/interp_track_point.hpp"
#include "trajectory_helper/utils.hpp"

namespace th {

/**
 * Interpolate track points linearly to a new stepsize.
 * 
 * @param track      Track2<T> containing points and widths (unclosed)
 * @param stepsize   Desired stepsize after interpolation in m
 * @return          Interpolated track (unclosed)
 */
template<typename T>
Track2<T> interp_track(const Track2<T>& track, double stepsize) {
    if (track.size() < 2) {
        throw std::runtime_error("Track must have at least 2 points!");
    }

    // Throw error if stepsize is zero
    if (stepsize == T(0)) {
        throw std::runtime_error("Stepsize must be greater than zero.");
    }

    // Create closed track by adding first point at end
    Track2<T> track_cl = track;
    if (track.closed) {
        track_cl.push_back(track[0]);
    }

    // Calculate cumulative distances and set s coordinates
    T cum_dist = T(0);
    track_cl[0].s = T(0);
    for (size_t i = 1; i < track_cl.size(); ++i) {
        Point2<T> diff = track_cl[i].to_point() - track_cl[i-1].to_point();
        cum_dist += std::sqrt(diff.x * diff.x + diff.y * diff.y);
        track_cl[i].s = cum_dist;
    }

    // Calculate number of interpolation points
    size_t no_points_interp_cl = static_cast<size_t>(std::ceil(cum_dist / stepsize)) + 1;
    Track2<T> track_interp;

    if (track.closed) {
        track_interp.resize(no_points_interp_cl - 1);  // Remove last point to keep unclosed
    } else {
        track_interp.resize(no_points_interp_cl);
    }

    // Interpolate all track points
    for (size_t i = 0; i < track_interp.size(); ++i) {
        T dist = T(i) * cum_dist / T(no_points_interp_cl - 1);
        track_interp[i] = interp_track_point(track_cl, dist);
    }

    return track_interp;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__INTERP_TRACK_HPP
