#ifndef TRAJECTORY_HELPER__LOOKUP_TRACK_HPP
#define TRAJECTORY_HELPER__LOOKUP_TRACK_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"
#include "trajectory_helper/interp_point.hpp"

namespace th {

TrackPoint2f lookup_track(const Track2f& track, double s) {
    if (track.empty()) {
        throw std::runtime_error("Track is empty");
    }

    // Get track length and handle wrapping
    double track_length = track.back().s;
    while (s < 0) s += track_length;
    while (s >= track_length) s -= track_length;

    // Create cumulative distances vector
    std::vector<double> dists_cum;
    dists_cum.reserve(track.size());
    for (const auto& p : track) {
        dists_cum.push_back(p.s);
    }

    // Create new track with single interpolated point
    return interp_point(track, dists_cum, s);
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__LOOKUP_TRACK_HPP