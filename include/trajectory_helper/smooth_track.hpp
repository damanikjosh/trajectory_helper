#ifndef TRAJECTORY_HELPER__SMOOTH_TRACK_HPP
#define TRAJECTORY_HELPER__SMOOTH_TRACK_HPP

#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <cmath>
#include <vector>
#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"
#include "trajectory_helper/interp_track.hpp"

namespace th {

/**
 * Smooth a track using spline approximation.
 * Template type T is automatically deduced from input track type.
 */
template<typename T>
Track2<T> smooth_track(const Track2<T>& track,
                    int k_reg = 3,
                    typename Track2<T>::value_type s_reg = 10.0,
                    typename Track2<T>::value_type stepsize_prep = 1.0,
                    typename Track2<T>::value_type stepsize_reg = 3.0,
                    bool debug = false) {
    
    // Linear interpolation before smoothing
    Track2<T> track_interp = interp_track(track, stepsize_prep);
    
    // Create closed track by adding first point at end
    Track2<T> track_cl = track_interp;
    // track_cl.push_back(track_interp.front());

    Track2<T> extended_track_cl;
    size_t half_size = track_cl.size() / 2;

    // Extend track from half size to end
    for (size_t i = half_size; i < track_cl.size(); ++i) {
        extended_track_cl.push_back(track_cl[i]);
    }

    // Extend the original track full size
    for (size_t i = 0; i < track_cl.size(); ++i) {
        extended_track_cl.push_back(track_cl[i]);
    }

    // Extend track from start to half size
    for (size_t i = 0; i < half_size; ++i) {
        extended_track_cl.push_back(track_cl[i]);
    }

    // Calculate cumulative distances
    std::vector<T> dists_cum = calculate_cumulative_distances(track_cl);
    std::vector<T> extended_dists_cum = calculate_cumulative_distances(extended_track_cl);
    T extended_total_length = extended_dists_cum.back();

    // Prepare data for B-spline interpolation
    std::vector<T> x_coords, y_coords;
    x_coords.reserve(extended_track_cl.size());
    y_coords.reserve(extended_track_cl.size());
    
    for (const auto& p : extended_track_cl) {
        x_coords.push_back(p.x);
        y_coords.push_back(p.y);
    }

    // Create cubic B-splines for x and y coordinates
    // Use parameter space [0, 1] for interpolation
    T t0 = T(0.0);
    T h = T(1.0) / (extended_track_cl.size() - 1);
    
    boost::math::interpolators::cardinal_cubic_b_spline<T> spline_x(
        x_coords.data(), x_coords.size(), t0, h);
    boost::math::interpolators::cardinal_cubic_b_spline<T> spline_y(
        y_coords.data(), y_coords.size(), t0, h);

    // Create smoothed track with regular spacing
    Track2<T> smoothed_track;
    size_t extended_num_points = static_cast<size_t>(std::ceil(extended_total_length / stepsize_reg));

    size_t start_idx = extended_num_points / 4;
    size_t end_idx = 3 * extended_num_points / 4 + (stepsize_prep / stepsize_reg) / 2 - 1;

    for (size_t i = start_idx; i < end_idx; ++i) {
        T t = static_cast<T>(i) / extended_num_points;
        
        TrackPoint2<T> point;
        // Evaluate splines at parameter t
        point.x = spline_x(t);
        point.y = spline_y(t);

        // Handle track widths if present
        if (track.has_widths()) {
            // Find closest original track segment for width interpolation
            size_t idx = find_nearest_idx(extended_track_cl, point.to_point());
            T segment_t = T(0.0);
            
            if (idx < extended_track_cl.size() - 1) {
                Point2<T> v1 = extended_track_cl[idx].to_point();
                Point2<T> v2 = extended_track_cl[idx + 1].to_point();
                Point2<T> v = point.to_point() - v1;
                Point2<T> dir = v2 - v1;
                T len_sq = dir.dot(dir);
                if (len_sq > 0) {
                    segment_t = std::clamp(v.dot(dir) / len_sq, T(0.0), T(1.0));
                }
            }

            // Interpolate widths
            point.wr = extended_track_cl[idx].wr + segment_t * (extended_track_cl[idx + 1].wr - extended_track_cl[idx].wr);
            point.wl = extended_track_cl[idx].wl + segment_t * (extended_track_cl[idx + 1].wl - extended_track_cl[idx].wl);
        }

        smoothed_track.push_back(point);
    }

    if (debug) {
        // Calculate mean deviation between original and smoothed track
        T total_deviation = T(0.0);
        T max_deviation = T(0.0);
        for (const auto& p : track) {
            size_t nearest_idx = find_nearest_idx(smoothed_track, p.to_point());
            T dev = distance(p, smoothed_track[nearest_idx]);
            total_deviation += dev;
            max_deviation = std::max(max_deviation, dev);
        }
        T mean_deviation = total_deviation / track.size();
        
        // Print debug information
        std::cout << "Spline approximation: mean deviation " << mean_deviation 
                  << "m, maximum deviation " << max_deviation << "m" << std::endl;
    }

    return smoothed_track;
}

// Remove the typedefs as they're no longer needed
// The function can be called directly with any Track2 type:
// auto smoothed = smooth_track(track2f);  // for Track2f
// auto smoothed = smooth_track(track2d);  // for Track2d

}  // namespace th

#endif  // TRAJECTORY_HELPER__SMOOTH_TRACK_HPP
