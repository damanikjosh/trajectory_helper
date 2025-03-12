#ifndef TRAJECTORY_HELPER__TRACK__CALC__TRACK_HPP
#define TRAJECTORY_HELPER__TRACK__CALC__TRACK_HPP

#include <vector>
#include <cmath>
#include <numeric>
#include "trajectory_helper/utils.hpp"
#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track.hpp"
#include "trajectory_helper/track/track_point.hpp"

namespace th {

/**
 * Calculate element lengths between track points
 */
std::vector<double> calc_el_lengths(const Track2f& track) {
    std::vector<double> el_lengths;
    el_lengths.reserve(track.size() - 1);
    
    for (size_t i = 0; i < track.size() - 1; ++i) {
        double dx = track[i+1].x - track[i].x;
        double dy = track[i+1].y - track[i].y;
        el_lengths.push_back(std::sqrt(dx*dx + dy*dy));
    }
    return el_lengths;
}

/**
 * Calculate track heading, curvature, and path length
 * 
 * @param track Input track points
 * @param is_closed Whether the track forms a closed loop
 * @param stepsize_psi_preview Preview distance for heading calculation
 * @param stepsize_psi_review Review distance for heading calculation
 * @param stepsize_curv_preview Preview distance for curvature calculation
 * @param stepsize_curv_review Review distance for curvature calculation
 * @param calc_curv Whether to calculate curvature
 * @return Updated track with calculated heading and curvature
 */
template<typename T>
Track2<T> calc_track(
    const std::vector<Point2<T>>& points,
    bool is_closed,
    double stepsize_psi_preview = 1.0,
    double stepsize_psi_review = 1.0,
    double stepsize_curv_preview = 2.0,
    double stepsize_curv_review = 2.0,
    bool calc_curv = true)
{
    Track2<T> result;
    result.reserve(points.size());
    
    // Convert points to track points
    for (const auto& p : points) {
        result.push_back(TrackPoint2<T>(p.x, p.y));
    }
    
    std::vector<double> el_lengths;
    el_lengths.reserve(points.size() - 1);
    
    for (size_t i = 0; i < points.size() - 1; ++i) {
        double dx = points[i+1].x - points[i].x;
        double dy = points[i+1].y - points[i].y;
        el_lengths.push_back(std::sqrt(dx*dx + dy*dy));
    }
    
    // Calculate cumulative path length (s coordinate)
    result[0].s = 0.0;  // Initialize first point
    for (size_t i = 1; i < points.size(); ++i) {
        result[i].s = result[i-1].s + el_lengths[i-1];
    }
    
    double avg_el_length = std::accumulate(el_lengths.begin(), el_lengths.end(), 0.0) / el_lengths.size();

    // Calculate step indices
    int ind_step_preview_psi = std::max(1, static_cast<int>(std::round(stepsize_psi_preview / avg_el_length)));
    int ind_step_review_psi = std::max(1, static_cast<int>(std::round(stepsize_psi_review / avg_el_length)));
    int ind_step_preview_curv = std::max(1, static_cast<int>(std::round(stepsize_curv_preview / avg_el_length)));
    int ind_step_review_curv = std::max(1, static_cast<int>(std::round(stepsize_curv_review / avg_el_length)));

    if (is_closed) {
        // Calculate heading (psi)
        for (size_t i = 0; i < points.size(); ++i) {
            int preview_idx = (i + ind_step_preview_psi) % points.size();
            int review_idx = (i - ind_step_review_psi + points.size()) % points.size();
            
            double dx = points[preview_idx].x - points[review_idx].x;
            double dy = points[preview_idx].y - points[review_idx].y;
            result[i].psi = normalize_psi(std::atan2(dy, dx));
        }

        // Calculate curvature (kappa)
        if (calc_curv) {
            for (size_t i = 0; i < points.size(); ++i) {
                size_t preview_idx = (i + ind_step_preview_curv) % points.size();
                size_t review_idx = (i - ind_step_review_curv + points.size()) % points.size();
                
                double delta_psi = normalize_psi(result[preview_idx].psi - result[review_idx].psi);
                
                // Calculate path length between review and preview points
                double path_length = 0.0;
                for (size_t j = review_idx; j != preview_idx; j = (j + 1) % points.size()) {
                    path_length += el_lengths[j];
                }
                
                result[i].kappa = delta_psi / path_length;
            }
        }
    } else {
        // Calculate heading for open path
        for (size_t i = 0; i < points.size(); ++i) {
            double dx, dy;
            if (i == 0) {
                dx = points[1].x - points[0].x;
                dy = points[1].y - points[0].y;
            } else if (i == points.size() - 1) {
                dx = points[i].x - points[i-1].x;
                dy = points[i].y - points[i-1].y;
            } else {
                dx = points[i+1].x - points[i-1].x;
                dy = points[i+1].y - points[i-1].y;
            }
            result[i].psi = normalize_psi(std::atan2(dy, dx));
        }

        // Calculate curvature for open path
        if (calc_curv) {
            for (size_t i = 0; i < points.size(); ++i) {
                double delta_psi;
                double path_length;
                
                if (i == 0) {
                    delta_psi = normalize_psi(result[1].psi - result[0].psi);
                    path_length = el_lengths[0];
                } else if (i == points.size() - 1) {
                    delta_psi = normalize_psi(result[i].psi - result[i-1].psi);
                    path_length = el_lengths[i-1];
                } else {
                    delta_psi = normalize_psi(result[i+1].psi - result[i-1].psi);
                    path_length = el_lengths[i] + el_lengths[i-1];
                }
                
                result[i].kappa = delta_psi / path_length;
            }
        }
    }

    return result;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__CALC__TRACK_HPP
