#ifndef TRAJECTORY_HELPER__CALC__TRACK_HPP
#define TRAJECTORY_HELPER__CALC__TRACK_HPP

#include <vector>
#include <cmath>
#include <numeric>
#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"

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
Track2f calc_track(
    const Track2f& track,
    bool is_closed,
    double stepsize_psi_preview = 1.0,
    double stepsize_psi_review = 1.0,
    double stepsize_curv_preview = 2.0,
    double stepsize_curv_review = 2.0,
    bool calc_curv = true)
{
    Track2f result = track;
    auto el_lengths = calc_el_lengths(track);
    
    // Calculate cumulative path length (s coordinate)
    result[0].s = 0.0;  // Initialize first point
    for (size_t i = 1; i < track.size(); ++i) {
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
        for (size_t i = 0; i < track.size(); ++i) {
            int preview_idx = (i + ind_step_preview_psi) % track.size();
            int review_idx = (i - ind_step_review_psi + track.size()) % track.size();
            
            double dx = track[preview_idx].x - track[review_idx].x;
            double dy = track[preview_idx].y - track[review_idx].y;
            result[i].psi = normalize_psi(std::atan2(dy, dx));
        }

        // Calculate curvature (kappa)
        if (calc_curv) {
            for (size_t i = 0; i < track.size(); ++i) {
                size_t preview_idx = (i + ind_step_preview_curv) % track.size();
                size_t review_idx = (i - ind_step_review_curv + track.size()) % track.size();
                
                double delta_psi = normalize_psi(result[preview_idx].psi - result[review_idx].psi);
                
                // Calculate path length between review and preview points
                double path_length = 0.0;
                for (size_t j = review_idx; j != preview_idx; j = (j + 1) % track.size()) {
                    path_length += el_lengths[j];
                }
                
                result[i].kappa = delta_psi / path_length;
            }
        }
    } else {
        // Calculate heading for open path
        for (size_t i = 0; i < track.size(); ++i) {
            double dx, dy;
            if (i == 0) {
                dx = track[1].x - track[0].x;
                dy = track[1].y - track[0].y;
            } else if (i == track.size() - 1) {
                dx = track[i].x - track[i-1].x;
                dy = track[i].y - track[i-1].y;
            } else {
                dx = track[i+1].x - track[i-1].x;
                dy = track[i+1].y - track[i-1].y;
            }
            result[i].psi = normalize_psi(std::atan2(dy, dx));
        }

        // Calculate curvature for open path
        if (calc_curv) {
            for (size_t i = 0; i < track.size(); ++i) {
                double delta_psi;
                double path_length;
                
                if (i == 0) {
                    delta_psi = normalize_psi(result[1].psi - result[0].psi);
                    path_length = el_lengths[0];
                } else if (i == track.size() - 1) {
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

#endif  // TRAJECTORY_HELPER__CALC__TRACK_HPP
