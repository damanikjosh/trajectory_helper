#ifndef TRAJECTORY_HELPER__TRACK__TRACK_HPP
#define TRAJECTORY_HELPER__TRACK__TRACK_HPP

#include <vector>
#include <stdexcept>
#include <cmath>
#include <numeric>

#include "trajectory_helper/utils.hpp"
#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track_point.hpp"

namespace th {

template<typename T>
class Track2;  // Forward declaration

/**
 * Calculate element lengths between track points
 */
template<typename T>
std::vector<T> calc_el_lengths(const Track2<T>& track) {
    std::vector<T> el_lengths;
    el_lengths.reserve(track.size() - 1);
    
    for (size_t i = 0; i < track.size() - 1; ++i) {
        T dx = track[i+1].x - track[i].x;
        T dy = track[i+1].y - track[i].y;
        el_lengths.push_back(std::sqrt(dx*dx + dy*dy));
    }
    return el_lengths;
}

/**
 * Calculate track heading, curvature, and path length
 */
template<typename T>
Track2<T> calc_track(
    const std::vector<Point2<T>>& points,
    bool is_closed,
    double stepsize_psi_preview = 1.0,
    double stepsize_psi_review = 1.0,
    double stepsize_curv_preview = 2.0,
    double stepsize_curv_review = 2.0,
    bool calc_curv = true);  // Forward declaration

template<typename T>
class Track2 : public std::vector<TrackPoint2<T>> {
public:
    // Inherit vector constructors
    using std::vector<TrackPoint2<T>>::vector;

    Track2(const std::vector<Point2<T>>& points,
           bool is_closed = false,
           double stepsize_psi_preview = 1.0,
           double stepsize_psi_review = 1.0,
           double stepsize_curv_preview = 2.0,
           double stepsize_curv_review = 2.0,
           bool calc_curv = true) {
        *this = calc_track(points, is_closed, stepsize_psi_preview, stepsize_psi_review, stepsize_curv_preview, stepsize_curv_review, calc_curv);
        closed = is_closed;
    }

    std::vector<T> s() const {
        std::vector<T> s_vals;
        s_vals.reserve(this->size());
        for (const auto& p : *this) {
            s_vals.push_back(p.s);
        }
        return s_vals;
    }

    std::vector<T> x() const {
        std::vector<T> x_vals;
        x_vals.reserve(this->size());
        for (const auto& p : *this) {
            x_vals.push_back(p.x);
        }
        return x_vals;
    }

    std::vector<T> y() const {
        std::vector<T> y_vals;
        y_vals.reserve(this->size());
        for (const auto& p : *this) {
            y_vals.push_back(p.y);
        }
        return y_vals;
    }

    std::vector<T> psi() const {
        std::vector<T> psi_vals;
        psi_vals.reserve(this->size());
        for (const auto& p : *this) {
            psi_vals.push_back(p.psi);
        }
        return psi_vals;
    }

    std::vector<T> kappa() const {
        std::vector<T> kappa_vals;
        kappa_vals.reserve(this->size());
        for (const auto& p : *this) {
            kappa_vals.push_back(p.kappa);
        }
        return kappa_vals;
    }

    std::vector<T> wr() const {
        std::vector<T> wr_vals;
        wr_vals.reserve(this->size());
        for (const auto& p : *this) {
            wr_vals.push_back(p.wr);
        }
        return wr_vals;
    }

    std::vector<T> wl() const {
        std::vector<T> wl_vals;
        wl_vals.reserve(this->size());
        for (const auto& p : *this) {
            wl_vals.push_back(p.wl);
        }
        return wl_vals;
    }

    void set_widths(std::vector<T> wl, std::vector<T> wr) {
        if (wl.size() != wr.size() || wl.size() != this->size()) {
            throw std::runtime_error("Width vectors must have the same size as the track.");
        }
        for (size_t i = 0; i < this->size(); ++i) {
            this->at(i).wl = wl[i];
            this->at(i).wr = wr[i];
        }
    }
    
    bool has_psi() const {
        if (this->empty()) return false;
        // Check if the value is not infinity
        return !std::isinf(this->front().psi);
    }

    bool has_kappa() const {
        if (this->empty()) return false;
        return !std::isinf(this->front().kappa);
    }

    bool has_widths() const {
        if (this->empty()) return false;
        return !std::isinf(this->front().wl) && !std::isinf(this->front().wr);
    }

    bool closed = false;
};

typedef Track2<int> Track2i;
typedef Track2<float> Track2f;
typedef Track2<double> Track2d;

// Implementation of calc_track after Track2 is fully defined
template<typename T>
Track2<T> calc_track(
    const std::vector<Point2<T>>& points,
    bool is_closed,
    double stepsize_psi_preview,
    double stepsize_psi_review,
    double stepsize_curv_preview,
    double stepsize_curv_review,
    bool calc_curv)
{
    // Make sure there are enough points
    if (points.size() < 2) {
        throw std::runtime_error("Track must have at least 2 points!");
    }
    
    Track2<T> result;
    result.reserve(points.size());
    
    // Convert points to track points
    for (const auto& p : points) {
        result.push_back(TrackPoint2<T>(p.x, p.y));
    }
    
    std::vector<T> el_lengths;
    el_lengths.reserve(points.size() - 1);
    
    for (size_t i = 0; i < points.size() - 1; ++i) {
        T dx = points[i+1].x - points[i].x;
        T dy = points[i+1].y - points[i].y;
        el_lengths.push_back(std::sqrt(dx*dx + dy*dy));
    }
    
    // Calculate cumulative path length (s coordinate)
    result[0].s = T();  // Initialize first point
    for (size_t i = 1; i < points.size(); ++i) {
        result[i].s = result[i-1].s + el_lengths[i-1];
    }
    
    T avg_el_length = std::accumulate(el_lengths.begin(), el_lengths.end(), T()) / static_cast<T>(el_lengths.size());

    // Calculate step indices using T for calculations
    int ind_step_preview_psi = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_psi_preview) / avg_el_length)));
    int ind_step_review_psi = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_psi_review) / avg_el_length)));
    int ind_step_preview_curv = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_curv_preview) / avg_el_length)));
    int ind_step_review_curv = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_curv_review) / avg_el_length)));

    if (is_closed) {
        // Calculate heading (psi)
        for (size_t i = 0; i < points.size(); ++i) {
            int preview_idx = (i + ind_step_preview_psi) % points.size();
            int review_idx = (i - ind_step_review_psi + points.size()) % points.size();
            
            T dx = points[preview_idx].x - points[review_idx].x;
            T dy = points[preview_idx].y - points[review_idx].y;
            result[i].psi = normalize_psi(std::atan2(dy, dx));
        }

        // Calculate curvature (kappa)
        if (calc_curv) {
            for (size_t i = 0; i < points.size(); ++i) {
                size_t preview_idx = (i + ind_step_preview_curv) % points.size();
                size_t review_idx = (i - ind_step_review_curv + points.size()) % points.size();
                
                T delta_psi = normalize_psi(result[preview_idx].psi - result[review_idx].psi);
                
                // Calculate path length between review and preview points
                T path_length = T();
                for (size_t j = review_idx; j != preview_idx; j = (j + 1) % points.size()) {
                    path_length += el_lengths[j];
                }
                
                result[i].kappa = delta_psi / path_length;
            }
        }
    } else {
        // Calculate heading for open path
        for (size_t i = 0; i < points.size(); ++i) {
            T dx, dy;
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
                T delta_psi;
                T path_length;
                
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


template<typename T>
size_t find_nearest_idx(const Track2<T>& track, const Point2<T>& point) {
    if (track.size() < 2) return track.empty() ? 0 : 1;
    
    T min_dist = std::numeric_limits<T>::max();
    size_t nearest_idx = 0;

    // Find nearest point
    for (size_t i = 0; i < track.size(); ++i) {
        T dist = std::hypot(track[i].x - point.x, track[i].y - point.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

template<typename T>
std::vector<size_t> find_k_nearest_idx(const Track2<T>& track, const Point2<T>& point, int k) {
    if (track.size() < 2) return track.empty() ? std::vector<size_t>() : std::vector<size_t>({0});
    
    std::vector<T> dists;
    dists.reserve(track.size());
    
    // Calculate distances to all points
    for (const auto& p : track) {
        dists.push_back(std::hypot(p.x - point.x, p.y - point.y));
    }
    
    // Find k nearest points
    std::vector<size_t> nearest_k;
    for (int i = 0; i < k; ++i) {
        auto it = std::min_element(dists.begin(), dists.end());
        nearest_k.push_back(std::distance(dists.begin(), it));
        *it = std::numeric_limits<T>::max();
    }
    
    return nearest_k;
}


}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__TRACK_HPP