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
class Track2 : public std::vector<TrackPoint2<T>> {
public:
    // Inherit vector constructors
    using std::vector<TrackPoint2<T>>::vector;
    
    explicit Track2(const std::vector<Point2<T>>& points)
    {
        if (points.size() < 2) {
            throw std::runtime_error("Track must have at least 2 points!");
        }
        this->reserve(points.size());
        for (const auto& p : points) {
            this->push_back(TrackPoint2<T>(p));
        }
    }

    explicit Track2(const std::vector<TrackPoint2<T>>& track_points)
    : std::vector<TrackPoint2<T>>(track_points)
    {
        if (track_points.size() < 2) {
            throw std::runtime_error("Track must have at least 2 points!");
        }
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

    bool has_s() const {
        if (this->empty()) return false;
        return !std::isinf(this->front().s);
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

    void calculate(
        bool is_closed = true,
        double stepsize_psi_preview = 1.0,
        double stepsize_psi_review = 1.0,
        double stepsize_curv_preview = 1.0,
        double stepsize_curv_review = 1.0,
        bool calc_curv = true)
    {
        if (this->size() < 2) {
            throw std::runtime_error("Track must have at least 2 points!");
        }
        
        // 1) Build el_lengths for consecutive edges
        std::vector<T> el_lengths;
        el_lengths.reserve(this->size() - 1);
        for (size_t i = 0; i < this->size() - 1; ++i) {
            el_lengths.push_back(distance((*this)[i], (*this)[i + 1]));
        }

        // 2) If the track is closed, add the last→first edge
        if (is_closed) {
            el_lengths.push_back(distance(this->back(), this->front()));
        }

        // 3) Now do the “cumulative path length” for s
        (*this)[0].s = T();
        for (size_t i = 1; i < this->size(); ++i) {
            (*this)[i].s = (*this)[i - 1].s + el_lengths[i - 1];
        }
        
        T avg_el_length = std::accumulate(el_lengths.begin(), el_lengths.end(), T()) / static_cast<T>(el_lengths.size());

        // Calculate step indices using T for calculations
        int ind_step_preview_psi = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_psi_preview) / avg_el_length)));
        int ind_step_review_psi = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_psi_review) / avg_el_length)));
        int ind_step_preview_curv = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_curv_preview) / avg_el_length)));
        int ind_step_review_curv = std::max(1, static_cast<int>(std::round(static_cast<T>(stepsize_curv_review) / avg_el_length)));

        if (is_closed) {
            
            // Calculate heading (psi)
            for (size_t i = 0; i < this->size(); ++i) {
                int preview_idx = (i + ind_step_preview_psi) % this->size();
                int review_idx = (i - ind_step_review_psi + this->size()) % this->size();
                
                T dx = (*this)[preview_idx].x - (*this)[review_idx].x;
                T dy = (*this)[preview_idx].y - (*this)[review_idx].y;
                (*this)[i].psi = normalize_psi(std::atan2(dy, dx));
            }

            // Calculate curvature (kappa)
            if (calc_curv) {
                for (size_t i = 0; i < this->size(); ++i) {
                    size_t preview_idx = (i + ind_step_preview_curv) % this->size();
                    size_t review_idx = (i - ind_step_review_curv + this->size()) % this->size();
                    
                    T delta_psi = normalize_psi((*this)[preview_idx].psi - (*this)[review_idx].psi);
                    
                    // Calculate path length between review and preview points
                    T path_length = T();
                    for (size_t j = review_idx; j != preview_idx; j = (j + 1) % this->size()) {
                        path_length += el_lengths[j];
                    }
                    
                    (*this)[i].kappa = delta_psi / path_length;
                }
            }
        } else {
            // Calculate heading for open path
            for (size_t i = 0; i < this->size(); ++i) {
                T dx, dy;
                if (i == 0) {
                    dx = (*this)[1].x - (*this)[0].x;
                    dy = (*this)[1].y - (*this)[0].y;
                } else if (i == this->size() - 1) {
                    dx = (*this)[i].x - (*this)[i-1].x;
                    dy = (*this)[i].y - (*this)[i-1].y;
                } else {
                    dx = (*this)[i+1].x - (*this)[i-1].x;
                    dy = (*this)[i+1].y - (*this)[i-1].y;
                }
                (*this)[i].psi = normalize_psi(std::atan2(dy, dx));
            }

            // Calculate curvature for open path
            if (calc_curv) {
                for (size_t i = 0; i < this->size(); ++i) {
                    T delta_psi;
                    T path_length;
                    
                    if (i == 0) {
                        delta_psi = normalize_psi((*this)[1].psi - (*this)[0].psi);
                        path_length = el_lengths[0];
                    } else if (i == this->size() - 1) {
                        delta_psi = normalize_psi((*this)[i].psi - (*this)[i-1].psi);
                        path_length = el_lengths[i-1];
                    } else {
                        delta_psi = normalize_psi((*this)[i+1].psi - (*this)[i-1].psi);
                        path_length = el_lengths[i] + el_lengths[i-1];
                    }
                    
                    (*this)[i].kappa = delta_psi / path_length;
                }
            }
        }
    }

    std::vector<TrackPoint2<T>> interpolate(const std::vector<T>& query_s, bool is_closed = true) const {
        if (this->empty()) {
            throw std::runtime_error("Track is empty!");
        }
        if (!this->has_s()) {
            throw std::runtime_error("Track must have s values to interpolate! Call calculate() first.");
        }

        // Create a copy of the track object
        Track2<T> track = *this;

        if (is_closed) {
            // Add the first point to the end of the track to handle wrap-around
            track.push_back(this->front());
            track.back().s = this->back().s + distance(this->back(), this->front());
        }

        std::vector<TrackPoint2<T>> interpolated_points;
        interpolated_points.reserve(query_s.size());

        // Compute total track length
        T s_min = track.front().s;
        T s_max = track.back().s;

        for (T s_query : query_s) {
            // Handle closed track wrap-around

            if (is_closed) {
                if (s_query < s_min || s_query >= s_max) {
                    s_query = s_min + std::fmod(s_query - s_min + (s_max - s_min), s_max - s_min);
                }
            } else {
                if (s_query < s_min || s_query > s_max) {
                    throw std::runtime_error("Query s is out of track range!");
                }
            }

            // Find the lower bound index using binary search
            auto it = std::lower_bound(track.begin(), track.end(), s_query,
            [](const TrackPoint2<T>& p, T s) { return p.s < s; });

            size_t idx = std::distance(track.begin(), it);

            // Handle wrap-around for closed track
            
            if (idx == 0) {
                interpolated_points.push_back(track.front());
                continue;
            }
            if (idx >= track.size()) {
                interpolated_points.push_back(track.back());
                continue;
            }
    
            size_t idx1 = idx - 1;
            size_t idx2 = idx;
            T alpha = (s_query - track[idx1].s) / (track[idx2].s - track[idx1].s);  // Linear interpolation factor

            TrackPoint2<T> interpolated;
            interpolated.s = s_query;
            interpolated.x = track[idx1].x + alpha * (track[idx2].x - track[idx1].x);
            interpolated.y = track[idx1].y + alpha * (track[idx2].y - track[idx1].y);
            interpolated.psi = normalize_psi(track[idx1].psi + alpha * (track[idx2].psi - track[idx1].psi));
            interpolated.kappa = track[idx1].kappa + alpha * (track[idx2].kappa - track[idx1].kappa);
            interpolated.wl = track[idx1].wl + alpha * (track[idx2].wl - track[idx1].wl);
            interpolated.wr = track[idx1].wr + alpha * (track[idx2].wr - track[idx1].wr);

            interpolated_points.push_back(interpolated);
        }

        return interpolated_points;
    }

    TrackPoint2<T> interpolate(const T& s_query, bool is_closed = true) const {
        if (this->empty()) {
            throw std::runtime_error("Track is empty!");
        }
        if (!this->has_s()) {
            throw std::runtime_error("Track must have s values to interpolate! Call calculate() first.");
        }

        // Create a copy of the track object
        Track2<T> track = *this;

        if (is_closed) {
            // Add the first point to the end of the track to handle wrap-around
            track.push_back(this->front());
            track.back().s = this->back().s + distance(this->back(), this->front());
        }

        // Compute total track length
        T s_min = track.front().s;
        T s_max = track.back().s;

        T s_query_normalized = s_query;
        // Handle closed track wrap-around
        if (is_closed) {
            if (s_query < s_min || s_query >= s_max) {
                s_query_normalized = s_min + std::fmod(s_query - s_min + (s_max - s_min), s_max - s_min);
            }
        } else {
            if (s_query < s_min || s_query > s_max) {
                throw std::runtime_error("Query s is out of track range!");
            }
        }

        // Find the lower bound index using binary search
        auto it = std::lower_bound(track.begin(), track.end(), s_query_normalized,
            [](const TrackPoint2<T>& p, T s) { return p.s < s; });

        size_t idx = std::distance(track.begin(), it);

        if (idx == 0) {
            return track.front();
        }
        if (idx >= track.size()) {
            return track.back();
        }

        size_t idx1 = idx - 1;
        size_t idx2 = idx;
        T alpha = (s_query_normalized - track[idx1].s) / (track[idx2].s - track[idx1].s);

        TrackPoint2<T> interpolated;
        interpolated.s = s_query_normalized;
        interpolated.x = track[idx1].x + alpha * (track[idx2].x - track[idx1].x);
        interpolated.y = track[idx1].y + alpha * (track[idx2].y - track[idx1].y);
        interpolated.psi = normalize_psi(track[idx1].psi + alpha * (track[idx2].psi - track[idx1].psi));
        interpolated.kappa = track[idx1].kappa + alpha * (track[idx2].kappa - track[idx1].kappa);
        interpolated.wl = track[idx1].wl + alpha * (track[idx2].wl - track[idx1].wl);
        interpolated.wr = track[idx1].wr + alpha * (track[idx2].wr - track[idx1].wr);

        return interpolated;
    }

    Track2<T> interpolate_track(T stepsize, bool is_closed = true) const {
        if (this->empty()) {
            throw std::runtime_error("Track is empty!");
        }
        if (!this->has_s()) {
            throw std::runtime_error("Track must have s values to interpolate! Call calculate() first.");
        }

        // Calculate total length and number of points needed
        T s_min = this->front().s;
        T s_max = this->back().s;
        if (is_closed) {
            s_max += distance(this->back(), this->front());
        }

        // Create vector of evenly spaced s values
        size_t n_points;
        if (is_closed) {
            n_points = static_cast<size_t>(std::floor((s_max - s_min) / stepsize));
        } else {
            n_points = static_cast<size_t>(std::floor((s_max - s_min) / stepsize + 1));
        }
        std::vector<T> s_values;
        s_values.reserve(n_points);
        
        for (size_t i = 0; i < n_points; ++i) {
            s_values.push_back(s_min + i * stepsize);
        }
        // s_values.push_back(s_max);  // Ensure we hit the end exactly

        // Get interpolated points
        auto interpolated_points = interpolate(s_values, is_closed);
        
        // Create new track and calculate its properties
        Track2<T> new_track(interpolated_points);
        new_track.calculate(is_closed);
        
        return new_track;
    }

    TrackPoint2<T> project(const Point2<T>& point, bool is_closed = true) const {
        if (this->size() < 2) {
            throw std::runtime_error("Track must have at least 2 points!");
        }

        T min_dist = std::numeric_limits<T>::max();
        size_t seg_idx1 = 0;  // Store segment indices instead of s value
        T proj_t = 0;         // Store projection parameter
        Point2<T> proj_point;

        // Create a copy for segment iteration
        Track2<T> track = *this;
        if (is_closed) {
            track.push_back(this->front());
            if (this->has_s()) {
                track.back().s = this->back().s + distance(this->back(), this->front());
            }
        }

        // Iterate through track segments to find closest projection
        for (size_t i = 0; i < track.size() - 1; ++i) {
            const auto& p1 = track[i];
            const auto& p2 = track[i + 1];

            // Calculate vectors
            Point2<T> segment = {p2.x - p1.x, p2.y - p1.y};
            Point2<T> to_point = {point.x - p1.x, point.y - p1.y};

            // Calculate dot product and segment length
            T dot = to_point.x * segment.x + to_point.y * segment.y;
            T segment_length_sq = segment.x * segment.x + segment.y * segment.y;

            // Calculate projection parameter (t)
            T t = std::clamp(dot / segment_length_sq, T(0), T(1));

            // Calculate projected point
            Point2<T> curr_proj = {
                p1.x + t * segment.x,
                p1.y + t * segment.y
            };

            // Calculate distance to projected point
            T curr_dist = distance(point, curr_proj);

            if (curr_dist < min_dist) {
                min_dist = curr_dist;
                proj_point = curr_proj;
                seg_idx1 = i;
                proj_t = t;
            }
        }

        // Now interpolate directly using the found segment
        const auto& p1 = track[seg_idx1];
        const auto& p2 = track[seg_idx1 + 1];

        TrackPoint2<T> interpolated;
        interpolated.x = proj_point.x;  // Use already calculated projection
        interpolated.y = proj_point.y;

        // Only interpolate other properties if they exist in the track
        if (this->has_s()) {
            interpolated.s = p1.s + proj_t * (p2.s - p1.s);
        }
        if (this->has_psi()) {
            interpolated.psi = normalize_psi(p1.psi + proj_t * normalize_psi(p2.psi - p1.psi));
        }
        if (this->has_kappa()) {
            interpolated.kappa = p1.kappa + proj_t * (p2.kappa - p1.kappa);
        }
        if (this->has_widths()) {
            interpolated.wl = p1.wl + proj_t * (p2.wl - p1.wl);
            interpolated.wr = p1.wr + proj_t * (p2.wr - p2.wr);
        }

        return interpolated;
    }
}; // class Track2

typedef Track2<int> Track2i;
typedef Track2<float> Track2f;
typedef Track2<double> Track2d;

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

// template<typename T>
// std::vector<size_t> find_k_nearest_idx(const Track2<T>& track, const Point2<T>& point, int k) {
//     if (track.size() < 2) return track.empty() ? std::vector<size_t>() : std::vector<size_t>({0});
    
//     std::vector<T> dists;
//     dists.reserve(track.size());
    
//     // Calculate distances to all points
//     for (const auto& p : track) {
//         dists.push_back(std::hypot(p.x - point.x, p.y - point.y));
//     }
    
//     // Find k nearest points
//     std::vector<size_t> nearest_k;
//     for (int i = 0; i < k; ++i) {
//         auto it = std::min_element(dists.begin(), dists.end());
//         nearest_k.push_back(std::distance(dists.begin(), it));
//         *it = std::numeric_limits<T>::max();
//     }
    
//     return nearest_k;
// }


}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__TRACK_HPP