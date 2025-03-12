#ifndef TRAJECTORY_HELPER__PROJECT_POINT_HPP
#define TRAJECTORY_HELPER__PROJECT_POINT_HPP

#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track.hpp"
#include "trajectory_helper/track/track_point.hpp"
#include "trajectory_helper/track/interp_track_point.hpp"

namespace th {
    

template<typename T>
TrackPoint2<T> project_point(const Track2<T>& track, const Point2<T>& point) {
    if (track.size() < 2) {
        throw std::runtime_error("Track must have at least 2 points!");
    }
    
    size_t nearest_idx = find_nearest_idx(track, point);

    // Check segments around nearest point
    std::vector<size_t> check_segments;
    if (nearest_idx == 0 && !track.closed) {
        check_segments.push_back(track.size() - 1);
    } else {
        check_segments.push_back(nearest_idx - 1);
    }
    check_segments.push_back(nearest_idx);


    T min_dist = std::numeric_limits<T>::max();
    TrackPoint2<T> nearest_point;

    for (size_t idx : check_segments) {
        if (track.closed && idx == track.size() - 1) {
            continue;
        }
        
        const auto& p1 = track[idx];
        const auto& p2 = track[(idx + 1) % track.size()];
        
        T dx = p2.x - p1.x;
        T dy = p2.y - p1.y;
        T segment_length_sq = dx * dx + dy * dy;
        
        if (segment_length_sq == T(0)) continue;
        
        T t = ((point.x - p1.x) * dx + (point.y - p1.y) * dy) / segment_length_sq;
        t = std::max(T(0), std::min(T(1), t));

        Track2<T> segment = {p1, p2};
        // Handle s overflows
        if (p2.s < p1.s) {
            segment[1].s += track.back().s;
        }

        T interp_s = p1.s + t * (segment[1].s - p1.s);
        TrackPoint2<T> interp = interp_track_point(segment, interp_s);

        // Handle s overflows
        if (interp.s > track.back().s) {
            interp.s -= track.back().s;
        }
        
        T dist = std::hypot(interp.x - point.x, interp.y - point.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_point = interp;
        }
    }

    return nearest_point;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__PROJECT_POINT_HPP
