#ifndef TRAJECTORY_HELPER__PROJECT_POINT_HPP
#define TRAJECTORY_HELPER__PROJECT_POINT_HPP

#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"
#include "trajectory_helper/interp_point.hpp"

namespace th {

TrackPoint2f project_point(const Track2f& track, const Point2f& point) {
    if (track.size() < 2) return track.empty() ? TrackPoint2f() : track.front();
    
    size_t nearest_idx = find_nearest_idx(track, point);

    // Check segments around nearest point
    std::vector<size_t> check_segments;
    if (nearest_idx == 0) {
        check_segments.push_back(track.size() - 1);
    } else {
        check_segments.push_back(nearest_idx - 1);
    }
    check_segments.push_back(nearest_idx);

    double min_dist = std::numeric_limits<double>::max();
    TrackPoint2f nearest_point;

    for (size_t idx : check_segments) {
        const auto& p1 = track[idx];
        const auto& p2 = track[(idx + 1) % track.size()];
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double segment_length_sq = dx * dx + dy * dy;
        
        if (segment_length_sq == 0) continue;
        
        double t = ((point.x - p1.x) * dx + (point.y - p1.y) * dy) / segment_length_sq;
        t = std::max(0.0, std::min(1.0, t));

        Track2f segment = {p1, p2};
        // Handle s overflows
        if (p2.s < p1.s) {
            segment[1].s += track.back().s;
        }
        
        std::vector<double> seg_dists = {0, std::sqrt(segment_length_sq)};
        
        TrackPoint2f interp = interp_point(segment, seg_dists, t * seg_dists[1]);

        // Handle s overflows
        if (interp.s > track.back().s) {
            interp.s += track.back().s;
        }
        
        double dist = std::hypot(interp.x - point.x, interp.y - point.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_point = interp;
        }
    }

    return nearest_point;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__PROJECT_POINT_HPP
