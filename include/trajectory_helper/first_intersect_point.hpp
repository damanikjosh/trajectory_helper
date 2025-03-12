#ifndef TRAJECTORY_HELPER__FIRST_INTERSECT_POINT_HPP
#define TRAJECTORY_HELPER__FIRST_INTERSECT_POINT_HPP

#include <cmath>
#include <optional>
#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"

namespace th {

std::optional<double> check_segment_intersection(const Track2f& segment, const Point2f& center, double radius) {
    if (segment.size() != 2) return false;
    
    const Point2f& p1 = segment[0].to_point();
    const Point2f& p2 = segment[1].to_point();
    
    // Vector from p1 to p2
    Point2f v = p2 - p1;
    
    // Quadratic equation coefficients
    double a = v.dot(v);
    double b = 2.0 * v.dot(p1 - center);
    double c = p1.dot(p1) + center.dot(center) - 2.0 * p1.dot(center) - radius * radius;
    
    // Calculate discriminant
    double disc = b * b - 4.0 * a * c;
    
    // No intersection if discriminant is negative
    if (disc < 0) return false;
    
    // Calculate intersection parameters
    double sqrt_disc = std::sqrt(disc);
    double t1 = (-b + sqrt_disc) / (2.0 * a);
    double t2 = (-b - sqrt_disc) / (2.0 * a);
    
    // Check if either intersection point lies on the segment (t in [0,1])
    if (0.0 <= t1 && t1 <= 1.0) {
        return t1;
    } else if (0.0 <= t2 && t2 <= 1.0) {
        return t2;
    } else {
        return std::nullopt;
    }
}

std::optional<TrackPoint2f> first_intersect_point(
    const Point2f& center,
    double radius,
    const Track2f& track,
    bool wrap = true)
{
    if (track.size() < 2) return std::nullopt;
    
    size_t nearest_idx = find_nearest_idx(track, center);

    // Iterate from nearest_idx to the end of the track
    for (size_t i = nearest_idx; i < track.size()-1; ++i) {
        Track2f segment = {track[i], track[i + 1]};
        auto t = check_segment_intersection(segment, center, radius);
        if (check_segment_intersection(segment, center, radius)) {
            return interp_point(segment, {0, 1}, t.value());
        }
    }

    // If wrapping is enabled and we haven't found an intersection, check the last-to-first segment
    if (wrap) {
        for (size_t i = 0; i < nearest_idx; ++i) {
            Track2f segment = {track[i], track[i + 1]};
            auto t = check_segment_intersection(segment, center, radius);
            if (check_segment_intersection(segment, center, radius)) {
                return interp_point(segment, {0, 1}, t.value());
            }
        }
    }

    return std::nullopt;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__FIRST_INTERSECT_POINT_HPP
