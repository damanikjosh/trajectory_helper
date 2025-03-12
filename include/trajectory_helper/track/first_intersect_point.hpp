#ifndef TRAJECTORY_HELPER__TRACK__FIRST_INTERSECT_POINT_HPP
#define TRAJECTORY_HELPER__TRACK__FIRST_INTERSECT_POINT_HPP

#include <cmath>
#include <optional>

#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track.hpp"
#include "trajectory_helper/track/track_point.hpp"

#include "trajectory_helper/utils.hpp"

namespace th {

template<typename T>
std::optional<T> check_segment_intersection(const Track2<T>& segment, const Point2<T>& center, T radius) {
    if (segment.size() != 2) return false;
    
    const Point2<T>& p1 = segment[0].to_point();
    const Point2<T>& p2 = segment[1].to_point();
    
    // Vector from p1 to p2
    Point2<T> v = p2 - p1;
    
    // Quadratic equation coefficients
    T a = v.dot(v);
    T b = T(2) * v.dot(p1 - center);
    T c = p1.dot(p1) + center.dot(center) - T(2) * p1.dot(center) - radius * radius;
    
    // Calculate discriminant
    T disc = b * b - T(4) * a * c;
    
    // No intersection if discriminant is negative
    if (disc < T(0)) return std::nullopt;
    
    // Calculate intersection parameters
    T sqrt_disc = std::sqrt(disc);
    T t1 = (-b + sqrt_disc) / (T(2) * a);
    T t2 = (-b - sqrt_disc) / (T(2) * a);
    
    // Check if either intersection point lies on the segment (t in [0,1])
    if (T(0) <= t1 && t1 <= T(1)) {
        return t1;
    } else if (T(0) <= t2 && t2 <= T(1)) {
        return t2;
    }
    return std::nullopt;
}

template<typename T>
std::optional<TrackPoint2<T>> first_intersect_point(
    const Track2<T>& track,
    const Point2<T>& center,
    T radius,
    bool wrap = true)
{
    if (track.size() < 2) return std::nullopt;
    
    size_t nearest_idx = find_nearest_idx(track, center);

    // Iterate from nearest_idx to the end of the track
    for (size_t i = nearest_idx; i < track.size()-1; ++i) {
        Track2<T> segment = {track[i], track[i + 1]};
        auto t = check_segment_intersection(segment, center, radius);
        if (t) {
            T interp_s = segment[0].s + t.value() * (segment[1].s - segment[0].s);
            return interp_track_point(segment, interp_s);
        }
    }

    // If wrapping is enabled and we haven't found an intersection
    if (wrap) {
        for (size_t i = 0; i < nearest_idx; ++i) {
            Track2<T> segment = {track[i], track[i + 1]};
            auto t = check_segment_intersection(segment, center, radius);
            if (t) {
                T interp_s = segment[0].s + t.value() * (segment[1].s - segment[0].s);
                return interp_track_point(segment, interp_s);
            }
        }
    }

    return std::nullopt;
}

}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__FIRST_INTERSECT_POINT_HPP
