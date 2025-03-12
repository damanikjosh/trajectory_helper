#ifndef TRAJECTORY_HELPER__TYPES_HPP
#define TRAJECTORY_HELPER__TYPES_HPP

#include <racecar_msgs/msg/trajectory.hpp>
#include <racecar_msgs/msg/trajectory_point.hpp>

namespace th {

template<typename T>
struct Point {
    T x, y;

    Point() : x(T()), y(T()) {}
    Point(T x, T y) : x(x), y(y) {}

    Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }
    Point operator-(const Point& p) const { return Point(x - p.x, y - p.y); }
    Point operator*(T scale) const { return Point(x * scale, y * scale); }
    Point operator/(T scale) const { return Point(x / scale, y / scale); }

    Point& operator+=(const Point& p) { x += p.x; y += p.y; return *this; }
    Point& operator-=(const Point& p) { x -= p.x; y -= p.y; return *this; }
    Point& operator*=(T scale) { x *= scale; y *= scale; return *this; }
    Point& operator/=(T scale) { x /= scale; y /= scale; return *this; }

    // dot product
    T dot(const Point& p) const { return x * p.x + y * p.y; }
    // cross product
    T cross(const Point& p) const { return x * p.y - y * p.x; }

    T norm() const { return std::hypot(x, y); }
};

// Common type definitions
typedef Point<int> Point2i;
typedef Point<float> Point2;
typedef Point<double> Point2d;
typedef Point<double> Point2f;  // Replacing the struct with typedef

template<typename T>
struct TrackPoint {
    T s, x, y, psi, wr, wl, kappa;

    TrackPoint() : s(T()), x(T()), y(T()), psi(T()), wr(T()), wl(T()), kappa(T()) {}
    TrackPoint(T x, T y) : s(T()), x(x), y(y), psi(T()), wr(T()), wl(T()), kappa(T()) {}
    TrackPoint(T x, T y, T psi) : s(T()), x(x), y(y), psi(psi), wr(T()), wl(T()), kappa(T()) {}
    TrackPoint(T x, T y, T wr, T wl) : s(T()), x(x), y(y), psi(T()), wr(wr), wl(wl), kappa(T()) {}
    TrackPoint(T x, T y, T psi, T wr, T wl) : s(T()), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(T()) {}
    TrackPoint(T x, T y, T psi, T wr, T wl, T kappa) : s(T()), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(kappa) {}
    TrackPoint(T s, T x, T y, T psi, T wr, T wl, T kappa) : s(s), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(kappa) {}

    Point<T> to_point() const { return Point<T>(x, y); }
    Point<T> to_point() { return Point<T>(x, y); }

    // bool has_psi() const { return psi != T(); }
    // bool has_kappa() const { return kappa != T(); }
    // bool has_widths() const { return wr != T() && wl != T(); }
};

typedef TrackPoint<int> TrackPoint2i;
typedef TrackPoint<float> TrackPoint2;
typedef TrackPoint<double> TrackPoint2d;
typedef TrackPoint<double> TrackPoint2f;

template<typename T>
class Track : public std::vector<TrackPoint<T>> {
public:
    // Inherit vector constructors
    using std::vector<TrackPoint<T>>::vector;
    
    static Track<T> from_msg(racecar_msgs::msg::Trajectory::SharedPtr msg) {
        Track<T> track;
        for (const auto& p : msg->points) {
            track.emplace_back(p.s, p.x, p.y, p.psi, p.wr, p.wl, p.kappa);
        }
        return track;
    }

    bool has_psi() const {
        if (this->empty()) return false;
        return this->front().psi != T();
    }

    bool has_kappa() const {
        if (this->empty()) return false;
        return this->front().kappa != T();
    }

    bool has_widths() const {
        if (this->empty()) return false;
        return this->front().wr != T() && this->front().wl != T();
    }

    racecar_msgs::msg::Trajectory::SharedPtr to_msg() const {
        racecar_msgs::msg::Trajectory::SharedPtr msg = std::make_shared<racecar_msgs::msg::Trajectory>();
        for (const auto& p : *this) {
            racecar_msgs::msg::TrajectoryPoint point;
            point.s = p.s;
            point.x = p.x;
            point.y = p.y;
            point.psi = p.psi;
            point.wr = p.wr;
            point.wl = p.wl;
            point.kappa = p.kappa;
            msg->points.push_back(point);
        }
        return msg;
    }

};


typedef Track<int> Track2i;
typedef Track<float> Track2;
typedef Track<double> Track2d;
typedef Track<double> Track2f;

}  // namespace th

#endif  // TRAJECTORY_HELPER__TYPES_HPP