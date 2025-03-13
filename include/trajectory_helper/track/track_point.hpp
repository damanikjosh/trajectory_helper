#ifndef TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP
#define TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP

#include "trajectory_helper/point/point.hpp"

namespace th {
template<typename T>
struct TrackPoint2 {
    T s, x, y, psi, wl, wr, kappa;

    TrackPoint2() : s(std::numeric_limits<T>::infinity()), x(std::numeric_limits<T>::infinity()), y(std::numeric_limits<T>::infinity()), psi(std::numeric_limits<T>::infinity()), wl(std::numeric_limits<T>::infinity()), wr(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(std::numeric_limits<T>::infinity()), wl(std::numeric_limits<T>::infinity()), wr(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T psi) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(psi), wl(std::numeric_limits<T>::infinity()), wr(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T wl, T wr) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(std::numeric_limits<T>::infinity()), wl(wl), wr(wr), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T psi, T wl, T wr) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(psi), wl(wl), wr(wr), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T psi, T wl, T wr, T kappa) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(psi), wl(wl), wr(wr), kappa(kappa) {}
    TrackPoint2(T s, T x, T y, T psi, T wl, T wr, T kappa) : s(s), x(x), y(y), psi(psi), wl(wl), wr(wr), kappa(kappa) {}

    TrackPoint2(const Point2<T>& point) : s(std::numeric_limits<T>::infinity()), x(point.x), y(point.y), psi(std::numeric_limits<T>::infinity()), wl(std::numeric_limits<T>::infinity()), wr(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}

    Point2<T> to_point() const { return Point2<T>(x, y); }
    Point2<T> to_point() { return Point2<T>(x, y); }

    bool has_s() const { return !std::isinf(s); }
    bool has_psi() const { return !std::isinf(psi); }
    bool has_kappa() const { return !std::isinf(kappa); }
    bool has_widths() const { return !std::isinf(wr) && !std::isinf(wl); }
};

typedef TrackPoint2<int> TrackPoint2i;
typedef TrackPoint2<float> TrackPoint2f;
typedef TrackPoint2<double> TrackPoint2d;


}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP