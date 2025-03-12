#ifndef TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP
#define TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP

#include "trajectory_helper/point/point.hpp"

namespace th {
template<typename T>
struct TrackPoint2 {
    T s, x, y, psi, wr, wl, kappa;

    TrackPoint2() : s(std::numeric_limits<T>::infinity()), x(std::numeric_limits<T>::infinity()), y(std::numeric_limits<T>::infinity()), psi(std::numeric_limits<T>::infinity()), wr(std::numeric_limits<T>::infinity()), wl(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(std::numeric_limits<T>::infinity()), wr(std::numeric_limits<T>::infinity()), wl(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T psi) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(psi), wr(std::numeric_limits<T>::infinity()), wl(std::numeric_limits<T>::infinity()), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T wr, T wl) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(std::numeric_limits<T>::infinity()), wr(wr), wl(wl), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T psi, T wr, T wl) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(std::numeric_limits<T>::infinity()) {}
    TrackPoint2(T x, T y, T psi, T wr, T wl, T kappa) : s(std::numeric_limits<T>::infinity()), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(kappa) {}
    TrackPoint2(T s, T x, T y, T psi, T wr, T wl, T kappa) : s(s), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(kappa) {}

    Point2<T> to_point() const { return Point2<T>(x, y); }
    Point2<T> to_point() { return Point2<T>(x, y); }

    // bool has_psi() const { return psi != std::numeric_limits<T>::infinity(); }
    // bool has_kappa() const { return kappa != std::numeric_limits<T>::infinity(); }
    // bool has_widths() const { return wr != std::numeric_limits<T>::infinity() && wl != std::numeric_limits<T>::infinity(); }
};

typedef TrackPoint2<int> TrackPoint2i;
typedef TrackPoint2<float> TrackPoint2f;
typedef TrackPoint2<double> TrackPoint2d;


}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP