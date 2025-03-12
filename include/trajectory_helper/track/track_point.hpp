#ifndef TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP
#define TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP

#include "trajectory_helper/point/point.hpp"

namespace th {
template<typename T>
struct TrackPoint2 {
    T s, x, y, psi, wr, wl, kappa;

    TrackPoint2() : s(T()), x(T()), y(T()), psi(T()), wr(T()), wl(T()), kappa(T()) {}
    TrackPoint2(T x, T y) : s(T()), x(x), y(y), psi(T()), wr(T()), wl(T()), kappa(T()) {}
    TrackPoint2(T x, T y, T psi) : s(T()), x(x), y(y), psi(psi), wr(T()), wl(T()), kappa(T()) {}
    TrackPoint2(T x, T y, T wr, T wl) : s(T()), x(x), y(y), psi(T()), wr(wr), wl(wl), kappa(T()) {}
    TrackPoint2(T x, T y, T psi, T wr, T wl) : s(T()), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(T()) {}
    TrackPoint2(T x, T y, T psi, T wr, T wl, T kappa) : s(T()), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(kappa) {}
    TrackPoint2(T s, T x, T y, T psi, T wr, T wl, T kappa) : s(s), x(x), y(y), psi(psi), wr(wr), wl(wl), kappa(kappa) {}

    Point2<T> to_point() const { return Point2<T>(x, y); }
    Point2<T> to_point() { return Point2<T>(x, y); }

    // bool has_psi() const { return psi != T(); }
    // bool has_kappa() const { return kappa != T(); }
    // bool has_widths() const { return wr != T() && wl != T(); }
};

typedef TrackPoint2<int> TrackPoint2i;
typedef TrackPoint2<float> TrackPoint2f;
typedef TrackPoint2<double> TrackPoint2d;


}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__TRACK_POINT_HPP