#ifndef TRAJECTORY_HELPER__TRACK__TRACK_HPP
#define TRAJECTORY_HELPER__TRACK__TRACK_HPP

#include <vector>
#include <stdexcept>

#include "trajectory_helper/point/point.hpp"
#include "trajectory_helper/track/track_point.hpp"
#include "trajectory_helper/track/calc_track.hpp"

namespace th {

template<typename T>
class Track2 : public std::vector<TrackPoint2<T>> {
public:
    // Inherit vector constructors
    using std::vector<TrackPoint2<T>>::vector;

    Track2(const std::vector<Point2<T>>& points,
           bool is_closed,
           double stepsize_psi_preview = 1.0,
           double stepsize_psi_review = 1.0,
           double stepsize_curv_preview = 2.0,
           double stepsize_curv_review = 2.0,
           bool calc_curv = true) {
        *this = calc_track(points, is_closed, stepsize_psi_preview, 
                          stepsize_psi_review, stepsize_curv_preview,
                          stepsize_curv_review, calc_curv);
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

    bool has_speeds() const {
        if (this->empty()) return false;
        return this->front().v != T();
    }
};


typedef Track2<int> Track2i;
typedef Track2<float> Track2f;
typedef Track2<double> Track2d;

}  // namespace th

#endif  // TRAJECTORY_HELPER__TRACK__TRACK_HPP