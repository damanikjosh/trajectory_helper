#ifndef TRAJECTORY_HELPER__CALC_SPLINES_HPP
#define TRAJECTORY_HELPER__CALC_SPLINES_HPP

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <limits>

// Include Eigen
#include <Eigen/Dense>

#include "trajectory_helper/types.hpp"
#include "trajectory_helper/utils.hpp"

namespace th {

/**
 * Spline result struct to hold the outputs.
 */
struct SplineResult {
    std::vector<std::vector<double>> x_coeff;           // (no_splines x 4)
    std::vector<std::vector<double>> y_coeff;           // (no_splines x 4)
    Eigen::MatrixXd M;                                  // The final LES matrix used (for reference)
    std::vector<std::vector<double>> normvec_normalized; // Normal vectors (no_splines x 2)
};

/**
 * Solve for curvature-continuous cubic splines between a set of path points.
 *
 * @param path              Vector of 2D points; may be "closed" or "unclosed."
 * @param el_lengths        (Optional) Distances between path points; if nullptr and scaling is
 *                          enabled, distances are computed internally.
 * @param psi_s             Heading at start point (required if path is unclosed).
 * @param psi_e             Heading at end   point (required if path is unclosed).
 * @param use_dist_scaling  Whether to apply distance scaling to heading/curvature constraints.
 *
 * @return                  SplineResult struct with x-coefficients, y-coefficients, M matrix, and
 *                          normalized normal vectors for each spline segment.
 *
 * @note   - "Closed" is detected if path.front() ~ path.back() and no headings are provided.
 *         - If unclosed, psi_s and psi_e must be valid (not NaN).
 *         - The returned coefficient matrices have shape (no_splines x 4) each.
 */
SplineResult calc_splines(const std::vector<Point2f>& path,
                          const std::vector<double>* el_lengths = nullptr,
                          double psi_s = std::numeric_limits<double>::quiet_NaN(),
                          double psi_e = std::numeric_limits<double>::quiet_NaN(),
                          bool use_dist_scaling = true)
{
    // Detect whether path is closed
    bool closed = false;
    if (pointsAreClose(path.front(), path.back())) {
        // If first ~ last and user did not provide headings, treat as closed
        if (std::isnan(psi_s) && std::isnan(psi_e)) {
            closed = true;
        }
    }

    // If unclosed, check that headings are given
    if (!closed) {
        if (std::isnan(psi_s) || std::isnan(psi_e)) {
            throw std::runtime_error(
                "Headings psi_s, psi_e must be provided for an unclosed spline calculation!");
        }
    }

    // Basic checks
    // path has N = path.size() points, so number of splines = N-1
    int N = static_cast<int>(path.size());
    if (N < 2) {
        throw std::runtime_error("Path must contain at least 2 points.");
    }
    int no_splines = N - 1;

    // Copy or compute element lengths if required
    std::vector<double> dist;
    if (use_dist_scaling) {
        if (el_lengths == nullptr) {
            // We need to compute distances
            dist.resize(N - 1);
            for (int i = 0; i < N - 1; i++) {
                double dx = path[i + 1].x - path[i].x;
                double dy = path[i + 1].y - path[i].y;
                dist[i] = std::sqrt(dx * dx + dy * dy);
            }
        } else {
            // Use provided distances
            if (static_cast<int>(el_lengths->size()) != (N - 1)) {
                throw std::runtime_error("el_lengths.size() must be exactly path.size() - 1!");
            }
            dist = *el_lengths;
        }
    }

    // If closed and dist-scaling, append the first distance
    // to align with the Python approach
    if (closed && use_dist_scaling && !dist.empty()) {
        dist.push_back(dist.front());
    }

    // Compute scaling factors
    // scaling has size (no_splines - 1) if unclosed
    // or size (no_splines) if closed (because dist had 1 appended)
    std::vector<double> scaling(std::max(0, no_splines - 1), 1.0);
    if (use_dist_scaling && !dist.empty()) {
        int limit = closed ? no_splines : (no_splines - 1);
        scaling.resize(limit);
        for (int i = 0; i < limit; i++) {
            if (dist[i+1] == 0.0) {
                throw std::runtime_error("Encountered zero distance in dist, invalid for scaling.");
            }
            scaling[i] = dist[i] / dist[i + 1];
        }
    }

    // M is dimension (4*no_splines) x (4*no_splines)
    // b_x, b_y are each dimension (4*no_splines)
    int dim = 4 * no_splines;

    // Create an Eigen matrix and vectors
    Eigen::MatrixXd M_eig = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd b_x_eig = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd b_y_eig = Eigen::VectorXd::Zero(dim);

    // Template for a single block in the matrix:
    // Row1: P_i(0) = path[i], row2: P_i(1) = path[i+1],
    // Row3: heading continuity, Row4: curvature continuity
    // We'll place these into M row-by-row, with row and column offsets.
    //   row 1 -> [1, 0,  0,  0,   0, 0, 0, 0]
    //   row 2 -> [1, 1,  1,  1,   0, 0, 0, 0]
    //   row 3 -> [0, 1,  2,  3,   0, -1, 0, 0]
    //   row 4 -> [0, 0,  2,  6,   0,  0,-2, 0]
    // But we multiply the "next-spline" portion in row3, row4 by scaling[i] or scaling[i]^2.

    // We'll fill these in carefully, then set boundary conditions.
    for (int i = 0; i < no_splines; i++) {
        int rowBase = 4 * i;
        int colBase = 4 * i;

        // If not the last spline, set all four rows
        // else only first two rows (pos constraints).
        if (i < no_splines - 1) {
            // row1
            M_eig(rowBase + 0, colBase + 0) = 1.0;
            // row2
            M_eig(rowBase + 1, colBase + 0) = 1.0;
            M_eig(rowBase + 1, colBase + 1) = 1.0;
            M_eig(rowBase + 1, colBase + 2) = 1.0;
            M_eig(rowBase + 1, colBase + 3) = 1.0;
            // row3
            M_eig(rowBase + 2, colBase + 1) = 1.0;    // a_1
            M_eig(rowBase + 2, colBase + 2) = 2.0;    // a_2
            M_eig(rowBase + 2, colBase + 3) = 3.0;    // a_3
            // This row also references the next spline => colBase+4..colBase+7,
            // specifically the a_1 of the next spline, which has a negative sign
            // but we scale it if needed
            // => M[rowBase+2, colBase+4+1] = -1 * scaling[i]
            M_eig(rowBase + 2, colBase + 4 + 1) = -1.0 * scaling[i];

            // row4
            M_eig(rowBase + 3, colBase + 2) = 2.0;   // 2 a_2
            M_eig(rowBase + 3, colBase + 3) = 6.0;   // 6 a_3
            // This row references the next spline => a_2 of next spline => -2*(scaling[i]^2)
            M_eig(rowBase + 3, colBase + 4 + 2) = -2.0 * scaling[i] * scaling[i];
        } else {
            // Last spline: only the first two rows
            // row1
            M_eig(rowBase + 0, colBase + 0) = 1.0;
            // row2
            M_eig(rowBase + 1, colBase + 0) = 1.0;
            M_eig(rowBase + 1, colBase + 1) = 1.0;
            M_eig(rowBase + 1, colBase + 2) = 1.0;
            M_eig(rowBase + 1, colBase + 3) = 1.0;
        }

        // Fill b_x, b_y for these 2 position constraints
        b_x_eig(rowBase + 0) = path[i].x;
        b_x_eig(rowBase + 1) = path[i + 1].x;
        b_y_eig(rowBase + 0) = path[i].y;
        b_y_eig(rowBase + 1) = path[i + 1].y;
    }

    // ---------------------------------------------------------
    // Set boundary conditions for last and first point
    // ---------------------------------------------------------
    if (!closed) {
        // Unclosed: fix heading at start (t=0) and end (t=1).
        // So we place these in the last two rows of M:
        // second-last row -> heading start => derivative at t=0
        // last row        -> heading end   => derivative at t=1

        // heading at start, M[-2, 1] = 1
        M_eig(dim - 2, 1) = 1.0;

        double el_length_s = (use_dist_scaling && !dist.empty()) ? dist[0] : 1.0;
        b_x_eig(dim - 2) = std::cos(psi_s + M_PI / 2.0) * el_length_s;
        b_y_eig(dim - 2) = std::sin(psi_s + M_PI / 2.0) * el_length_s;

        // heading at end => derivative at t=1 => [0,1,2,3] in the last 4 columns
        M_eig(dim - 1, dim - 4 + 0) = 0.0;
        M_eig(dim - 1, dim - 4 + 1) = 1.0;
        M_eig(dim - 1, dim - 4 + 2) = 2.0;
        M_eig(dim - 1, dim - 4 + 3) = 3.0;

        double el_length_e = (use_dist_scaling && !dist.empty()) ? dist.back() : 1.0;
        b_x_eig(dim - 1) = std::cos(psi_e + M_PI / 2.0) * el_length_e;
        b_y_eig(dim - 1) = std::sin(psi_e + M_PI / 2.0) * el_length_e;
    } else {
        // Closed: continuity of heading and curvature from last to first
        // heading boundary => M[-2, 1] = scaling[-1], M[-2, -3..-1] = [-1, -2, -3]
        // curvature boundary => M[-1, 2] = 2*scaling[-1]^2, M[-1, -2..-1] = [-2, -6]
        double scaleEnd = 1.0;
        if (use_dist_scaling && !scaling.empty()) {
            // For closed, the last "scaling" is scaling[no_splines - 1]
            scaleEnd = scaling[no_splines - 1];
        }

        // heading continuity
        M_eig(dim - 2, 1) = scaleEnd;
        M_eig(dim - 2, dim - 3) = -1.0;
        M_eig(dim - 2, dim - 2) = -2.0;
        M_eig(dim - 2, dim - 1) = -3.0;

        // curvature continuity
        M_eig(dim - 1, 2) = 2.0 * (scaleEnd * scaleEnd);
        M_eig(dim - 1, dim - 2) = -2.0;
        M_eig(dim - 1, dim - 1) = -6.0;
    }

    // ---------------------------------------------------------
    // Solve for x and y using Eigen
    // ---------------------------------------------------------
    Eigen::VectorXd x_les = M_eig.colPivHouseholderQr().solve(b_x_eig);
    Eigen::VectorXd y_les = M_eig.colPivHouseholderQr().solve(b_y_eig);

    // Reshape solution into (no_splines x 4)
    std::vector<std::vector<double>> coeffs_x(no_splines, std::vector<double>(4, 0.0));
    std::vector<std::vector<double>> coeffs_y(no_splines, std::vector<double>(4, 0.0));
    for (int i = 0; i < no_splines; i++) {
        for (int j = 0; j < 4; j++) {
            coeffs_x[i][j] = x_les(4 * i + j);
            coeffs_y[i][j] = y_les(4 * i + j);
        }
    }

    // Compute normal vectors: (coeffs_y[i][1], -coeffs_x[i][1]) for each spline
    // Then normalize
    std::vector<std::vector<double>> normvec_normalized(no_splines, std::vector<double>(2, 0.0));
    for (int i = 0; i < no_splines; i++) {
        double nx = coeffs_y[i][1];
        double ny = -coeffs_x[i][1];
        double len = std::sqrt(nx * nx + ny * ny);
        if (len < 1e-12) {
            // Degenerate case
            normvec_normalized[i][0] = 0.0;
            normvec_normalized[i][1] = 0.0;
        } else {
            normvec_normalized[i][0] = nx / len;
            normvec_normalized[i][1] = ny / len;
        }
    }

    // Return all in a SplineResult
    SplineResult result;
    result.x_coeff = coeffs_x;
    result.y_coeff = coeffs_y;
    result.M       = M_eig;  // store the final matrix for reference
    result.normvec_normalized = normvec_normalized;

    return result;
}
}  // namespace th

#endif  // TRAJECTORY_HELPER__CALC_SPLINES_HPP