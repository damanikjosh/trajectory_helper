#include <gtest/gtest.h>
#include <trajectory_helper/track/track.hpp>
#include <cmath>

TEST(Track2ProjectTest, ProjectWithoutCalculated) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    th::Point2d point(0.5, -1.0);
    th::TrackPoint2d projected_point = track.project(point, true);

    EXPECT_NEAR(projected_point.x, 0.5, 1e-10);
    EXPECT_NEAR(projected_point.y, 0.0, 1e-10);

    EXPECT_EQ(projected_point.has_s(), false);
    EXPECT_EQ(projected_point.has_psi(), false);
    EXPECT_EQ(projected_point.has_kappa(), false);
    EXPECT_EQ(projected_point.has_widths(), false);
}

TEST(Track2ProjectTest, ProjectTrackClosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    th::Point2d point_inside(0.5, -1.0);
    th::TrackPoint2d projected_point_inside = track.project(point_inside, true);

    EXPECT_NEAR(projected_point_inside.s, 0.5, 1e-10);
    EXPECT_NEAR(projected_point_inside.x, 0.5, 1e-10);
    EXPECT_NEAR(projected_point_inside.y, 0.0, 1e-10);

    th::Point2d point_outside(-1.0, 0.2);
    th::TrackPoint2d projected_point_outside = track.project(point_outside, true);

    EXPECT_NEAR(projected_point_outside.s, 3.8, 1e-10);
    EXPECT_NEAR(projected_point_outside.x, 0.0, 1e-10);
    EXPECT_NEAR(projected_point_outside.y, 0.2, 1e-10);
}

TEST(Track2ProjectTest, ProjectTrackUnclosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    th::Point2d point_inside(0.5, -1.0);
    th::TrackPoint2d projected_point_inside = track.project(point_inside, false);

    EXPECT_NEAR(projected_point_inside.s, 0.5, 1e-10);
    EXPECT_NEAR(projected_point_inside.x, 0.5, 1e-10);
    EXPECT_NEAR(projected_point_inside.y, 0.0, 1e-10);

    th::Point2d point_outside1(-1.0, 0.2);
    th::TrackPoint2d projected_point_outside1 = track.project(point_outside1, false);

    EXPECT_NEAR(projected_point_outside1.s, 0.0, 1e-10);
    EXPECT_NEAR(projected_point_outside1.x, 0.0, 1e-10);
    EXPECT_NEAR(projected_point_outside1.y, 0.0, 1e-10);

    th::Point2d point_outside2(-1.0, 0.8);
    th::TrackPoint2d projected_point_outside2 = track.project(point_outside2, false);

    EXPECT_NEAR(projected_point_outside2.s, 3.0, 1e-10);
    EXPECT_NEAR(projected_point_outside2.x, 0.0, 1e-10);
    EXPECT_NEAR(projected_point_outside2.y, 1.0, 1e-10);
}

// TEST(Track2ProjectTest, InterpolateSingleTrackClosed) {
//     std::vector<th::TrackPoint2d> points = {
//         th::TrackPoint2d(0.0, 0.0),
//         th::TrackPoint2d(1.0, 0.0),
//         th::TrackPoint2d(1.0, 1.0),
//         th::TrackPoint2d(0.0, 1.0)
//     };
//     th::Track2d track(points);
//     track.calculate(true);

//     th::TrackPoint2d interpolated_point = track.interpolate(-0.5, true);
//     EXPECT_NEAR(interpolated_point.s, 3.5, 1e-10);
//     EXPECT_NEAR(interpolated_point.x, 0.0, 1e-10);
//     EXPECT_NEAR(interpolated_point.y, 0.5, 1e-10);
// }

// TEST(Track2ProjectTest, InterpolateTrackUnclosed) {
//     std::vector<th::TrackPoint2d> points = {
//         th::TrackPoint2d(0.0, 0.0),
//         th::TrackPoint2d(1.0, 0.0),
//         th::TrackPoint2d(1.0, 1.0),
//         th::TrackPoint2d(0.0, 1.0)
//     };
//     th::Track2d track(points);
//     track.calculate(true);

//     std::vector<double> s_query = {0.0, 2.5, 3.0};
//     std::vector<th::TrackPoint2d> interpolated_points = track.interpolate(s_query, false);
//     EXPECT_EQ(interpolated_points.size(), s_query.size());
//     EXPECT_NEAR(interpolated_points[0].s, 0.0, 1e-10);
//     EXPECT_NEAR(interpolated_points[0].x, 0.0, 1e-10);
//     EXPECT_NEAR(interpolated_points[0].y, 0.0, 1e-10);

//     EXPECT_NEAR(interpolated_points[1].s, 2.5, 1e-10);
//     EXPECT_NEAR(interpolated_points[1].x, 0.5, 1e-10);
//     EXPECT_NEAR(interpolated_points[1].y, 1.0, 1e-10);

//     EXPECT_NEAR(interpolated_points[2].s, 3.0, 1e-10);
//     EXPECT_NEAR(interpolated_points[2].x, 0.0, 1e-10);
//     EXPECT_NEAR(interpolated_points[2].y, 1.0, 1e-10);
// }

// TEST(Track2ProjectTest, InterpolateSingleTrackUnclosed) {
//     std::vector<th::TrackPoint2d> points = {
//         th::TrackPoint2d(0.0, 0.0),
//         th::TrackPoint2d(1.0, 0.0),
//         th::TrackPoint2d(1.0, 1.0),
//         th::TrackPoint2d(0.0, 1.0)
//     };
//     th::Track2d track(points);
//     track.calculate(true);

//     th::TrackPoint2d interpolated_point = track.interpolate(2.5, false);
//     EXPECT_NEAR(interpolated_point.s, 2.5, 1e-10);
//     EXPECT_NEAR(interpolated_point.x, 0.5, 1e-10);
//     EXPECT_NEAR(interpolated_point.y, 1.0, 1e-10);
// }

// TEST(Track2ProjectTest, ErrorUnclosedOutbound) {
//     std::vector<th::TrackPoint2d> points = {
//         th::TrackPoint2d(0.0, 0.0),
//         th::TrackPoint2d(1.0, 0.0),
//         th::TrackPoint2d(1.0, 1.0),
//         th::TrackPoint2d(0.0, 1.0)
//     };
//     th::Track2d track(points);
//     track.calculate(true);

//     EXPECT_THROW(track.interpolate(3.5, false), std::runtime_error);
// }


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
