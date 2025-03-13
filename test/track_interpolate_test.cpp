#include <gtest/gtest.h>
#include <trajectory_helper/track/track.hpp>
#include <cmath>

TEST(Track2InterpolateTest, NotCalculatedError) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    std::vector<double> s_query = {0.5, 1.5};
    EXPECT_THROW(track.interpolate(s_query), std::runtime_error);
}

TEST(Track2InterpolateTest, InterpolateTrackClosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    std::vector<double> s_query = {0.5, 3.0, 3.5, 4.5, -0.5};
    std::vector<th::TrackPoint2d> interpolated_points = track.interpolate(s_query, true);
    EXPECT_EQ(interpolated_points.size(), s_query.size());
    EXPECT_NEAR(interpolated_points[0].s, 0.5, 1e-10);
    EXPECT_NEAR(interpolated_points[0].x, 0.5, 1e-10);
    EXPECT_NEAR(interpolated_points[0].y, 0.0, 1e-10);

    EXPECT_NEAR(interpolated_points[1].s, 3.0, 1e-10);
    EXPECT_NEAR(interpolated_points[1].x, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_points[1].y, 1.0, 1e-10);

    EXPECT_NEAR(interpolated_points[2].s, 3.5, 1e-10);
    EXPECT_NEAR(interpolated_points[2].x, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_points[2].y, 0.5, 1e-10);

    EXPECT_NEAR(interpolated_points[3].s, 0.5, 1e-10);
    EXPECT_NEAR(interpolated_points[3].x, 0.5, 1e-10);
    EXPECT_NEAR(interpolated_points[3].y, 0.0, 1e-10);

    EXPECT_NEAR(interpolated_points[4].s, 3.5, 1e-10);
    EXPECT_NEAR(interpolated_points[4].x, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_points[4].y, 0.5, 1e-10);
}

TEST(Track2InterpolateTest, InterpolateSingleTrackClosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    th::TrackPoint2d interpolated_point = track.interpolate(-0.5, true);
    EXPECT_NEAR(interpolated_point.s, 3.5, 1e-10);
    EXPECT_NEAR(interpolated_point.x, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_point.y, 0.5, 1e-10);
}

TEST(Track2InterpolateTest, InterpolateTrackUnclosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    std::vector<double> s_query = {0.0, 2.5, 3.0};
    std::vector<th::TrackPoint2d> interpolated_points = track.interpolate(s_query, false);
    EXPECT_EQ(interpolated_points.size(), s_query.size());
    EXPECT_NEAR(interpolated_points[0].s, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_points[0].x, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_points[0].y, 0.0, 1e-10);

    EXPECT_NEAR(interpolated_points[1].s, 2.5, 1e-10);
    EXPECT_NEAR(interpolated_points[1].x, 0.5, 1e-10);
    EXPECT_NEAR(interpolated_points[1].y, 1.0, 1e-10);

    EXPECT_NEAR(interpolated_points[2].s, 3.0, 1e-10);
    EXPECT_NEAR(interpolated_points[2].x, 0.0, 1e-10);
    EXPECT_NEAR(interpolated_points[2].y, 1.0, 1e-10);
}

TEST(Track2InterpolateTest, InterpolateSingleTrackUnclosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    th::TrackPoint2d interpolated_point = track.interpolate(2.5, false);
    EXPECT_NEAR(interpolated_point.s, 2.5, 1e-10);
    EXPECT_NEAR(interpolated_point.x, 0.5, 1e-10);
    EXPECT_NEAR(interpolated_point.y, 1.0, 1e-10);
}

TEST(Track2InterpolateTest, ErrorUnclosedOutbound) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    EXPECT_THROW(track.interpolate(3.5, false), std::runtime_error);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
