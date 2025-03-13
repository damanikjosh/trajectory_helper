#include <gtest/gtest.h>
#include <trajectory_helper/track/track.hpp>
#include <cmath>

TEST(Track2InterpolateTrackTest, NotCalculatedError) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    EXPECT_THROW(track.interpolate_track(0.1), std::runtime_error);
}

TEST(Track2InterpolateTrackTest, InterpolateTrackClosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    th::Track2d interp_track_half = track.interpolate_track(0.5, true);
    EXPECT_EQ(interp_track_half.size(), 8);

    th::Track2d interp_track_quarter = track.interpolate_track(0.25, true);
    EXPECT_EQ(interp_track_quarter.size(), 16);

    th::Track2d interp_uneven = track.interpolate_track(0.4, true);
    EXPECT_EQ(interp_uneven.size(), 10); // (0.0, 0.4, 0.8, 1.0, 1.4, 1.8, 2.0, 2.4, 3.2, 3.6): 4.0 is skipped because it is the same as 0.0
}

TEST(Track2InterpolateTrackTest, InterpolateTrackUnclosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);

    th::Track2d interp_track_half = track.interpolate_track(0.5, false);
    EXPECT_EQ(interp_track_half.size(), 7);

    th::Track2d interp_track_quarter = track.interpolate_track(0.25, false);
    EXPECT_EQ(interp_track_quarter.size(), 13);

    th::Track2d interp_uneven = track.interpolate_track(0.4, false);
    EXPECT_EQ(interp_uneven.size(), 8); // (0.0, 0.4, 0.8, 1.0, 1.4, 1.8, 2.0, 2.4): 3.2 is skipped because it is out of bounds

}

// TEST(Track2InterpolateTrackTest, InterpolateSingleTrackClosed) {
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

// TEST(Track2InterpolateTrackTest, InterpolateTrackUnclosed) {
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

// TEST(Track2InterpolateTrackTest, InterpolateSingleTrackUnclosed) {
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

// TEST(Track2InterpolateTrackTest, ErrorUnclosedOutbound) {
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
