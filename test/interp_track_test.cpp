#include <gtest/gtest.h>
#include <trajectory_helper/track/interp_track.hpp>
#include <cmath>

TEST(InterpTrackTest, BasicInterpolationUnclosed) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(1.0, 0.0), th::Point2<double>(1.0, 1.0),  th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);

    
    // Interpolate to 0.5m step size
    auto interp_track = th::interp_track(track, 0.5, false);
    EXPECT_EQ(interp_track.size(), 7);  // Should have 6 points
    
    // Check interpolated points
    EXPECT_NEAR(interp_track[0].x, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[0].y, 0.0, 1e-10);

    EXPECT_NEAR(interp_track[1].x, 0.5, 1e-10);
    EXPECT_NEAR(interp_track[1].y, 0.0, 1e-10);

    EXPECT_NEAR(interp_track[2].x, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[2].y, 0.0, 1e-10);

    EXPECT_NEAR(interp_track[3].x, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[3].y, 0.5, 1e-10);

    EXPECT_NEAR(interp_track[4].x, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[4].y, 1.0, 1e-10);

    EXPECT_NEAR(interp_track[5].x, 0.5, 1e-10);
    EXPECT_NEAR(interp_track[5].y, 1.0, 1e-10);

    EXPECT_NEAR(interp_track[6].x, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[6].y, 1.0, 1e-10);
}

TEST(InterpTrackTest, BasicInterpolationClosed) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(1.0, 0.0), th::Point2<double>(1.0, 1.0),  th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, true);

    
    // Interpolate to 0.5m step size
    auto interp_track = th::interp_track(track, 0.5, true);
    EXPECT_EQ(interp_track.size(), 8);  // Should have 8 points
    
    // Check interpolated points
    EXPECT_NEAR(interp_track[0].x, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[0].y, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[0].psi, -M_PI/4, 1e-10);

    EXPECT_NEAR(interp_track[1].x, 0.5, 1e-10);
    EXPECT_NEAR(interp_track[1].y, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[1].psi, 0, 1e-10);

    EXPECT_NEAR(interp_track[2].x, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[2].y, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[2].psi, M_PI/4, 1e-10);

    EXPECT_NEAR(interp_track[3].x, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[3].y, 0.5, 1e-10);
    EXPECT_NEAR(interp_track[3].psi, M_PI/2, 1e-10);

    EXPECT_NEAR(interp_track[4].x, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[4].y, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[4].psi, 3*M_PI/4, 1e-10);

    EXPECT_NEAR(interp_track[5].x, 0.5, 1e-10);
    EXPECT_NEAR(interp_track[5].y, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[5].psi, M_PI, 1e-10);

    EXPECT_NEAR(interp_track[6].x, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[6].y, 1.0, 1e-10);
    EXPECT_NEAR(interp_track[6].psi, -3*M_PI/4, 1e-10);

    EXPECT_NEAR(interp_track[7].x, 0.0, 1e-10);
    EXPECT_NEAR(interp_track[7].y, 0.5, 1e-10);
    EXPECT_NEAR(interp_track[7].psi, -M_PI/2, 1e-10);

}


TEST(InterpTrackTest, PreservesWidths) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);

    // Set widths
    std::vector<double> wl = {1.0, 2.0};
    std::vector<double> wr = {3.0, 4.0};
    track.set_widths(wl, wr); 
    
    auto interp_track = th::interp_track(track, 0.5);
    EXPECT_TRUE(interp_track.has_widths());
    
    // Check interpolated widths
    EXPECT_NEAR(interp_track[1].wl, 1.5, 1e-10);  // Middle point
    EXPECT_NEAR(interp_track[1].wr, 3.5, 1e-10);
}

TEST(InterpTrackTest, PreservesHeading) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 1.0)
    };
    th::Track2<double> track(points, false);
    
    auto interp_track = th::interp_track(track, 0.5);
    EXPECT_TRUE(interp_track.has_psi());
    
    // All points should have approximately 45-degree heading
    for (const auto& point : interp_track) {
        EXPECT_NEAR(point.psi, M_PI/4, 1e-10);
    }
}

TEST(InterpTrackTest, EmptyTrackError) {
    th::Track2<double> track;
    EXPECT_THROW(th::interp_track(track, 0.5), std::runtime_error);
}

TEST(InterpTrackTest, ZeroStepSizeHandling) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    // Very small step size should still work
    auto interp_track = th::interp_track(track, 1e-5);
    EXPECT_EQ(interp_track.size(), 1e5 + 1);
    interp_track[1].x = 1e-5;

    // Zero step size should throw error
    EXPECT_THROW(th::interp_track(track, 0.0), std::runtime_error);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
