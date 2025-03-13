#include <gtest/gtest.h>
#include <trajectory_helper/track/track.hpp>
#include <cmath>


TEST(Track2CalculateTest, CalculateTrackClosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(true);
    
    // Expect that psi and kappa values are calculated
    EXPECT_TRUE(track.has_psi());
    EXPECT_TRUE(track.has_kappa());
    EXPECT_FALSE(track.has_widths());

    // Check that psi values were calculated
    EXPECT_NEAR(track[0].psi, -M_PI/4, 1e-10);
    EXPECT_NEAR(track[1].psi, M_PI/4, 1e-10);
    EXPECT_NEAR(track[2].psi, 3*M_PI/4, 1e-10);
    EXPECT_NEAR(track[3].psi, -3*M_PI/4, 1e-10);

    // Check that kappa values were calculated
    EXPECT_NEAR(track[0].kappa, track[1].kappa, 1e-10);
    EXPECT_NEAR(track[1].kappa, track[2].kappa, 1e-10);
    EXPECT_NEAR(track[2].kappa, track[3].kappa, 1e-10);
    EXPECT_NEAR(track[3].kappa, track[0].kappa, 1e-10);
}

TEST(Track2CalculateTest, CalculateTrackUnclosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    track.calculate(false);
    
    // Expect that psi and kappa values are calculated
    EXPECT_TRUE(track.has_psi());
    EXPECT_TRUE(track.has_kappa());
    EXPECT_FALSE(track.has_widths());

    // Check that psi values were calculated
    EXPECT_NEAR(track[0].psi, 0.0, 1e-10);
    EXPECT_NEAR(track[1].psi, M_PI/4, 1e-10);
    EXPECT_NEAR(track[2].psi, 3*M_PI/4, 1e-10);
    EXPECT_NEAR(track[3].psi, M_PI, 1e-10);

    // Check that kappa values were calculated
    EXPECT_LT(track[0].kappa, track[1].kappa);
    EXPECT_NEAR(track[1].kappa, track[2].kappa, 1e-10);
    EXPECT_GT(track[2].kappa, track[3].kappa);
    EXPECT_NEAR(track[3].kappa, track[0].kappa, 1e-10);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
