#include <gtest/gtest.h>
#include <trajectory_helper/track/track.hpp>
#include <cmath>

TEST(Track2Test, DefaultConstructor) {
    th::Track2d track;
    EXPECT_TRUE(track.empty());
}


TEST(Track2Test, ConstructorWithPointsUnclosed) {
    std::vector<th::Point2d> points = {
        th::Point2d(0.0, 0.0),
        th::Point2d(1.0, 0.0),
        th::Point2d(1.0, 1.0),
        th::Point2d(0.0, 1.0)
    };
    th::Track2d track(points);
    
    // Check that the points were added
    EXPECT_EQ(track.size(), 4);
    EXPECT_EQ(track[3].x, 0.0);
    EXPECT_EQ(track[3].y, 1.0);
    
    // Expect that psi and kappa values are not calculated
    EXPECT_FALSE(track.has_psi());
    EXPECT_FALSE(track.has_kappa());
    EXPECT_FALSE(track.has_widths());
}

TEST(Track2Test, ConstructorWithTrackPointsUnclosed) {
    std::vector<th::TrackPoint2d> points = {
        th::TrackPoint2d(0.0, 0.0),
        th::TrackPoint2d(1.0, 0.0),
        th::TrackPoint2d(1.0, 1.0),
        th::TrackPoint2d(0.0, 1.0)
    };
    th::Track2d track(points);
    
    // Check that the points were added
    EXPECT_EQ(track.size(), 4);
    EXPECT_EQ(track[3].x, 0.0);
    EXPECT_EQ(track[3].y, 1.0);
    
    // Expect that psi and kappa values are not calculated
    EXPECT_FALSE(track.has_psi());
    EXPECT_FALSE(track.has_kappa());
    EXPECT_FALSE(track.has_widths());
}

TEST(Track2Test, VectorAccessors) {
    th::Track2<double> track;
    track.push_back(th::TrackPoint2<double>(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
    track.push_back(th::TrackPoint2<double>(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0));
    
    auto s_vals = track.s();
    EXPECT_EQ(s_vals.size(), 2);
    EXPECT_EQ(s_vals[0], 0.0);
    EXPECT_EQ(s_vals[1], 1.0);
    
    auto x_vals = track.x();
    EXPECT_EQ(x_vals[0], 1.0);
    EXPECT_EQ(x_vals[1], 2.0);
    
    auto y_vals = track.y();
    EXPECT_EQ(y_vals[0], 2.0);
    EXPECT_EQ(y_vals[1], 3.0);
    
    auto psi_vals = track.psi();
    EXPECT_EQ(psi_vals[0], 3.0);
    EXPECT_EQ(psi_vals[1], 4.0);
    
    auto kappa_vals = track.kappa();
    EXPECT_EQ(kappa_vals[0], 6.0);
    EXPECT_EQ(kappa_vals[1], 7.0);
    
    auto wl_vals = track.wl();
    EXPECT_EQ(wl_vals[0], 4.0);
    EXPECT_EQ(wl_vals[1], 5.0);
    
    auto wr_vals = track.wr();
    EXPECT_EQ(wr_vals[0], 5.0);
    EXPECT_EQ(wr_vals[1], 6.0);
}

TEST(Track2Test, SetWidths) {
    std::vector<th::Point2d> points = {
        th::Point2d(0.0, 0.0),
        th::Point2d(0.0, 1.0)
    };
    th::Track2d track(points);
    
    std::vector<double> wl = {1.0, 2.0};
    std::vector<double> wr = {3.0, 4.0};
    
    track.set_widths(wl, wr);
    EXPECT_TRUE(track.has_widths());
    EXPECT_EQ(track[0].wl, 1.0);
    EXPECT_EQ(track[0].wr, 3.0);
    EXPECT_EQ(track[1].wl, 2.0);
    EXPECT_EQ(track[1].wr, 4.0);
}

TEST(Track2Test, SetWidthsInvalidSize) {
    th::Track2d track;
    track.push_back(th::TrackPoint2d(0.0, 0.0));
    
    std::vector<double> wl = {1.0, 2.0};
    std::vector<double> wr = {3.0, 4.0};
    
    EXPECT_THROW(track.set_widths(wl, wr), std::runtime_error);
}

TEST(Track2Test, SinglePointError) {
    std::vector<th::Point2d> points = {
        th::Point2d(0.0, 0.0)
    };
    EXPECT_THROW(th::Track2d track(points), std::runtime_error);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
