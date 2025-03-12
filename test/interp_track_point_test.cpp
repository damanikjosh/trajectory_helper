#include <gtest/gtest.h>
#include <trajectory_helper/track/interp_track_point.hpp>
#include <cmath>

TEST(InterpTrackPoint2Test, InterpolationX) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(1.0, 0.0)};
    th::Track2<double> track(points, false);
    
    auto point = th::interp_track_point(track, 0.5);
    EXPECT_EQ(point.s, 0.5);
    EXPECT_EQ(point.x, 0.5);
    EXPECT_EQ(point.y, 0.0);
}

TEST(InterpTrackPoint2Test, InterpolationY) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);
    
    auto point = th::interp_track_point(track, 0.5);
    EXPECT_EQ(point.s, 0.5);
    EXPECT_EQ(point.x, 0.0);
    EXPECT_EQ(point.y, 0.5);
}

TEST(InterpTrackPoint2Test, EdgePoints) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);
    
    // Test start point
    auto start = th::interp_track_point(track, 0.0);
    EXPECT_EQ(start.s, 0.0);
    EXPECT_EQ(start.x, 0.0);
    EXPECT_EQ(start.y, 0.0);
    
    // Test end point
    auto end = th::interp_track_point(track, 1.0);
    EXPECT_EQ(end.s, 1.0);
    EXPECT_EQ(end.x, 0.0);
    EXPECT_EQ(end.y, 1.0);
}

TEST(InterpTrackPoint2Test, HeadingInterpolation) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);

    
    auto point = th::interp_track_point(track, 0.5);
    EXPECT_EQ(point.s, 0.5);
    EXPECT_NEAR(point.psi, M_PI/2, 1e-10);  // Should be straight up
}

TEST(InterpTrackPoint2Test, WidthInterpolation) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);
    
    std::vector<double> wl = {1.0, 2.0};
    std::vector<double> wr = {3.0, 4.0};
    track.set_widths(wl, wr);
    
    auto point = th::interp_track_point(track, 0.5);
    EXPECT_EQ(point.wl, 1.5);  // Linear interpolation
    EXPECT_EQ(point.wr, 3.5);
}

TEST(InterpTrackPoint2Test, CurvatureInterpolation) {
    th::Track2<double> track;
    track.push_back(th::TrackPoint2<double>(0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0));    // Zero curvature
    track.push_back(th::TrackPoint2<double>(1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0));    // Some curvature
    
    auto point = th::interp_track_point(track, 0.5);
    EXPECT_EQ(point.kappa, 0.5);  // Linear interpolation of curvature
}

TEST(InterpTrackPoint2Test, OutOfBoundsWarning) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0), th::Point2<double>(0.0, 1.0)};
    th::Track2<double> track(points, false);
    
    // Redirect cerr to capture warning
    std::stringstream buffer;
    std::streambuf* old = std::cerr.rdbuf(buffer.rdbuf());
    
    // Test before start
    auto before = th::interp_track_point(track, -0.5);
    EXPECT_EQ(before.s, 0.0);  // Should return first point
    EXPECT_TRUE(buffer.str().find("Warning") != std::string::npos);
    
    buffer.str("");  // Clear buffer
    
    // Test after end
    auto after = th::interp_track_point(track, 1.5);
    EXPECT_EQ(after.s, 1.0);  // Should return last point
    EXPECT_TRUE(buffer.str().find("Warning") != std::string::npos);
    
    // Restore cerr
    std::cerr.rdbuf(old);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
