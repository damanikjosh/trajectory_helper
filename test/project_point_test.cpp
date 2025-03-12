#include <gtest/gtest.h>
#include <trajectory_helper/track/project_point.hpp>
#include <cmath>

TEST(ProjectPointTest, ProjectToStraightLine) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    th::Point2<double> point(0.5, 1.0);  // Point above the line
    auto projected = th::project_point(track, point);
    
    EXPECT_NEAR(projected.x, 0.5, 1e-10);
    EXPECT_NEAR(projected.y, 0.0, 1e-10);
    EXPECT_NEAR(projected.s, 0.5, 1e-10);
}

TEST(ProjectPointTest, ProjectToClosedTrack) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0),
        th::Point2<double>(1.0, 1.0),
        th::Point2<double>(0.0, 1.0)
    };
    th::Track2<double> track(points, true);
    
    th::Point2<double> point(0.5, 0.5);  // Point in the middle
    auto projected = th::project_point(track, point);
    
    // Should project to nearest edge
    EXPECT_TRUE((projected.x == 0.5 && projected.y == 0.0) ||  // Bottom edge
                (projected.x == 1.0 && projected.y == 0.5) ||  // Right edge
                (projected.x == 0.5 && projected.y == 1.0) ||  // Top edge
                (projected.x == 0.0 && projected.y == 0.5));   // Left edge
}

TEST(ProjectPointTest, ProjectToEndPoints) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    // Project point before start
    th::Point2<double> start_point(-1.0, 0.0);
    auto projected_start = th::project_point(track, start_point);
    EXPECT_NEAR(projected_start.x, 0.0, 1e-10);
    EXPECT_NEAR(projected_start.y, 0.0, 1e-10);
    EXPECT_NEAR(projected_start.s, 0.0, 1e-10);
    
    // Project point after end
    th::Point2<double> end_point(2.0, 0.0);
    auto projected_end = th::project_point(track, end_point);
    EXPECT_NEAR(projected_end.x, 1.0, 1e-10);
    EXPECT_NEAR(projected_end.y, 0.0, 1e-10);
    EXPECT_NEAR(projected_end.s, 1.0, 1e-10);
}

TEST(ProjectPointTest, ProjectToCorner) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0),
        th::Point2<double>(1.0, 1.0)
    };
    th::Track2<double> track(points, false);
    
    // Point near the corner
    th::Point2<double> corner_point(1.1, 0.1);
    auto projected = th::project_point(track, corner_point);
    EXPECT_NEAR(projected.x, 1.0, 1e-10);
    EXPECT_NEAR(projected.y, 0.1, 1e-10);
}

TEST(ProjectPointTest, ProjectToOutbond) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0),
        th::Point2<double>(1.0, 1.0)
    };
    th::Track2<double> track(points, false);
    
    // Point near the corner
    th::Point2<double> corner_point(2.0, 2.0);
    auto projected = th::project_point(track, corner_point);
    EXPECT_NEAR(projected.x, 1.0, 1e-10);
    EXPECT_NEAR(projected.y, 1.0, 1e-10);
}

TEST(ProjectPointTest, EmptyTrackHandling) {
    th::Track2<double> track;
    th::Point2<double> point(0.0, 0.0);
    
    EXPECT_THROW(th::project_point(track, point), std::runtime_error);
}

TEST(ProjectPointTest, SinglePointTrackHandling) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(1.0, 1.0)
    };
    th::Track2<double> track;
    
    th::Point2<double> point(0.0, 0.0);
    EXPECT_THROW(th::project_point(track, point), std::runtime_error);
}

TEST(ProjectPointTest, PreservesTrackProperties) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, true);
    
    // Set track properties
    std::vector<double> wl = {1.0, 2.0};
    std::vector<double> wr = {3.0, 4.0};
    track.set_widths(wl, wr);
    
    th::Point2<double> point(0.5, 1.0);
    auto projected = th::project_point(track, point);
    
    // Check that properties were interpolated
    EXPECT_NEAR(projected.wl, 1.5, 1e-10);
    EXPECT_NEAR(projected.wr, 3.5, 1e-10);
    EXPECT_NEAR(projected.psi, 0.0, 1e-10);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
