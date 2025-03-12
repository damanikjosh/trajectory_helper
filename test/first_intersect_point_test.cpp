#include <gtest/gtest.h>
#include <trajectory_helper/track/first_intersect_point.hpp>
#include <cmath>

TEST(FirstIntersectPointTest, BasicIntersection) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    th::Point2<double> center(0.5, 1.0);
    auto intersection = th::first_intersect_point(track, center, 1.0);
    
    ASSERT_TRUE(intersection.has_value());
    EXPECT_NEAR(intersection->x, 0.5, 1e-10);
    EXPECT_NEAR(intersection->y, 0.0, 1e-10);
    EXPECT_NEAR(intersection->s, 0.5, 1e-10);
}

TEST(FirstIntersectPointTest, NoIntersection) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    th::Point2<double> center(0.5, 2.0);  // Too far to intersect
    auto intersection = th::first_intersect_point(track, center, 1.0);
    
    EXPECT_FALSE(intersection.has_value());
}

TEST(FirstIntersectPointTest, CircularTrackWraparound) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(1.0, 0.0),
        th::Point2<double>(0.0, 1.0),
        th::Point2<double>(-1.0, 0.0),
        th::Point2<double>(0.0, -1.0)
    };
    th::Track2<double> track(points, true);
    
    th::Point2<double> center(0.0, 0.0);
    auto intersection = th::first_intersect_point(track, center, 1.0);
    
    ASSERT_TRUE(intersection.has_value());
    EXPECT_NEAR(std::hypot(intersection->x, intersection->y), 1.0, 1e-10);
}

TEST(FirstIntersectPointTest, MultipleIntersections) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(-1.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    th::Point2<double> center(0.0, 0.0);
    auto intersection = th::first_intersect_point(track, center, 0.5);
    
    ASSERT_TRUE(intersection.has_value());
    // Should return first intersection from nearest point
    EXPECT_NEAR(intersection->x, 0.5, 1e-10);
    EXPECT_NEAR(intersection->y, 0.0, 1e-10);
}

TEST(FirstIntersectPointTest, TangentIntersection) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    th::Point2<double> center(0.5, 1.0);
    auto intersection = th::first_intersect_point(track, center, 1.0);
    
    ASSERT_TRUE(intersection.has_value());
    EXPECT_NEAR(intersection->x, 0.5, 1e-10);
    EXPECT_NEAR(intersection->y, 0.0, 1e-10);
}

TEST(FirstIntersectPointTest, PreservesTrackProperties) {
    std::vector<th::Point2<double>> points = {
        th::Point2<double>(0.0, 0.0),
        th::Point2<double>(1.0, 0.0)
    };
    th::Track2<double> track(points, false);
    
    // Set track properties
    std::vector<double> wl = {1.0, 2.0};
    std::vector<double> wr = {3.0, 4.0};
    track.set_widths(wl, wr);
    
    th::Point2<double> center(0.5, 1.0);
    auto intersection = th::first_intersect_point(track, center, 1.0);
    
    ASSERT_TRUE(intersection.has_value());
    EXPECT_NEAR(intersection->wl, 1.5, 1e-10);
    EXPECT_NEAR(intersection->wr, 3.5, 1e-10);
    EXPECT_NEAR(intersection->psi, 0.0, 1e-10);
}

TEST(FirstIntersectPointTest, EmptyTrackHandling) {
    th::Track2<double> track;
    th::Point2<double> center(0.0, 0.0);
    
    auto intersection = th::first_intersect_point(track, center, 1.0);
    EXPECT_FALSE(intersection.has_value());
}

TEST(FirstIntersectPointTest, SinglePointTrackHandling) {
    std::vector<th::Point2<double>> points = {th::Point2<double>(0.0, 0.0)};
    th::Track2<double> track;
    track.push_back(th::TrackPoint2<double>(0.0, 0.0));
    
    th::Point2<double> center(1.0, 0.0);
    auto intersection = th::first_intersect_point(track, center, 1.0);
    EXPECT_FALSE(intersection.has_value());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
