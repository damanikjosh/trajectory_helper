#include <gtest/gtest.h>
#include <trajectory_helper/track/track_point.hpp>
#include <cmath>

TEST(TrackPoint2Test, DefaultConstructor) {
    th::TrackPoint2<double> p;
    EXPECT_TRUE(std::isinf(p.s));
    EXPECT_TRUE(std::isinf(p.x));
    EXPECT_TRUE(std::isinf(p.y));
    EXPECT_TRUE(std::isinf(p.psi));
    EXPECT_TRUE(std::isinf(p.wr));
    EXPECT_TRUE(std::isinf(p.wl));
    EXPECT_TRUE(std::isinf(p.kappa));
}

TEST(TrackPoint2Test, XYConstructor) {
    th::TrackPoint2<double> p(1.0, 2.0);
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
    EXPECT_TRUE(std::isinf(p.s));
    EXPECT_TRUE(std::isinf(p.psi));
    EXPECT_TRUE(std::isinf(p.wr));
    EXPECT_TRUE(std::isinf(p.wl));
    EXPECT_TRUE(std::isinf(p.kappa));
}

TEST(TrackPoint2Test, XYPsiConstructor) {
    th::TrackPoint2<double> p(1.0, 2.0, 0.5);
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
    EXPECT_EQ(p.psi, 0.5);
    EXPECT_TRUE(std::isinf(p.s));
    EXPECT_TRUE(std::isinf(p.wr));
    EXPECT_TRUE(std::isinf(p.wl));
    EXPECT_TRUE(std::isinf(p.kappa));
}

TEST(TrackPoint2Test, XYWidthsConstructor) {
    th::TrackPoint2<double> p(1.0, 2.0, 3.0, 4.0);
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
    EXPECT_EQ(p.wl, 3.0);
    EXPECT_EQ(p.wr, 4.0);
    EXPECT_TRUE(std::isinf(p.s));
    EXPECT_TRUE(std::isinf(p.psi));
    EXPECT_TRUE(std::isinf(p.kappa));
}

TEST(TrackPoint2Test, FullConstructor) {
    th::TrackPoint2<double> p(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
    EXPECT_EQ(p.s, 1.0);
    EXPECT_EQ(p.x, 2.0);
    EXPECT_EQ(p.y, 3.0);
    EXPECT_EQ(p.psi, 4.0);
    EXPECT_EQ(p.wl, 5.0);
    EXPECT_EQ(p.wr, 6.0);
    EXPECT_EQ(p.kappa, 7.0);
}

TEST(TrackPoint2Test, ToPoint) {
    th::TrackPoint2<double> tp(1.0, 2.0);
    th::Point2<double> p = tp.to_point();
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
