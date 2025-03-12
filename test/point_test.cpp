#include <gtest/gtest.h>
#include <trajectory_helper/point/point.hpp>
#include <cmath>

TEST(Point2Test, DefaultConstructor) {
    th::Point2<double> p;
    EXPECT_EQ(p.x, 0.0);
    EXPECT_EQ(p.y, 0.0);
}

TEST(Point2Test, ParameterizedConstructor) {
    th::Point2<double> p(3.0, 4.0);
    EXPECT_EQ(p.x, 3.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(Point2Test, ArithmeticOperators) {
    th::Point2<double> p1(1.0, 2.0);
    th::Point2<double> p2(3.0, 4.0);
    
    auto sum = p1 + p2;
    EXPECT_EQ(sum.x, 4.0);
    EXPECT_EQ(sum.y, 6.0);
    
    auto diff = p2 - p1;
    EXPECT_EQ(diff.x, 2.0);
    EXPECT_EQ(diff.y, 2.0);
    
    auto scaled = p1 * 2.0;
    EXPECT_EQ(scaled.x, 2.0);
    EXPECT_EQ(scaled.y, 4.0);
    
    auto divided = p2 / 2.0;
    EXPECT_EQ(divided.x, 1.5);
    EXPECT_EQ(divided.y, 2.0);
}

TEST(Point2Test, DotProduct) {
    th::Point2<double> p1(1.0, 2.0);
    th::Point2<double> p2(3.0, 4.0);
    
    EXPECT_EQ(p1.dot(p2), 11.0);
}

TEST(Point2Test, CrossProduct) {
    th::Point2<double> p1(1.0, 2.0);
    th::Point2<double> p2(3.0, 4.0);
    
    EXPECT_EQ(p1.cross(p2), -2.0);
}

TEST(Point2Test, Norm) {
    th::Point2<double> p(3.0, 4.0);
    EXPECT_EQ(p.norm(), 5.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
