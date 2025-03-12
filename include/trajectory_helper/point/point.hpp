#ifndef TRAJECTORY_HELPER__POINT__POINT_HPP
#define TRAJECTORY_HELPER__POINT__POINT_HPP

namespace th {

template<typename T>
struct Point2 {
    T x, y;

    Point2() : x(T()), y(T()) {}
    Point2(T x, T y) : x(x), y(y) {}

    Point2 operator+(const Point2& p) const { return Point2(x + p.x, y + p.y); }
    Point2 operator-(const Point2& p) const { return Point2(x - p.x, y - p.y); }
    Point2 operator*(T scale) const { return Point2(x * scale, y * scale); }
    Point2 operator/(T scale) const { return Point2(x / scale, y / scale); }

    Point2& operator+=(const Point2& p) { x += p.x; y += p.y; return *this; }
    Point2& operator-=(const Point2& p) { x -= p.x; y -= p.y; return *this; }
    Point2& operator*=(T scale) { x *= scale; y *= scale; return *this; }
    Point2& operator/=(T scale) { x /= scale; y /= scale; return *this; }

    // dot product
    T dot(const Point2& p) const { return x * p.x + y * p.y; }
    // cross product
    T cross(const Point2& p) const { return x * p.y - y * p.x; }

    T norm() const { return std::hypot(x, y); }
};

// Common type definitions
typedef Point2<int> Point2i;
typedef Point2<float> Point2f;
typedef Point2<double> Point2d;

}  // namespace th

#endif  // TRAJECTORY_HELPER__POINT__POINT_HPP