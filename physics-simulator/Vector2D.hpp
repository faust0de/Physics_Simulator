#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

class Vector2D {
public:
    double x;
    double y;

    Vector2D();
    Vector2D(double xValue, double yValue);

    Vector2D operator+(const Vector2D& other) const;
    Vector2D operator-(const Vector2D& other) const;
    Vector2D operator*(double scalar) const;

    Vector2D& operator+=(const Vector2D& other);
    Vector2D& operator-=(const Vector2D& other);
    Vector2D& operator*=(double scalar);
};

#endif
