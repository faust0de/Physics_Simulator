#pragma once

class Vector2D {
public:
    double x;
    double y;

    Vector2D(double xValue=0.0, double yValue=0.0); // Initializes a vector with the specified components or zero by default.

    Vector2D operator+(const Vector2D& other)const; // Returns the sum of this vector and another vector.
    Vector2D operator-(const Vector2D& other)const; // Returns the result of subtracting another vector from this vector.
    Vector2D operator*(const double& other)const;   // Returns this vector multiplied by a scalar.
    Vector2D operator/(const double& other)const;   // Returns this vector divided by a scalar.
    Vector2D& operator+=(const Vector2D& other); // Adds another vector to this vector and returns a reference to the result.
    Vector2D& operator-=(const Vector2D& other); // Subtracts another vector from this vector and returns a reference to the result.
    Vector2D& operator*=(const double& other); // Multiplies this vector by a scalar and returns a reference to the result.
    Vector2D& operator/=(const double& other); // Divides this vector by a scalar and returns a reference to the result.
    Vector2D normalized()const; // Returns a unit vector in the same direction, or a zero vector if the magnitude is zero.

    double magnitudeSquared() const; // Returns the squared magnitude of this vector.
    double magnitude() const;                       // Returns the magnitude of this vector.
    double dot(const Vector2D& other)const; // Returns the dot product of this vector and another vector.
    double cross(const Vector2D& other)const; // Returns the scalar cross product of this vector and another vector.
    double distanceTo(const Vector2D& other)const; // Returns the distance between this vector and another vector.
};
