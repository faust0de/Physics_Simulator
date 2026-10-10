#include "Vector2D.hpp"
#include <cmath>

// Initializes a vector with the specified components.
Vector2D::Vector2D(double xValue, double yValue){
    x=xValue;
    y=yValue;
}

// Returns the sum of this vector and another vector.
Vector2D Vector2D::operator+(const Vector2D& other)const{
    return Vector2D (x+other.x,y+other.y);
}

// Returns the result of subtracting another vector from this vector.
Vector2D Vector2D:: operator-(const Vector2D& other)const{
    return Vector2D (x-other.x,y-other.y);
}

// Returns this vector multiplied by a scalar.
Vector2D Vector2D:: operator*(const double& other)const{
    return Vector2D (x*other,y*other);
}

// Returns this vector divided by a scalar.
Vector2D Vector2D:: operator/(const double& other)const{
    return Vector2D (x/other,y/other);
}

// Returns a unit vector in the same direction, or a zero vector if the magnitude is zero.
Vector2D Vector2D::normalized()const{
    double mag= magnitude();
    // Avoids division by zero when normalizing a zero vector.
    if(mag==0){
        return Vector2D(0.0,0.0);
    }
    return Vector2D(x/mag,y/mag);
}

// Adds another vector to this vector and returns a reference to the result.
Vector2D& Vector2D::operator+=(const Vector2D& other){
    x+=other.x;
    y+=other.y;
    return *this;
}
// Subtracts another vector from this vector and returns a reference to the result.
Vector2D& Vector2D::operator-=(const Vector2D& other){
    x-=other.x;
    y-=other.y;
    return *this;
}

// Multiplies this vector by a scalar and returns a reference to the result.
Vector2D& Vector2D::operator*=(const double& other){
    x*=other;
    y*=other;
    return *this;
}

// Divides this vector by a scalar and returns a reference to the result.
Vector2D& Vector2D::operator/=(const double& other){
    x/=other;
    y/=other;
    return *this;
}

// Returns the squared magnitude of this vector.
double Vector2D::magnitudeSquared() const{
    double mag= pow(x,2)+pow(y,2);
    return mag;
}

// Returns the magnitude of this vector.
double Vector2D::magnitude() const{
    double mag= sqrt(pow(x,2)+pow(y,2));
    return mag;
}

// Returns the dot product of this vector and another vector.
double Vector2D:: dot(const Vector2D& other)const{
    double product= x*other.x+y*other.y;
    return product;
}

// Returns the scalar cross product of this vector and another vector.
double Vector2D::cross(const Vector2D& other)const{
    double product= x*other.y-other.x*y;
    return product;
}

// Returns the distance between this vector and another vector.
double Vector2D::distanceTo(const Vector2D&other)const{
    double d= sqrt(pow(other.x-x,2)+pow(other.y-y,2));
    return d;
}
