#ifndef PHYSICALOBJECT_HPP
#define PHYSICALOBJECT_HPP

#include "Vector2D.hpp"

class PhysicalObject {
public:
    PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity);

    void applyForce(const Vector2D& force);
    void integrate(double deltaTime);

    const Vector2D& getPosition() const;
    const Vector2D& getVelocity() const;

private:
    double mass;
    Vector2D position;
    Vector2D velocity;
    Vector2D accumulatedForce;
};

#endif
