#include "PhysicalObject.hpp"

PhysicalObject::PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity)
    : mass(massValue), position(initialPosition), velocity(initialVelocity), accumulatedForce() {}

void PhysicalObject::applyForce(const Vector2D& force) {
    accumulatedForce += force;
}

void PhysicalObject::integrate(double deltaTime) {
    if (mass <= 0.0 || deltaTime <= 0.0) {
        accumulatedForce = Vector2D();
        return;
    }

    const Vector2D acceleration = accumulatedForce * (1.0 / mass);
    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;
    accumulatedForce = Vector2D();
}

const Vector2D& PhysicalObject::getPosition() const {
    return position;
}

const Vector2D& PhysicalObject::getVelocity() const {
    return velocity;
}
