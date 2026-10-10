#pragma once

#include "Vector2D.hpp"

// Stores the physical state of an object with mass and motion.
class PhysicalObject {
private:
    // Stores the mass in kilograms.
    double mass=1.0;
    // Stores the velocity in meters per second.
    Vector2D velocity;
    // Stores the world position in meters.
    Vector2D position;
    // Stores the prescribed acceleration in meters per second squared.
    Vector2D acceleration;
    // Stores the sum of forces applied during the current physics step.
    Vector2D accumulatedForce;
public:
    // Initializes an object with unit mass and zero vectors.
    PhysicalObject() = default;
    // Initializes an object with the specified mass, position, and velocity.
    PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity);
    // Adds a force to the accumulated force.
    void applyForce(const Vector2D& force);
    // Returns the position of the object.
    Vector2D getPosition() const;
    // Returns the velocity of the object.
    Vector2D getVelocity() const;
    // Returns the acceleration set for the object.
    Vector2D getAcceleration() const;
    // Returns the mass of the object.
    double getMass()const;
    // Returns the AccumulatedForce of the object
    Vector2D getAccumulatedForce() const;
    
    // Set the AcumulatedForce in (0,0)
    void clearForces();

    // Sets the position of the object.
    void setPosition(const Vector2D& newPosition);
    // Sets the velocity of the object.
    void setVelocity(const Vector2D& newVelocity);
    // Sets the acceleration of the object.
    void setAcceleration(const Vector2D& newAcceleration);
};

