#pragma once

#include "Vector2D.hpp"

class PhysicalObject {
private:
    double mass=1.0;
    Vector2D velocity;
    Vector2D position;
    Vector2D acceleration;
    Vector2D accumulatedForce;
public:
    PhysicalObject() = default; // Initializes an object with unit mass and zero vectors.
    PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity); // Initializes an object with the specified mass, position, and velocity.
    void applyForce(const Vector2D& force); // Adds a force to the accumulated force.
    Vector2D getPosition() const; // Returns the position of the object.
    Vector2D getVelocity() const; // Returns the velocity of the object.
    Vector2D getAcceleration() const; // Returns the acceleration set for the object.
    double getMass()const; // Returns the mass of the object.
    Vector2D getAccumulatedForce() const; //Returns the AccumulatedForce of the object
    
    void clearForces();//Set the AcumulatedForce in (0,0)

    void setPosition(const Vector2D& newPosition); // Sets the position of the object.
    void setVelocity(const Vector2D& newVelocity); // Sets the velocity of the object.
    void setAcceleration(const Vector2D& newAcceleration); // Sets the acceleration of the object.
};

