#include "PhysicalObject.hpp"

// Initializes an object with the specified mass, position, and velocity.
PhysicalObject::PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity)
    : mass(massValue), velocity(initialVelocity), position(initialPosition) {}

// Adds a force to the accumulated force.
void PhysicalObject::applyForce(const Vector2D& force) {
    accumulatedForce += force;
}

// Returns the acceleration set for the object.
Vector2D PhysicalObject::getAcceleration() const{
    return acceleration;
}

// Returns the velocity of the object.
Vector2D PhysicalObject::getVelocity()const{
    return velocity;
}

// Returns the position of the object.
Vector2D PhysicalObject::getPosition()const{
    return position;
}

// Returns the mass of the object.
double PhysicalObject:: getMass()const {
    return mass;
}

// Sets the acceleration of the object.
void PhysicalObject:: setAcceleration(const Vector2D& NewAcceleration){
    acceleration=NewAcceleration;
}

// Sets the position of the object.
void PhysicalObject:: setPosition(const Vector2D& NewPosition){
    position=NewPosition;
}

// Sets the velocity of the object.
void PhysicalObject:: setVelocity(const Vector2D& NewVelocity){
    velocity=NewVelocity;
}

//Returns the AccumulatedForce of the object
Vector2D PhysicalObject:: getAccumulatedForce()const{
    return accumulatedForce;
}

//Sets the AccumulatedForce in (0,0)
void PhysicalObject:: clearForces(){
    accumulatedForce.x=0;
    accumulatedForce.y=0;
}