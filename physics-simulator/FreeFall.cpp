#include"FreeFall.hpp"
#include <cmath>

// Applies the gravitational force to the object.
void FreeFall::apply(PhysicalObject& object, double /*deltatime*/)const{
    double mass=object.getMass();
    Vector2D gravityForce(0.0,mass*gravity);
    object.applyForce(gravityForce);
}

// Checks whether the object has reached or passed ground level.
bool FreeFall::isFinished(const PhysicalObject& object) const {
    return object.getPosition().y <= groundHeight;
}

// Places the object exactly at ground level while preserving impact velocity.
void FreeFall::finish(PhysicalObject& object) const {
    Vector2D position =object.getPosition();

    // Correct the numerical overshoot
    position.y = groundHeight;

    object.setPosition(position);

    // Keep the velocity at the instant of impact.
    object.clearForces();
}

// Describes a height scale between ground level and the supplied object position.
std::vector<Scale> FreeFall::getScales(const PhysicalObject& object) const {

    double initialHeight =object.getPosition().y;

    double rulerX =-initialHeight * 0.20;

    Scale heightScale(Vector2D(rulerX,groundHeight),Vector2D(rulerX,initialHeight),groundHeight,initialHeight,5,"m");

    return {
        heightScale
    };
}

// Estimates the fall duration from the supplied height assuming release from rest.
double FreeFall::getExpectedDuration(const PhysicalObject& object) const {
    double initialHeight =object.getPosition().y - groundHeight;

    if (initialHeight <= 0.0) {
        return 0.0;
    }

    return std::sqrt((2.0 * initialHeight) /(-gravity));
}

// Returns the constant vertical gravitational acceleration.
Vector2D FreeFall::getAcceleration(const PhysicalObject& object) const {
    return Vector2D(0.0,gravity);
}