#include"FreeFall.hpp"

// Applies the gravitational force to the object.
void FreeFall::apply(PhysicalObject& object, double /*deltatime*/)const{
    double mass=object.getMass();
    Vector2D gravityForce(0.0,mass*gravity);
    object.applyForce(gravityForce);
}
