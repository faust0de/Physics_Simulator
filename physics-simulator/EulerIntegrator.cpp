#include "EulerIntegrator.hpp"

// Updates position and velocity using explicit Euler integration and clears accumulated forces.
void EulerIntegrator::integrate(PhysicalObject& object,double deltatime)const{
    if(object.getMass()<=0||deltatime<=0){
        return;
    }
    //Gets actual values
    Vector2D position= object.getPosition();
    Vector2D velocity= object.getVelocity();
    Vector2D acceleration= object.getAcceleration();
    Vector2D ForceAcceleration=object.getAccumulatedForce()/object.getMass();
    Vector2D TotalAcceleration= acceleration+ForceAcceleration;

    //Explicit Euler Method
    Vector2D NewPosition= velocity*deltatime + position;
    Vector2D NewVelocity= velocity + TotalAcceleration*deltatime;
    
    //Update State
    object.setPosition(NewPosition);
    object.setVelocity(NewVelocity);

    object.clearForces();
}

