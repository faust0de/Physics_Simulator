#pragma once
#include "PhysicalObject.hpp"

class Integrator{
public:
    // Allows derived integrators to be destroyed through a base class pointer.
    virtual ~Integrator()=default;

    // Defines how an integrator updates an object over a time step.
    virtual void integrate(PhysicalObject& object, double deltatime)const=0;
};