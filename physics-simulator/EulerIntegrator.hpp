#pragma once

#include "Integrator.hpp"

class EulerIntegrator:public Integrator{
public:
    void integrate(PhysicalObject& object, double deltatime)const override; //Updates position and velocity using explicit Euler integration and clears accumulated forces.
};