#pragma once

#include "Integrator.hpp"

// Implements the explicit Euler numerical integration strategy.
class EulerIntegrator:public Integrator{
public:
    // Updates position and velocity using explicit Euler integration and clears accumulated forces.
    void integrate(PhysicalObject& object, double deltatime)const override;
};