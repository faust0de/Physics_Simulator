#pragma once
#include "Integrator.hpp"

// Explicit Euler advances position using the old velocity, never the new velocity.
class EulerIntegrator : public Integrator {
public:
    void integrate(PhysicalObject& object,double dt) const override;
    void integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const override;
    MotionState integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const override;
};
