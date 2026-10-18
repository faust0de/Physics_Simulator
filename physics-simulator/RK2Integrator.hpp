#pragma once
#include "Integrator.hpp"

// Explicit Midpoint (second order); every stage evaluates acceleration at its own immutable state.
class RK2Integrator : public Integrator {
public:
    IntegratorType getType() const override { return IntegratorType::RK2; }
    void integrate(PhysicalObject& object,double dt) const override;
    void integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const override;
    MotionState integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const override;
};
