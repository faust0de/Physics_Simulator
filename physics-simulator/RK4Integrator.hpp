#pragma once
#include "Integrator.hpp"

// Classical RK4 (fourth order); every stage evaluates acceleration at its own immutable state.
class RK4Integrator : public Integrator {
public:
    IntegratorType getType() const override { return IntegratorType::RK4; }
    void integrate(PhysicalObject& object,double dt) const override;
    void integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const override;
    MotionState integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const override;
};
