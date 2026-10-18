#include "RK2Integrator.hpp"

MotionState RK2Integrator::integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const {
    if (dt <= 0.0) return initial;
    Vector2D k1x = initial.velocity;
    Vector2D k1v = acceleration(initial);
    MotionState midpoint{initial.position+k1x*(dt*0.5),initial.velocity+k1v*(dt*0.5)};
    Vector2D k2x = midpoint.velocity;
    Vector2D k2v = acceleration(midpoint);
    return {initial.position+k2x*dt,initial.velocity+k2v*dt};
}

void RK2Integrator::integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const {
    apply(object,dt,acceleration);
}

void RK2Integrator::integrate(PhysicalObject& object,double dt) const {
    (void)object; (void)dt;
    throw std::logic_error("RK2 requires an acceleration evaluator; cached acceleration is insufficient.");
}
