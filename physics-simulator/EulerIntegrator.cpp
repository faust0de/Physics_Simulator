#include "EulerIntegrator.hpp"

MotionState EulerIntegrator::integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const {
    if (dt <= 0.0) return initial;
    Vector2D a = acceleration(initial);
    return {initial.velocity*dt+initial.position,initial.velocity+a*dt};
}

void EulerIntegrator::integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const {
    apply(object,dt,acceleration);
}

void EulerIntegrator::integrate(PhysicalObject& object,double dt) const {
    Vector2D acceleration = object.getAcceleration();
    apply(object,dt,[&](const MotionState&) { return acceleration; });
}
