#include "RK4Integrator.hpp"

MotionState RK4Integrator::integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const {
    if (dt <= 0.0) return initial;
    Vector2D k1x = initial.velocity;
    Vector2D k1v = acceleration(initial);
    MotionState second{initial.position+k1x*(dt*0.5),initial.velocity+k1v*(dt*0.5)};
    Vector2D k2x = second.velocity;
    Vector2D k2v = acceleration(second);
    MotionState third{initial.position+k2x*(dt*0.5),initial.velocity+k2v*(dt*0.5)};
    Vector2D k3x = third.velocity;
    Vector2D k3v = acceleration(third);
    MotionState fourth{initial.position+k3x*dt,initial.velocity+k3v*dt};
    Vector2D k4x = fourth.velocity;
    Vector2D k4v = acceleration(fourth);
    return {initial.position+(k1x+k2x*2.0+k3x*2.0+k4x)*(dt/6.0),
        initial.velocity+(k1v+k2v*2.0+k3v*2.0+k4v)*(dt/6.0)};
}

void RK4Integrator::integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const {
    apply(object,dt,acceleration);
}

void RK4Integrator::integrate(PhysicalObject& object,double dt) const {
    (void)object; (void)dt;
    throw std::logic_error("RK4 requires an acceleration evaluator; cached acceleration is insufficient.");
}
