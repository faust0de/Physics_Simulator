#pragma once
#include "PhysicalObject.hpp"
#include <functional>
#include <stdexcept>

// Selects a numerical strategy independently of the physical regime.
enum class IntegratorType { Euler, RK2, RK4 };

struct MotionState {
    Vector2D position;
    Vector2D velocity;
};

using AccelerationEvaluator = std::function<Vector2D(const MotionState&)>;

// Numerical strategies do not own physical models, phases, clocks, or targets.
class Integrator {
protected:
    // Accumulated external force remains constant over the step, as in legacy Euler.
    void apply(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const {
        if (object.getMass() <= 0.0 || dt <= 0.0) return;
        Vector2D forceAcceleration = object.getAccumulatedForce()/object.getMass();
        MotionState initial{object.getPosition(),object.getVelocity()};
        MotionState result = integrate(initial,dt,[&](const MotionState& state) {
            return acceleration(state)+forceAcceleration;
        });
        object.setPosition(result.position);
        object.setVelocity(result.velocity);
        object.clearForces();
    }
public:
    virtual ~Integrator() = default;
    virtual IntegratorType getType() const { return IntegratorType::Euler; }
    // Retains the existing constant-acceleration entry point for callers and adapters.
    virtual void integrate(PhysicalObject& object,double dt) const = 0;
    virtual MotionState integrate(const MotionState& initial,double dt,const AccelerationEvaluator& acceleration) const {
        (void)initial; (void)dt; (void)acceleration;
        throw std::logic_error("Integrator does not support state evaluation.");
    }
    // Legacy custom integrators can continue to implement the original object interface.
    virtual void integrate(PhysicalObject& object,double dt,const AccelerationEvaluator& acceleration) const {
        object.setAcceleration(acceleration({object.getPosition(),object.getVelocity()}));
        integrate(object,dt);
    }
};
