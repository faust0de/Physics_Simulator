#include "Simulation.hpp"
#include "IntegratorFactory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Initializes physical time and playback state before configuration.
Simulation::Simulation(): current_time(0.0),timeScale(1.0),hasInitialState(false),paused(false),termination(),traveledDistance(0.0),physicalDuration(0.0) {
}

// Coordinates one physics step and finalizes the kinematics when it ends.
void Simulation::step(PhysicalObject& object,const Kinematics& kinematics,const Integrator& integrator,double frameDeltaTime) {
    (void)kinematics;
    if (paused || sequenceComplete) return;
    double remaining = frameDeltaTime*timeScale;
    if (!std::isfinite(remaining) || remaining <= 0.0) return;
    if (explicitMotionSequence) {
        if (seekCheckpoint) {
            // Resume on the original numerical grid instead of splitting an Euler step twice.
            double budget = current_time-seekCheckpoint->current_time+pendingPhysicalTime+remaining;
            double required = std::min(physicsStep,std::max(0.0,physicalDuration-seekCheckpoint->current_time));
            if (budget+1e-12 < required) { pendingPhysicalTime += remaining; return; }
            auto saved = seekCheckpoint;
            PhysicalObject savedObject = seekCheckpointObject;
            *this = *saved;
            object.setPosition(savedObject.getPosition());
            object.setVelocity(savedObject.getVelocity());
            object.setAcceleration(savedObject.getAcceleration());
            paused = false;
            pendingPhysicalTime = budget;
            remaining = 0.0;
            seekCheckpoint.reset();
        }
        // Frame fractions remain pending so playback and preflight use the same physical grid.
        pendingPhysicalTime += remaining;
        while (!paused && !sequenceComplete && pendingPhysicalTime+1e-12 >= physicsStep) {
            consumeTime(object,integrator,physicsStep);
            pendingPhysicalTime = std::max(0.0,pendingPhysicalTime-physicsStep);
        }
        if (!sequenceComplete && !paused && physicalDuration > 0.0 &&
            current_time+pendingPhysicalTime >= physicalDuration-1e-12) {
            double before = current_time;
            consumeTime(object,integrator,physicsStep);
            pendingPhysicalTime = std::max(0.0,pendingPhysicalTime-(current_time-before));
        }
        return;
    }
    if (!currentKinematics->isCircular() && !(targetKinematics && targetKinematics->isCircular())) {
        consumeTime(object,integrator,remaining);
        return;
    }
    while (remaining > 0.0 && !paused) {
        double dt = std::min(remaining,physicsStep);
        consumeTime(object,integrator,dt);
        remaining -= dt;
    }
}

// Both bodies share this numerical path during playback and seek.
double Simulation::advance(PhysicalObject& object,const Kinematics& kinematics,const Integrator& integrator,double physicalDeltaTime,double stopTime) {
    if (!std::isfinite(physicalDeltaTime) || physicalDeltaTime <= 0.0) return 0.0;
    if ((!explicitMotionSequence && !kinematics.isCircular()) || termination.Type == TerminationType::Time || termination.Type == TerminationType::Target) {
        if (termination.Type != TerminationType::Natural && phaseExpectedDuration > phaseElapsedTime &&
            physicalDeltaTime > phaseExpectedDuration-phaseElapsedTime)
            physicalDeltaTime = phaseExpectedDuration-phaseElapsedTime;
    }
    if (termination.Type == TerminationType::Distance) {
        double speed = object.getVelocity().magnitude();
        if (speed > 0.0) {
            double remaining = (termination.value-phaseTraveledDistance)/speed;
            if (remaining < physicalDeltaTime-1e-12*std::max(1.0,physicalDeltaTime)) physicalDeltaTime = remaining;
        }
    }
    PhysicalObject previous = object;
    PhysicalObject previousTarget = targetObject;
    bool targetWasFinished = targetMotionFinished;
    object.setAcceleration(kinematics.getAcceleration(object));
    integrator.integrate(object,physicalDeltaTime,[&](const MotionState& state) {
        return kinematics.getAcceleration(state.position,state.velocity);
    });
    PhysicalObject rawPrimary = object;
    if (targetKinematics && !targetWasFinished) {
        targetObject.setAcceleration(targetKinematics->getAcceleration(targetObject));
        integrator.integrate(targetObject,physicalDeltaTime,[&](const MotionState& state) {
            return targetKinematics->getAcceleration(state.position,state.velocity);
        });
    }
    PhysicalObject rawTarget = targetObject;
    if (explicitMotionSequence) {
        for (const Vector2D& value : {rawPrimary.getPosition(),rawPrimary.getVelocity(),rawTarget.getPosition(),rawTarget.getVelocity()})
            if (!std::isfinite(value.x) || !std::isfinite(value.y))
                throw std::runtime_error("Sequence integration produced a non-finite physical state.");
    }
    bool targetEnds = targetKinematics && !targetWasFinished && targetKinematics->isFinished(rawTarget);
    double targetBoundary = targetEnds ? targetKinematics->getNaturalBoundaryFraction(previousTarget,rawTarget) : 1.0;
    bool primaryEnds = kinematics.isFinished(rawPrimary);
    double naturalBoundary = primaryEnds ? kinematics.getNaturalBoundaryFraction(previous,rawPrimary) : 1.0;
    double limit = (explicitMotionSequence || termination.Type == TerminationType::Target || integrator.getType() != IntegratorType::Euler) && primaryEnds ? naturalBoundary : 1.0;

    if (integrator.getType() != IntegratorType::Euler && termination.Type == TerminationType::Distance) {
        double segment = rawPrimary.getPosition().distanceTo(previous.getPosition());
        if (segment > 0.0) limit = std::min(limit,std::clamp((termination.value-phaseTraveledDistance)/segment,0.0,1.0));
    }
    // Event interpolation uses only the integrator's endpoints, never analytical positions.
    auto interpolate = [](const PhysicalObject& start,const PhysicalObject& end,double u) {
        if (u >= 1.0) return end;
        PhysicalObject state = end;
        state.setPosition(start.getPosition()+(end.getPosition()-start.getPosition())*u);
        state.setVelocity(start.getVelocity()+(end.getVelocity()-start.getVelocity())*u);
        return state;
    };
    bool numericalAnglePrefix = integrator.getType() != IntegratorType::Euler && termination.Type == TerminationType::Angle;
    auto prefix = [&](const PhysicalObject& start,const PhysicalObject& end,const Kinematics& model,double u) {
        if (!numericalAnglePrefix || u >= 1.0) return interpolate(start,end,u);
        // Reintegrate angular-event prefixes with the same RK method, never an ideal circle.
        Vector2D force = start.getAccumulatedForce()/start.getMass();
        MotionState state = integrator.integrate(MotionState{start.getPosition(),start.getVelocity()},physicalDeltaTime*u,
            [&](const MotionState& value) { return model.getAcceleration(value.position,value.velocity)+force; });
        PhysicalObject result = end;
        result.setPosition(state.position);
        result.setVelocity(state.velocity);
        return result;
    };
    auto primaryAt = [&](double u) { return prefix(previous,rawPrimary,kinematics,u); };
    auto targetAt = [&](double u) {
        if (!targetKinematics || targetWasFinished) return previousTarget;
        PhysicalObject state = prefix(previousTarget,rawTarget,*targetKinematics,targetEnds ? std::min(u,targetBoundary) : u);
        if (targetEnds && u >= targetBoundary) targetKinematics->finish(state);
        return state;
    };
    // Localize circular limits on the numerical segment before observing contacts.
    bool angularBoundary = false;
    if (explicitMotionSequence || kinematics.isCircular()) {
        auto locate = [&](auto reached) {
            double lo = 0.0,hi = limit;
            for (int i=0;i<55;++i) {
                double mid = (lo+hi)*0.5;
                if (reached(primaryAt(mid))) hi = mid; else lo = mid;
            }
            return hi;
        };
        if (termination.Type == TerminationType::Angle) {
            double remaining = termination.value*(std::acos(-1.0)/180.0)-phaseTraveledAngle;
            if (kinematics.getAngularIncrement(previous.getPosition(),primaryAt(limit).getPosition()) >= remaining)
                limit = locate([&](const PhysicalObject& state) {
                    return kinematics.getAngularIncrement(previous.getPosition(),state.getPosition()) >= remaining;
                });
        }
        if (termination.Type == TerminationType::Speed && explicitMotionSequence) {
            // Intersect the numerical velocity segment with the requested speed circle.
            // This also finds a zero-speed crossing even if the trial step reverses velocity.
            Vector2D v0 = previous.getVelocity(),dv = rawPrimary.getVelocity()-v0;
            double a = dv.magnitudeSquared();
            if (a > 0.0) {
                double center = -v0.dot(dv)/a;
                Vector2D closest = v0+dv*center;
                double gap = termination.value*termination.value-closest.magnitudeSquared();
                double speedTolerance = 1e-8*std::max({1.0,phaseInitialState.getVelocity().magnitude(),termination.value});
                double tolerance = speedTolerance*speedTolerance;
                if (gap >= -tolerance) {
                    double half = std::sqrt(std::max(0.0,gap)/a);
                    for (double u : {center-half,center+half})
                        if (u >= 0.0 && u <= limit) limit = u;
                }
            }
        }
        else if (termination.Type == TerminationType::Speed && termination.value > 0.0) {
            bool increasing = termination.value > phaseInitialState.getVelocity().magnitude();
            auto reached = [&](const PhysicalObject& state) {
                return increasing ? state.getVelocity().magnitude() >= termination.value :
                    state.getVelocity().magnitude() <= termination.value;
            };
            if (reached(primaryAt(limit))) limit = locate(reached);
        }
        // A numerical angular stop can retain radial Euler velocity, especially for Speed=0.
        // Stop before reversal without claiming that the full velocity magnitude is zero.
        if (kinematics.hasAngularReversal(primaryAt(limit))) {
            limit = locate([&](const PhysicalObject& state) { return kinematics.hasAngularReversal(state); });
            angularBoundary = true;
        }
    }
    double eventLimit = limit;
    // Seek observes the canonical numerical step without shortening its RK trial stages.
    limit = std::min(limit,std::clamp((stopTime-current_time)/physicalDeltaTime,0.0,1.0));
    auto intervals = [&](double end) {
        std::vector<double> cuts{0.0};
        if (targetEnds && targetBoundary > 0.0 && targetBoundary < end) cuts.push_back(targetBoundary);
        cuts.push_back(end);
        return cuts;
    };
    TargetHitResult contact;
    if (target.enabled && !phaseTargetHit.hit) {
        double contactLimit = limit;
        if (termination.Type == TerminationType::Target && std::isfinite(stopTime))
            contactLimit = std::min(eventLimit,std::clamp((stopTime-current_time+1e-12)/physicalDeltaTime,0.0,1.0));
        auto cuts = intervals(contactLimit);
        for (std::size_t i=1;i<cuts.size() && !contact.hit;++i) {
            double lo=cuts[i-1],hi=cuts[i];
            auto p0=primaryAt(lo),p1=primaryAt(hi),t0=targetAt(lo),t1=targetAt(hi);
            contact = TargetDetector::detectRelative(target,p0.getPosition(),p1.getPosition(),t0.getPosition(),t1.getPosition(),
                p0.getVelocity(),p1.getVelocity(),t0.getVelocity(),t1.getVelocity(),
                current_time+physicalDeltaTime*lo,current_time+physicalDeltaTime*hi);
            if (contact.hit) contact.fraction=lo+(hi-lo)*contact.fraction;
        }
    }
    bool stopAtHit = termination.Type == TerminationType::Target && contact.hit;
    double used = stopAtHit ? contact.fraction : limit;
    PhysicalObject usedPrimary = primaryAt(used);
    object.setPosition(usedPrimary.getPosition());
    object.setVelocity(usedPrimary.getVelocity());
    object.setAcceleration(usedPrimary.getAcceleration());
    targetObject = targetAt(used);
    if (stopAtHit) {
        object.setPosition(contact.position);
        object.setVelocity(contact.velocity);
        targetObject.setPosition(contact.targetPosition);
        targetObject.setVelocity(contact.targetVelocity);
    }
    targetMotionFinished = targetWasFinished || (targetEnds && used >= targetBoundary);
    if (contact.hit) {
        phaseTargetHit = contact;
        if (!targetHit.hit) targetHit = contact;
    }
    if (target.enabled) {
        // Minimum separation and traveled distance include only the fraction actually used.
        auto cuts=intervals(used);
        for (std::size_t i=1;i<cuts.size();++i) {
            Vector2D r0=primaryAt(cuts[i-1]).getPosition()-targetAt(cuts[i-1]).getPosition();
            Vector2D r1=primaryAt(cuts[i]).getPosition()-targetAt(cuts[i]).getPosition();
            minimumTargetDistance=std::min(minimumTargetDistance,TargetDetector::minimumDistance(r0,r1,Vector2D()));
        }
    }
    double distance = object.getPosition().distanceTo(previous.getPosition());
    double angle = kinematics.getAngularIncrement(previous.getPosition(),object.getPosition());
    traveledDistance += distance;
    phaseTraveledDistance += distance;
    traveledAngle += angle;
    phaseTraveledAngle += angle;
    angularStopReached = angularBoundary && used >= eventLimit;
    if (kinematics.isCircular()) object.setAcceleration(kinematics.getAcceleration(object));
    if (targetKinematics && targetKinematics->isCircular())
        targetObject.setAcceleration(targetKinematics->getAcceleration(targetObject));
    double consumed = physicalDeltaTime*used;
    current_time = stopAtHit ? contact.time : current_time+consumed;
    phaseElapsedTime += consumed;
    bool naturalFinished = !stopAtHit && primaryEnds && used >= (explicitMotionSequence ? naturalBoundary : limit);
    if (naturalFinished) kinematics.finish(object);
    if (!explicitMotionSequence || trajectoryHistory.empty() || trajectoryHistory.back().distanceTo(object.getPosition()) > 0.0)
        trajectoryHistory.push_back(object.getPosition());
    if (explicitMotionSequence) {
        includePreviewPoint(object.getPosition());
        if (target.enabled) includePreviewPoint(targetObject.getPosition(),target.radius);
    }
    return consumed;
}

// Returns the elapsed physical simulation time in seconds.
double Simulation:: getCurrentTime() const{
    return current_time;
}

// Returns the ratio of physical time to real playback time.
double Simulation:: getTimeScale() const{
    return timeScale;
}

// Restores the saved initial physical state and resets physical time.
void Simulation::reset(PhysicalObject& object) {
    if (!hasInitialState) {
        return;
    }

    object = initialState;

    current_time = 0.0;
    traveledDistance = 0.0;
    traveledAngle = 0.0;
    angularStopReached = false;
    trajectoryHistory = {object.getPosition()};
    pendingPhysicalTime = 0.0;
    seekCheckpoint.reset();
    sequenceComplete = false;
    targetMiss = false;
    if (phaseLocalTargets) {
        targetHit = TargetHitResult();
        initializeTarget(TrajectoryTarget(),object);
    } else configureTarget(sharedTarget);
    if (!motionSequence.empty()) activatePhase(0,object);
}

// Saves the initial state and relates physical duration to real playback duration.
void Simulation::configureTimeScale(const PhysicalObject& object,const Kinematics& kinematics,const TerminationCondition& terminationCondition,double playbackDuration,IntegratorType integratorType) {
    trajectoryHistory = {object.getPosition()};
    initialState = object;
    hasInitialState = true;
    explicitMotionSequence = false;
    phaseLocalTargets = false;
    preflightTargetMiss = false;
    motionSequence = {MotionPhase{kinematics.getConfiguration(),terminationCondition,false,0.0,std::nullopt,integratorType}};
    currentKinematics = kinematics;
    currentIntegrator = IntegratorFactory::create(integratorType);
    if (!currentIntegrator) throw std::runtime_error("Unable to create selected integrator.");
    preflightFinalState.reset();
    currentPhaseIndex = 0;
    phaseInitialState = object;
    termination = terminationCondition;
    traveledDistance = traveledAngle = 0.0;
    phaseElapsedTime = phaseTraveledDistance = phaseTraveledAngle = 0.0;
    angularStopReached = sequenceComplete = targetMiss = false;
    configureTarget(target);
    physicalDuration = phaseExpectedDuration = kinematics.getExpectedDuration(object,termination);
    currentVisuals = kinematics.getVisuals(object,termination);
    currentProjectileMetrics = kinematics.isProjectile() ? kinematics.getProjectileMetrics(object) : ProjectileMetrics();
    timeScale = playbackDuration > 0.0 && physicalDuration > 0.0 ? physicalDuration/playbackDuration : 1.0;
}

// Reconstructs the physical state at the requested physical time.
void Simulation::seek(PhysicalObject& object,const Kinematics& kinematics,const Integrator& integrator,double targetTime) {
    if (!hasInitialState) return;
    targetTime=std::max(0.0,targetTime);
    reset(object);
    bool canonicalRK = explicitMotionSequence && std::any_of(motionSequence.begin(),motionSequence.end(),
        [](const MotionPhase& phase) { return phase.integrator != IntegratorType::Euler; });
    while (current_time < targetTime && !isFinished(object,kinematics)) {
        double previousTime=current_time;
        double dt = std::min(physicsStep,targetTime-current_time);
        if (explicitMotionSequence && dt < physicsStep-1e-12 && dt > 1e-12) {
            seekCheckpointObject = object;
            seekCheckpoint = std::make_shared<Simulation>(*this);
            seekCheckpoint->seekCheckpoint.reset();
        }
        if (canonicalRK) consumeTime(object,integrator,physicsStep,targetTime);
        else consumeTime(object,integrator,dt);
        if (current_time <= previousTime) break;
    }
}

// Checks external limits independently of the kinematics's natural end.
bool Simulation::hasReachedTermination(const PhysicalObject& object) const {
    switch (termination.Type) {
        case TerminationType::Angle:
            return phaseTraveledAngle >= termination.value*(std::acos(-1.0)/180.0)-1e-12;
        case TerminationType::Natural:
            return false;
        case TerminationType::Target:
            return phaseTargetHit.hit || phaseElapsedTime >= termination.maxTime-(explicitMotionSequence ? 1e-12 : 0.0);
        case TerminationType::Time:
            return phaseElapsedTime >= termination.value-(explicitMotionSequence ? 1e-12 : 0.0);
        case TerminationType::Distance:
            return phaseTraveledDistance >= termination.value -
                1e-9 * std::max(1.0,termination.value);
        case TerminationType::Speed: {
            double initialSpeed = phaseInitialState.getVelocity().magnitude();
            double currentSpeed = object.getVelocity().magnitude();
            double target = termination.value;
            // Matches the relative tolerance of rounded rectilinear input vectors.
            double tolerance = 1e-8 * std::max({1.0,initialSpeed,target});
            if (std::abs(target - initialSpeed) <= tolerance) {
                return true;
            }
            return target > initialSpeed ? currentSpeed >= target - tolerance :
                currentSpeed <= target + tolerance;
        }
        default:
            return false;
    }
}

// Resumes playback.
void Simulation::play() {
    paused = false;
}

// Pauses playback.
void Simulation::pause() {
    paused = true;
}

// Toggles the playback pause state.
void Simulation::togglePause() {
    paused = !paused;
}

// Reports whether playback is paused.
bool Simulation::isPaused() const {
    return paused;
}

// Returns the estimated kinematics duration from the saved initial state.
double Simulation::getDuration(const Kinematics& kinematics) const {
    (void)kinematics;
    if (!hasInitialState) {
        return 0.0;
    }
    return physicalDuration;
}

// Restores the target body and initial overlap without creating an independent clock.
void Simulation::configureTarget(const TrajectoryTarget& targetCondition) {
    sharedTarget = targetCondition;
    initializeTarget(targetCondition,initialState);
    targetHit = TargetHitResult();
    if (target.enabled && hasInitialState && termination.Type == TerminationType::Target) {
        targetHit = TargetDetector::detectRelative(target,initialState.getPosition(),initialState.getPosition(),
            targetObject.getPosition(),targetObject.getPosition(),initialState.getVelocity(),initialState.getVelocity(),
            targetObject.getVelocity(),targetObject.getVelocity(),0.0,0.0);
        phaseTargetHit = targetHit;
        if (targetHit.hit) pause();
    }
}

// A local target starts from its own JSON state and uses the current primary phase clock.
void Simulation::initializeTarget(const TrajectoryTarget& config,const PhysicalObject& primary) {
    target=config;
    targetKinematics.reset();
    targetMotionFinished=false;
    targetObject=PhysicalObject(target.mass,target.position,target.moving ? target.velocity : Vector2D());
    if (target.moving) {
        targetKinematics.emplace(target.motion);
        targetObject.setAcceleration(targetKinematics->getAcceleration(targetObject));
        if (targetKinematics->isFinished(targetObject)) {
            targetKinematics->finish(targetObject);
            targetMotionFinished=true;
        }
    }
    initialTargetState=targetObject;
    phaseTargetHit = TargetHitResult();
    minimumTargetDistance = target.enabled ? primary.getPosition().distanceTo(targetObject.getPosition()) :
        std::numeric_limits<double>::infinity();
}

// Returns current visual geometry without overwriting the configured initial position.
TrajectoryTarget Simulation::getCurrentTarget() const {
    TrajectoryTarget current=target;
    current.position=targetObject.getPosition();
    return current;
}

// Only camera bounds are requested from target kinematics; no target guides are rendered.
std::vector<Vector2D> Simulation::getTargetPreviewBounds() const {
    // Local targets contribute only their numerically reached bounds during preflight.
    if (phaseLocalTargets) return {};
    return targetKinematics ? targetKinematics->getPreviewBounds(initialTargetState,physicalDuration) : std::vector<Vector2D>();
}

// Exposes the target's numerical state without maintaining its trajectory.
const PhysicalObject& Simulation::getTargetObject() const { return targetObject; }

// Reports whether a natural target boundary has frozen its motion.
bool Simulation::isTargetMotionFinished() const { return targetMotionFinished; }

// Exposes primary path length, including event-localized final steps.
double Simulation::getTraveledDistance() const { return traveledDistance; }
double Simulation::getTraveledAngle() const { return traveledAngle; }
bool Simulation::hasReachedAngularStop() const { return angularStopReached; }

// Exposes only stored numerical positions to graphical representation.
const std::vector<Vector2D>& Simulation::getTrajectoryHistory() const { return trajectoryHistory; }

// Exposes the first accepted target contact.
const TargetHitResult& Simulation::getTargetHit() const { return targetHit; }

// Exposes the minimum segment-to-center error independently of crossing constraints.
double Simulation::getMinimumTargetDistance() const { return minimumTargetDistance; }

// Distinguishes a paused simulation from one that has actually terminated.
bool Simulation::isFinished(const PhysicalObject& object,const Kinematics& kinematics) const {
    (void)kinematics;
    return sequenceComplete || (!explicitMotionSequence && currentKinematics && phaseEnded(object));
}

// Activating a regime only changes acceleration; the live object's mass, P and V survive.
void Simulation::activatePhase(std::size_t index,PhysicalObject& object) {
    const MotionPhase& phase = motionSequence.at(index);
    KinematicsConfig config = phase.kinematics;
    if (phase.accelerationAlongVelocity) {
        if (object.getVelocity().magnitude() <= 1e-9)
            throw std::runtime_error("along_velocity requires a non-zero inherited velocity.");
        config.initialAcceleration = object.getVelocity().normalized()*phase.accelerationValue;
    }
    bool circular = config.phenomenon == "uniform_circular_motion" || config.phenomenon == "uniformly_accelerated_circular_motion";
    if (index > 0 && circular)
        config = Kinematics::inheritCircularConfig(config,object,currentKinematics ? &*currentKinematics : nullptr);
    if (phase.until.Type == TerminationType::Target && !(phaseLocalTargets ? phase.target.has_value() : target.enabled))
        throw std::runtime_error("Phase until Target requires an active target.");
    Kinematics model(config);
    std::string error = model.validate(object,phase.until);
    if (!error.empty()) {
        if (config.phenomenon == "free_fall" && object.getVelocity().magnitude() > 1e-9)
            error += " Use projectile_motion to release an object with inherited velocity.";
        throw std::runtime_error("Phase "+std::to_string(index+1)+": "+error);
    }
    auto method = IntegratorFactory::create(phase.integrator);
    if (!method) throw std::runtime_error("Unable to create the phase integrator.");
    currentIntegrator = std::move(method);
    currentPhaseIndex = index;
    currentKinematics = model;
    termination = phase.until;
    phaseElapsedTime = phaseTraveledDistance = phaseTraveledAngle = 0.0;
    angularStopReached = false;
    phaseInitialState = object;
    phaseExpectedDuration = model.getExpectedDuration(object,termination);
    object.setAcceleration(model.getAcceleration(object));
    // Shared targets retain their physical state; a local target belongs only to this phase.
    if (phaseLocalTargets) {
        TrajectoryTarget local = phase.target.value_or(TrajectoryTarget());
        local.enabled = phase.target.has_value();
        initializeTarget(local,object);
    }
    phaseTargetHit = TargetHitResult();
    if (target.enabled && (termination.Type == TerminationType::Target || phaseLocalTargets)) {
        phaseTargetHit = TargetDetector::detectRelative(target,object.getPosition(),object.getPosition(),
            targetObject.getPosition(),targetObject.getPosition(),object.getVelocity(),object.getVelocity(),
            targetObject.getVelocity(),targetObject.getVelocity(),current_time,current_time);
        if (phaseTargetHit.hit && !targetHit.hit) targetHit = phaseTargetHit;
    }
    currentVisuals = model.getVisuals(object,termination);
    currentProjectileMetrics = model.isProjectile() ? model.getProjectileMetrics(object) : ProjectileMetrics();
    if (explicitMotionSequence) {
        includeCurrentGuides();
        if (target.enabled) includePreviewPoint(targetObject.getPosition(),target.radius);
    }
}

bool Simulation::phaseEnded(const PhysicalObject& object) const {
    return angularStopReached || currentKinematics->isFinished(object) || hasReachedTermination(object);
}

// Every event consumes only its numerical prefix; the following regime receives the remainder.
void Simulation::consumeTime(PhysicalObject& object,const Integrator& integrator,double dt,double stopTime) {
    std::size_t transitions = 0;
    while (!sequenceComplete) {
        if (!phaseEnded(object)) {
            if (dt <= 1e-15 || current_time >= stopTime) break;
            // An injected legacy strategy is retained only when it matches the resolved method.
            const Integrator& method = integrator.getType() == currentIntegrator->getType() ? integrator : *currentIntegrator;
            double used = advance(object,*currentKinematics,method,dt,stopTime);
            dt = std::max(0.0,dt-used);
            if (!phaseEnded(object)) break;
        }
        if (phaseLocalTargets) validateActiveTargetHorizon();
        if (termination.Type == TerminationType::Target && !phaseTargetHit.hit) {
            targetMiss = true;
            sequenceComplete = true;
            pause();
            break;
        }
        if (currentPhaseIndex+1 >= motionSequence.size()) {
            sequenceComplete = true;
            pause();
            break;
        }
        if (++transitions > maximumTransitionsPerStep)
            throw std::runtime_error("Maximum phase transitions per physical step exceeded.");
        activatePhase(currentPhaseIndex+1,object);
    }
}

// Bounds store just their extrema; preview never creates a target trail or future guides.
void Simulation::includePreviewPoint(const Vector2D& point,double radius) {
    if (sequenceBounds.empty()) sequenceBounds = {point,point};
    sequenceBounds[0].x = std::min(sequenceBounds[0].x,point.x-radius);
    sequenceBounds[0].y = std::min(sequenceBounds[0].y,point.y-radius);
    sequenceBounds[1].x = std::max(sequenceBounds[1].x,point.x+radius);
    sequenceBounds[1].y = std::max(sequenceBounds[1].y,point.y+radius);
}

void Simulation::includeCurrentGuides() {
    for (const Scale& scale : currentVisuals.scales) { includePreviewPoint(scale.getStart()); includePreviewPoint(scale.getEnd()); }
    if (currentVisuals.circularGuide.visible) {
        auto guide = currentVisuals.circularGuide;
        includePreviewPoint(guide.center,guide.radius);
    }
}

// Preflight clones state and executes the same stepping and transition path as playback.
void Simulation::runPreflight(const Integrator& integrator) {
    Simulation preview = *this;
    PhysicalObject object = initialState;
    preview.paused = false;
    preview.sequenceBounds.clear();
    preview.includePreviewPoint(object.getPosition());
    preview.includeCurrentGuides();
    if (target.enabled) preview.includePreviewPoint(targetObject.getPosition(),target.radius);
    std::size_t steps = 0;
    while (!preview.sequenceComplete) {
        if (++steps > maximumPreviewSteps)
            throw std::runtime_error("Sequence preflight exceeded the maximum simulated preview steps.");
        double before = preview.current_time;
        preview.consumeTime(object,integrator,physicsStep);
        if (preview.current_time <= before && !preview.sequenceComplete)
            throw std::runtime_error("Sequence preflight made no physical progress.");
    }
    if (preview.targetKinematics && !phaseLocalTargets) {
        TerminationCondition horizon;
        horizon.Type = TerminationType::Target;
        horizon.maxTime = preview.current_time;
        std::string error = preview.targetKinematics->validate(initialTargetState,horizon);
        if (!error.empty()) throw std::runtime_error("Invalid target motion over sequence horizon: "+error);
    }
    physicalDuration = preview.current_time;
    sequenceBounds = preview.sequenceBounds;
    preflightTargetMiss = preview.targetMiss;
    preflightFinalState = MotionState{object.getPosition(),object.getVelocity()};
}

bool Simulation::configureSequence(PhysicalObject& object,const SimulationConfig& config,const Integrator& integrator,std::string& error) {
    try {
        if (!std::isfinite(config.playbackDuration) || config.playbackDuration <= 0.0)
            throw std::runtime_error("playback_duration must be finite and positive.");
        if (config.motionSequence.empty() || config.motionSequence.size() > maximumPhases)
            throw std::runtime_error("Motion sequence must contain between 1 and 64 phases.");
        // Prepare a temporary simulation so failed preflight cannot alter live state.
        Simulation prepared;
        prepared.initialState = object;
        prepared.hasInitialState = true;
        prepared.motionSequence = config.motionSequence;
        prepared.explicitMotionSequence = config.explicitMotionSequence;
        bool hasLocal = std::any_of(config.motionSequence.begin(),config.motionSequence.end(),[](const MotionPhase& phase) { return phase.target.has_value(); });
        if (config.target.enabled && hasLocal)
            throw std::runtime_error("Use either one shared sequence target or phase-local targets, not both.");
        prepared.phaseLocalTargets = hasLocal;
        prepared.termination = config.motionSequence[0].until;
        prepared.configureTarget(config.target);
        PhysicalObject initial = object;
        prepared.activatePhase(0,initial);
        prepared.initialState = initial;
        prepared.trajectoryHistory = {initial.getPosition()};
        if (config.explicitMotionSequence) prepared.runPreflight(integrator);
        else prepared.physicalDuration = prepared.phaseExpectedDuration;
        prepared.timeScale = prepared.physicalDuration > 0.0 ? prepared.physicalDuration/config.playbackDuration : 1.0;
        object.setAcceleration(initial.getAcceleration());
        prepared.paused = false;
        *this = std::move(prepared);
        error.clear();
        return true;
    } catch (const std::exception& exception) { error = exception.what(); return false; }
}

const Kinematics& Simulation::getCurrentKinematics() const { return currentKinematics.value(); }
const KinematicsVisuals& Simulation::getCurrentVisuals() const { return currentVisuals; }
const ProjectileMetrics& Simulation::getCurrentProjectileMetrics() const { return currentProjectileMetrics; }
const std::vector<Vector2D>& Simulation::getSequenceBounds() const { return sequenceBounds; }
std::size_t Simulation::getCurrentPhaseIndex() const { return currentPhaseIndex; }
std::size_t Simulation::getPhaseCount() const { return motionSequence.size(); }
double Simulation::getPhaseElapsedTime() const { return phaseElapsedTime; }
double Simulation::getPhaseTraveledDistance() const { return phaseTraveledDistance; }
double Simulation::getPhaseTraveledAngle() const { return phaseTraveledAngle; }
const TargetHitResult& Simulation::getPhaseTargetHit() const { return phaseTargetHit; }
bool Simulation::isTargetMiss() const { return targetMiss; }
bool Simulation::isSequenceComplete() const { return sequenceComplete; }
bool Simulation::usesMotionSequence() const { return explicitMotionSequence; }

// A local moving target is validated only over the time its own phase actually consumed.
void Simulation::validateActiveTargetHorizon() const {
    if (!targetKinematics) return;
    TerminationCondition horizon;
    horizon.Type = TerminationType::Target;
    horizon.maxTime = phaseElapsedTime;
    std::string error = targetKinematics->validate(initialTargetState,horizon);
    if (!error.empty()) throw std::runtime_error("Invalid phase target motion: "+error);
}

bool Simulation::hasActiveTarget() const { return target.enabled; }
const PhysicalObject* Simulation::getActiveTargetObject() const { return target.enabled ? &targetObject : nullptr; }
const TrajectoryTarget* Simulation::getActiveTargetConfig() const { return target.enabled ? &target : nullptr; }

const TerminationCondition& Simulation::getCurrentUntil() const { return termination; }

const TargetHitResult& Simulation::getActiveTargetHit() const { return phaseLocalTargets ? phaseTargetHit : targetHit; }

bool Simulation::isPreflightTargetMiss() const { return preflightTargetMiss; }

const Integrator& Simulation::getCurrentIntegrator() const { return *currentIntegrator; }
IntegratorType Simulation::getCurrentIntegratorType() const { return currentIntegrator->getType(); }
const std::optional<MotionState>& Simulation::getPreflightFinalState() const { return preflightFinalState; }

const PhysicalObject& Simulation::getPhaseInitialState() const { return phaseInitialState; }
