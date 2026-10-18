#include "Kinematics.hpp"
#include "SimulationConfig.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Resolves identifiers only during configuration, preserving the external JSON contract.
Kinematics::KinematicsType Kinematics::resolveType(const std::string& name) {
    if (name == "uniform_motion") return KinematicsType::ConstantVelocity;
    if (name == "uniformly_accelerated_motion") return KinematicsType::ConstantAcceleration;
    if (name == "free_fall") return KinematicsType::VerticalFall;
    if (name == "inclined_plane_motion") return KinematicsType::Slope;
    if (name == "projectile_motion") return KinematicsType::Projectile;
    if (name == "uniform_circular_motion") return KinematicsType::UniformCircularMotion;
    if (name == "uniformly_accelerated_circular_motion") return KinematicsType::UniformlyAcceleratedCircularMotion;
    throw std::invalid_argument("Unknown phenomenon.");
}

// Preserves the established primary JSON defaults while sharing the reusable model.
Kinematics::Kinematics(const SimulationConfig& config)
    : Kinematics([&] {
        KinematicsConfig motion = config.circularMotion;
        motion.phenomenon = config.phenomenon;
        motion.initialAcceleration = config.initialAcceleration;
        motion.inclinedPlaneAngleDegrees = config.inclinedPlaneAngleDegrees;
        motion.gravityMagnitude = config.phenomenon == "free_fall" ? static_cast<double>(9.8f) : config.gravityMagnitude;
        motion.groundHeight = config.phenomenon == "free_fall" ? 0.0 : config.groundHeight;
        return motion;
    }()) {}
// Resolves fixed acceleration or reusable circular parameters without evolving state.
Kinematics::Kinematics(const KinematicsConfig& config)
    : type(resolveType(config.phenomenon)),gravityMagnitude(config.gravityMagnitude),groundHeight(config.groundHeight),circularConfig(config) {
    switch (type) {
        case KinematicsType::UniformCircularMotion:
        case KinematicsType::UniformlyAcceleratedCircularMotion: break;
        case KinematicsType::ConstantVelocity: acceleration = Vector2D(); break;
        case KinematicsType::ConstantAcceleration: acceleration = config.initialAcceleration; break;
        case KinematicsType::VerticalFall:
            acceleration = Vector2D(0.0,-gravityMagnitude);
            break;
        case KinematicsType::Projectile: acceleration = Vector2D(0.0,-gravityMagnitude); break;
        case KinematicsType::Slope: {
            double theta = config.inclinedPlaneAngleDegrees * std::acos(-1.0) / 180.0;
            planeDirection = Vector2D(std::cos(theta),std::sin(theta));
            acceleration = planeDirection * Vector2D(0.0,-gravityMagnitude).dot(planeDirection);
            break;
        }
    }
}

// Acceleration is prescribed directly rather than encoded as accumulated force.
Vector2D Kinematics::getAcceleration(const PhysicalObject& object) const {
    return getAcceleration(object.getPosition(),object.getVelocity());
}

// Intermediate RK states use exactly the same physical acceleration law.
Vector2D Kinematics::getAcceleration(const Vector2D& position,const Vector2D& velocity) const {
    if (isCircular()) {
        Vector2D r = position-circularConfig.circularCenter;
        double r2 = r.magnitudeSquared();
        if (!std::isfinite(r2) || r2 <= 1e-18)
            throw std::runtime_error("Invalid circular state: radius is zero or non-finite.");
        // Derive omega from the numerical state; never project radius or velocity.
        double omega = r.cross(velocity)/r2;
        double alpha = type == KinematicsType::UniformlyAcceleratedCircularMotion ? circularConfig.angularAcceleration : 0.0;
        // Centripetal acceleration uses the current radius vector.
        Vector2D centripetal = r*(-omega*omega);
        // The perpendicular already has radius magnitude; do not normalize it.
        Vector2D tangential = Vector2D(-r.y,r.x)*alpha;
        return centripetal+tangential;
    }
    return acceleration;
}

// Only ground-bound scenarios have a natural end.
bool Kinematics::isFinished(const PhysicalObject& object) const {
    if (type == KinematicsType::VerticalFall) return object.getPosition().y <= groundHeight;
    if (type == KinematicsType::Projectile)
        return object.getPosition().y <= groundHeight + 1e-9 && object.getVelocity().y <= 0.0;
    return false;
}

// Preserves numerical position X and both components of impact velocity.
void Kinematics::finish(PhysicalObject& object) const {
    if (type != KinematicsType::VerticalFall && type != KinematicsType::Projectile) return;
    Vector2D position = object.getPosition();
    position.y = groundHeight;
    object.setPosition(position);
    object.clearForces();
}

// A shared scalar frame preserves incline projection and zero-speed conventions.
Kinematics::LinearMotionInfo Kinematics::getLinearMotionInfo(const PhysicalObject& object) const {
    LinearMotionInfo info;
    Vector2D velocity = object.getVelocity();
    double speed = velocity.magnitude();
    const double epsilon = 1e-9;
    if (type == KinematicsType::Slope) {
        double signedVelocity = velocity.dot(planeDirection);
        speed = std::abs(signedVelocity);
        if (speed > epsilon) {
            info.direction = planeDirection * (signedVelocity > 0.0 ? 1.0 : -1.0);
            info.initialSpeed = speed;
            info.accelerationAlongMotion = acceleration.dot(info.direction);
            info.valid = true;
            return info;
        }
    }
    else if (speed > (type == KinematicsType::ConstantVelocity ? 0.0 : epsilon)) {
        info.direction = velocity.normalized();
        info.initialSpeed = speed;
        info.accelerationAlongMotion = acceleration.dot(info.direction);
        info.valid = true;
        return info;
    }
    info.direction = acceleration.normalized();
    info.accelerationAlongMotion = acceleration.magnitude();
    info.valid = info.accelerationAlongMotion > 0.0;
    return info;
}

// Migrates the rationalized quadratic root without altering numerical integration.
double Kinematics::getAcceleratedDuration(const PhysicalObject& object,const TerminationCondition& termination) const {
    LinearMotionInfo info = getLinearMotionInfo(object);
    double a = info.accelerationAlongMotion;
    double duration = 0.0;
    double epsilon = type == KinematicsType::Slope ? 0.0 : 1e-9;
    if (termination.Type == TerminationType::Speed && std::abs(a) > epsilon) {
        duration = (termination.value-info.initialSpeed)/a;
    }
    else if (termination.Type == TerminationType::Distance && termination.value > 0.0) {
        if (std::abs(a) <= epsilon) {
            duration = info.initialSpeed > epsilon ? termination.value/info.initialSpeed : 0.0;
        }
        else {
            double discriminant = info.initialSpeed*info.initialSpeed+2.0*a*termination.value;
            double tolerance = 1e-9*std::max(1.0,info.initialSpeed*info.initialSpeed);
            if (discriminant < -tolerance) return 0.0;
            double denominator = info.initialSpeed+std::sqrt(std::max(0.0,discriminant));
            duration = denominator > 0.0 ? 2.0*termination.value/denominator : 0.0;
        }
    }
    return std::isfinite(duration) && duration > 0.0 ? duration : 0.0;
}

// Retains scenario-specific duration conventions used by playback and timeline.
double Kinematics::getExpectedDuration(const PhysicalObject& object,const TerminationCondition& termination) const {
    if (isCircular()) return getCircularExpectedDuration(termination);
    if (termination.Type == TerminationType::Target) return termination.maxTime;
    if (termination.Type == TerminationType::Time) return termination.value;
    switch (type) {
        case KinematicsType::UniformCircularMotion:
        case KinematicsType::UniformlyAcceleratedCircularMotion: return getCircularExpectedDuration(termination);
        case KinematicsType::ConstantVelocity: {
            double speed = object.getVelocity().magnitude();
            return termination.Type == TerminationType::Distance && speed > 0.0 ? termination.value/speed : 0.0;
        }
        case KinematicsType::ConstantAcceleration:
        case KinematicsType::Slope: return getAcceleratedDuration(object,termination);
        case KinematicsType::VerticalFall: {
            double height = object.getPosition().y-groundHeight;
            return termination.Type == TerminationType::Natural && height > 0.0 ? std::sqrt(2.0*height/gravityMagnitude) : 0.0;
        }
        case KinematicsType::Projectile: return getProjectileMetrics(object).impactTime;
    }
    return 0.0;
}

// Separates launch height, additional rise, and maximum height above ground.
ProjectileMetrics Kinematics::getProjectileMetrics(const PhysicalObject& object) const {
    ProjectileMetrics metrics;
    Vector2D position = object.getPosition();
    Vector2D velocity = object.getVelocity();
    metrics.initialHeightAboveGround = position.y - groundHeight;
    if (velocity.y > 0.0) {
        metrics.riseAboveLaunch = velocity.y * velocity.y / (2.0 * gravityMagnitude);
        metrics.apexTime = velocity.y / gravityMagnitude;
    }
    metrics.maximumHeightAboveGround = metrics.initialHeightAboveGround + metrics.riseAboveLaunch;
    metrics.apexPosition = Vector2D(position.x + velocity.x * metrics.apexTime,
        position.y + metrics.riseAboveLaunch);
    double root = std::sqrt(std::max(0.0,velocity.y * velocity.y +
        2.0 * gravityMagnitude * metrics.initialHeightAboveGround));
    // Rationalization avoids cancellation for a downward launch from height.
    metrics.impactTime = velocity.y < 0.0 ?
        (root - velocity.y > 0.0 ? 2.0 * metrics.initialHeightAboveGround / (root - velocity.y) : 0.0) :
        (velocity.y + root) / gravityMagnitude;
    metrics.impactPosition = Vector2D(position.x + velocity.x * metrics.impactTime,groundHeight);
    metrics.horizontalRange = std::abs(metrics.impactPosition.x - position.x);
    return metrics;
}

// Identifies the applicability of projectile-specific analytical reference data.
bool Kinematics::isProjectile() const { return type == KinematicsType::Projectile; }

// Describes positive traveled magnitudes while preserving geometric orientation.
std::vector<Scale> Kinematics::buildLinearScales(const Vector2D& start,const Vector2D& end,double pathDistance) const {
    const double epsilon = 1e-9;
    double dx = end.x-start.x;
    double dy = end.y-start.y;
    if (std::abs(dx) <= epsilon && std::abs(dy) <= epsilon) return {};
    if (std::abs(dy) <= epsilon)
        return {Scale(start,end,0.0,std::abs(dx),5,"m","Δx",ScaleRole::Horizontal)};
    if (std::abs(dx) <= epsilon)
        return {Scale(start,end,0.0,std::abs(dy),5,"m","Δy",ScaleRole::Vertical)};
    Vector2D projection(end.x,start.y);
    return {
        Scale(start,end,0.0,pathDistance,5,"m","s",ScaleRole::Path),
        Scale(start,projection,0.0,std::abs(dx),5,"m","Δx",ScaleRole::Horizontal),
        Scale(projection,end,0.0,std::abs(dy),5,"m","Δy",ScaleRole::Vertical)
    };
}

// Uses launch, relevant apex, and endpoint references without drawing a false path chord.
std::vector<Scale> Kinematics::buildProjectileScales(const PhysicalObject& object,const TerminationCondition& termination) const {
    ProjectileMetrics metrics = getProjectileMetrics(object);
    double duration = termination.Type == TerminationType::Time ? std::min(termination.value,metrics.impactTime) : metrics.impactTime;
    Vector2D start = object.getPosition();
    Vector2D velocity = object.getVelocity();
    double endX = start.x+velocity.x*duration;
    double peakTime = std::min(duration,metrics.apexTime);
    double top = start.y+velocity.y*peakTime-0.5*gravityMagnitude*peakTime*peakTime;
    double range = std::abs(endX-start.x);
    double rulerX = start.x-std::max(range*0.08,0.5);
    std::vector<Scale> scales;
    if (range > 1e-9) scales.emplace_back(Vector2D(start.x,groundHeight),Vector2D(endX,groundHeight),0.0,range,5,"m","Δx",ScaleRole::Horizontal);
    if (top-groundHeight > 1e-9) scales.emplace_back(Vector2D(rulerX,groundHeight),Vector2D(rulerX,top),0.0,top-groundHeight,5,"m","h",ScaleRole::Vertical);
    return scales;
}

// Anchors the acute geometric slope angle at the lower point for either motion direction.
AngleGuide Kinematics::buildInclinedAngleGuide(const Vector2D& start,const Vector2D& end) const {
    AngleGuide guide;
    if (start.distanceTo(end) <= 1e-9) return guide;
    guide.visible = true;
    guide.vertex = start.y < end.y ? start : end;
    Vector2D upper = start.y < end.y ? end : start;
    guide.direction = (upper-guide.vertex).normalized();
    guide.referenceDirection = Vector2D(guide.direction.x >= 0.0 ? 1.0 : -1.0,0.0);
    guide.angleDegrees = std::atan2(std::abs(guide.direction.y),std::abs(guide.direction.x))*180.0/std::acos(-1.0);
    return guide;
}

// Builds measurement guides from analytical references, never from corrected state.
KinematicsVisuals Kinematics::getVisuals(const PhysicalObject& object,const TerminationCondition& termination) const {
    if (isCircular()) {
        KinematicsVisuals visuals;
        visuals.circularGuide.visible = true;
        visuals.circularGuide.center = circularConfig.circularCenter;
        visuals.circularGuide.radius = circularConfig.circularRadius;
        return visuals;
    }
    if (termination.Type == TerminationType::Target) {
        TerminationCondition preview;
        preview.Type = TerminationType::Time;
        preview.value = termination.maxTime;
        if (type == KinematicsType::VerticalFall || type == KinematicsType::Projectile) {
            TerminationCondition natural;
            preview.value = std::min(preview.value,getExpectedDuration(object,natural));
        }
        return getVisuals(object,preview);
    }
    KinematicsVisuals visuals;
    if (type == KinematicsType::Projectile) {
        visuals.scales = buildProjectileScales(object,termination);
        return visuals;
    }
    Vector2D start = object.getPosition();
    double duration = getExpectedDuration(object,termination);
    Vector2D end;
    double distance;
    if (type == KinematicsType::VerticalFall) {
        end = termination.Type == TerminationType::Natural ? Vector2D(start.x,groundHeight) :
            start+object.getVelocity()*duration+acceleration*(0.5*duration*duration);
        distance = start.distanceTo(end);
    }
    else {
        LinearMotionInfo info = getLinearMotionInfo(object);
        if (!info.valid || duration <= 0.0) return visuals;
        distance = termination.Type == TerminationType::Distance ? termination.value :
            info.initialSpeed*duration+0.5*info.accelerationAlongMotion*duration*duration;
        end = start+info.direction*distance;
    }
    if (!std::isfinite(distance) || distance <= 0.0) return visuals;
    visuals.scales = buildLinearScales(start,end,distance);
    if (type == KinematicsType::Slope) visuals.angleGuide = buildInclinedAngleGuide(start,end);
    return visuals;
}

// Checks the same scalar reachability limits used by both accelerated scenarios.
std::string Kinematics::validateLinearMotion(const PhysicalObject& object,const TerminationCondition& termination) const {
    const double epsilon = 1e-9;
    LinearMotionInfo info = getLinearMotionInfo(object);
    double initialSpeed = info.initialSpeed;
    double aParallel = info.accelerationAlongMotion;
    double value = termination.value;
    if (aParallel < 0.0) {
        double stopTime = initialSpeed/(-aParallel);
        double stopDistance = initialSpeed*initialSpeed/(2.0*(-aParallel));
        if (termination.Type == TerminationType::Time && value > stopTime+epsilon*std::max(1.0,stopTime))
            return "Time termination would cause a direction reversal, which is not supported yet.";
        if (termination.Type == TerminationType::Distance && value > stopDistance+epsilon*std::max(1.0,stopDistance))
            return "Distance termination would require a direction reversal, which is not supported yet.";
    }
    if (termination.Type == TerminationType::Speed) {
        if (aParallel > 0.0 && value <= initialSpeed)
            return "Target speed must be greater than the initial speed for this accelerating motion.";
        if (aParallel < 0.0 && value >= initialSpeed)
            return "Target speed must be lower than the initial speed for this decelerating motion.";
    }
    return {};
}

// Centralizes scenario semantics while JSON shape and exclusivity stay in the parser.
std::string Kinematics::validate(const PhysicalObject& object,const TerminationCondition& termination) const {
    if (isCircular()) return validateCircularMotion(object,termination);
    if (termination.Type == TerminationType::Angle) return "Angle termination requires circular motion.";
    if (termination.Type == TerminationType::Target) {
        TerminationCondition horizon;
        horizon.Type = (type == KinematicsType::VerticalFall || type == KinematicsType::Projectile) ?
            TerminationType::Natural : TerminationType::Time;
        horizon.value = termination.maxTime;
        return validate(object,horizon);
    }
    double speed = object.getVelocity().magnitude();
    const double epsilon = 1e-9;
    switch (type) {
        case KinematicsType::UniformCircularMotion:
        case KinematicsType::UniformlyAcceleratedCircularMotion: return validateCircularMotion(object,termination);
        case KinematicsType::VerticalFall:
            if (object.getPosition().y <= groundHeight) return "object.position.y must be greater than 0 for free_fall.";
            if (object.getVelocity().x != 0.0) return "free_fall currently requires initial velocity.x = 0.";
            if (object.getVelocity().y != 0.0) return "free_fall currently requires initial velocity.y = 0.";
            if (termination.Type != TerminationType::Natural && termination.Type != TerminationType::Time)
                return "Termination type is not currently supported for free_fall.";
            return {};
        case KinematicsType::ConstantVelocity:
            if (termination.Type == TerminationType::Natural) return "uniform_motion requires a time or distance termination.";
            if (termination.Type == TerminationType::Speed) return "Speed termination is not supported for uniform_motion.";
            if (termination.Type == TerminationType::Distance && speed <= 0.0)
                return "uniform_motion requires a non-zero velocity for distance termination.";
            return {};
        case KinematicsType::ConstantAcceleration:
            if (acceleration.magnitude() <= epsilon) return "uniformly_accelerated_motion requires a non-zero acceleration.";
            if (speed > epsilon && std::abs(object.getVelocity().cross(acceleration)) > epsilon*speed*acceleration.magnitude())
                return "uniformly_accelerated_motion requires parallel velocity and acceleration.";
            if (termination.Type == TerminationType::Natural)
                return "uniformly_accelerated_motion requires a time, distance, or speed termination.";
            return validateLinearMotion(object,termination);
        case KinematicsType::Slope:
            if (speed > epsilon && std::abs(object.getVelocity().cross(planeDirection)) > 1e-8*speed)
                return "object.velocity must be parallel to the inclined plane.";
            if (termination.Type == TerminationType::Natural)
                return "inclined_plane_motion requires a time, distance, or speed termination.";
            return validateLinearMotion(object,termination);
        case KinematicsType::Projectile:
            if (object.getPosition().y < groundHeight-1e-9) return "Initial position must be at or above ground_height.";
            if (!std::isfinite(getProjectileMetrics(object).impactTime) || getProjectileMetrics(object).impactTime <= 0.0)
                return "Projectile launch must have a positive flight time.";
            if (termination.Type != TerminationType::Natural && termination.Type != TerminationType::Time)
                return "projectile_motion supports only natural or time termination.";
            if (termination.Type == TerminationType::Time && termination.value >
                getProjectileMetrics(object).impactTime+1e-8*std::max(1.0,getProjectileMetrics(object).impactTime))
                return "Time termination exceeds projectile impact time.";
            return {};
    }
    return {};
}


// Linear interpolation locates the ground event within the integrator's numerical segment.
double Kinematics::getNaturalBoundaryFraction(const PhysicalObject& previous,const PhysicalObject& current) const {
    if (!isFinished(current)) return 1.0;
    if (isFinished(previous)) return 0.0;
    double deltaY = current.getPosition().y-previous.getPosition().y;
    if (deltaY >= 0.0) return 1.0;
    return std::clamp((groundHeight-previous.getPosition().y)/deltaY,0.0,1.0);
}

// Analytical preview points affect only camera bounds, never the integrated states.
std::vector<Vector2D> Kinematics::getPreviewBounds(const PhysicalObject& object,double horizon) const {
    if (isCircular()) {
        Vector2D r(circularConfig.circularRadius,circularConfig.circularRadius);
        return {circularConfig.circularCenter-r,circularConfig.circularCenter+r};
    }
    Vector2D start = object.getPosition();
    Vector2D velocity = object.getVelocity();
    double duration = std::max(0.0,horizon);
    bool hasGround = type == KinematicsType::VerticalFall || type == KinematicsType::Projectile;
    bool endsNaturally = false;
    if (hasGround) {
        TerminationCondition natural;
        double impact = getExpectedDuration(object,natural);
        endsNaturally = duration >= impact;
        duration = std::min(duration,std::max(0.0,impact));
    }
    auto positionAt = [&](double time) { return start+velocity*time+acceleration*(0.5*time*time); };
    Vector2D end = positionAt(duration);
    if (endsNaturally) end.y = groundHeight;
    std::vector<Vector2D> points{start,end};
    for (int axis=0;axis<2;++axis) {
        double component = axis == 0 ? acceleration.x : acceleration.y;
        double initial = axis == 0 ? velocity.x : velocity.y;
        if (component != 0.0) {
            double turningTime = -initial/component;
            if (turningTime > 0.0 && turningTime < duration) points.push_back(positionAt(turningTime));
        }
    }
    return points;
}

// Configuration angles are degrees; angular velocity and acceleration use radians.
bool Kinematics::isCircular() const {
    return type == KinematicsType::UniformCircularMotion || type == KinematicsType::UniformlyAcceleratedCircularMotion;
}

PhysicalObject Kinematics::calculateCircularInitialState(double mass) const {
    double theta = std::remainder(circularConfig.initialAngleDegrees,360.0)*(std::acos(-1.0)/180.0);
    Vector2D r(std::cos(theta)*circularConfig.circularRadius,std::sin(theta)*circularConfig.circularRadius);
    PhysicalObject object(mass,circularConfig.circularCenter+r,Vector2D(-r.y,r.x)*circularConfig.initialAngularVelocity);
    object.setAcceleration(getAcceleration(object));
    return object;
}

CircularMetrics Kinematics::getCircularMetrics(const PhysicalObject& object) const {
    CircularMetrics metrics;
    if (!isCircular()) return metrics;
    Vector2D r = object.getPosition()-circularConfig.circularCenter;
    metrics.configuredRadius = circularConfig.circularRadius;
    metrics.currentRadius = r.magnitude();
    // Signed radial error intentionally exposes the integrator's drift.
    metrics.radialError = metrics.currentRadius-metrics.configuredRadius;
    metrics.angleRadians = std::atan2(r.y,r.x);
    metrics.angleDegrees = metrics.angleRadians*180.0/std::acos(-1.0);
    metrics.angularVelocity = r.magnitudeSquared() > 1e-18 ? r.cross(object.getVelocity())/r.magnitudeSquared() : 0.0;
    metrics.angularAcceleration = type == KinematicsType::UniformlyAcceleratedCircularMotion ? circularConfig.angularAcceleration : 0.0;
    metrics.tangentialSpeed = std::abs(metrics.angularVelocity)*metrics.currentRadius;
    metrics.centripetalAcceleration = metrics.angularVelocity*metrics.angularVelocity*metrics.currentRadius;
    metrics.tangentialAcceleration = std::abs(metrics.angularAcceleration)*metrics.currentRadius;
    return metrics;
}

// Relative atan2 measures each numerical segment across the +/-pi branch cut.
double Kinematics::getAngularIncrement(const Vector2D& previous,const Vector2D& current) const {
    if (!isCircular()) return 0.0;
    Vector2D r0 = previous-circularConfig.circularCenter,r1 = current-circularConfig.circularCenter;
    return std::abs(std::atan2(r0.cross(r1),r0.dot(r1)));
}

bool Kinematics::hasAngularReversal(const PhysicalObject& object) const {
    return type == KinematicsType::UniformlyAcceleratedCircularMotion &&
        std::abs(circularConfig.initialAngularVelocity) > 1e-9 &&
        circularConfig.initialAngularVelocity*circularConfig.angularAcceleration < 0.0 &&
        getCircularMetrics(object).angularVelocity*circularConfig.initialAngularVelocity <= 0.0;
}

// These ideal scalar equations only predict playback duration and reachability.
double Kinematics::getCircularExpectedDuration(const TerminationCondition& termination) const {
    if (termination.Type == TerminationType::Target) return termination.maxTime;
    if (termination.Type == TerminationType::Time) return termination.value;
    AngularMotionInfo info = getAngularMotionInfo();
    double w = info.initialAngularSpeed,beta = info.accelerationAlongMotion;
    if (termination.Type == TerminationType::Speed)
        return beta != 0.0 ? (termination.value/circularConfig.circularRadius-w)/beta : 0.0;
    double angle = termination.Type == TerminationType::Angle ? termination.value*(std::acos(-1.0)/180.0) :
        termination.Type == TerminationType::Distance ? termination.value/circularConfig.circularRadius : 0.0;
    if (std::abs(beta) <= 1e-18) return w > 0.0 ? angle/w : 0.0;
    double denominator = w+std::sqrt(std::max(0.0,w*w+2.0*beta*angle));
    return denominator > 0.0 ? 2.0*angle/denominator : 0.0;
}

std::string Kinematics::validateCircularMotion(const PhysicalObject& object,const TerminationCondition& termination) const {
    const auto& c = circularConfig;
    if (!std::isfinite(c.circularRadius) || c.circularRadius <= 0.0) return "circular_motion.radius must be greater than 0.";
    if (!std::isfinite(c.initialAngularVelocity) || !std::isfinite(c.angularAcceleration) ||
        !std::isfinite(c.initialAngleDegrees) || !std::isfinite(c.circularCenter.x) || !std::isfinite(c.circularCenter.y))
        return "Circular parameters must be finite.";
    if (type == KinematicsType::UniformCircularMotion && std::abs(c.initialAngularVelocity) <= 1e-9)
        return "MCU requires non-zero angular_velocity.";
    if (type == KinematicsType::UniformlyAcceleratedCircularMotion && std::abs(c.angularAcceleration) <= 1e-9)
        return "MCUA requires non-zero angular_acceleration.";
    if (termination.Type == TerminationType::Natural) return "Circular motion requires an explicit termination.";
    if (type == KinematicsType::UniformCircularMotion && termination.Type == TerminationType::Speed)
        return "Speed termination is not supported for MCU.";
    AngularMotionInfo info = getAngularMotionInfo();
    double w = info.initialAngularSpeed,beta = info.accelerationAlongMotion;
    if (type == KinematicsType::UniformCircularMotion) return {};
    double initialLinearSpeed = object.getVelocity().magnitude();
    if (termination.Type == TerminationType::Speed &&
        ((beta > 0.0 && termination.value <= initialLinearSpeed) ||
         (beta < 0.0 && termination.value >= initialLinearSpeed)))
        return "Target speed is unreachable before angular direction reversal.";
    if (beta < 0.0) {
        double stopTime = w/(-beta),stopAngle = w*w/(-2.0*beta);
        double requestedTime = termination.Type == TerminationType::Target ? termination.maxTime : termination.value;
        bool beyondTime = (termination.Type == TerminationType::Time || termination.Type == TerminationType::Target) &&
            requestedTime > stopTime+1e-9*std::max(1.0,stopTime);
        double requestedAngle = termination.Type == TerminationType::Distance ? termination.value/c.circularRadius :
            termination.value*(std::acos(-1.0)/180.0);
        bool beyondAngle = (termination.Type == TerminationType::Angle || termination.Type == TerminationType::Distance) &&
            requestedAngle > stopAngle+1e-9*std::max(1.0,stopAngle);
        if (beyondTime || beyondAngle) return "Circular termination would require angular direction reversal, which is not supported yet.";
    }
    return {};
}

// The initial direction follows omega, or alpha when starting from rest.
Kinematics::AngularMotionInfo Kinematics::getAngularMotionInfo() const {
    AngularMotionInfo info;
    info.initialAngularSpeed = std::abs(circularConfig.initialAngularVelocity);
    double sign = info.initialAngularSpeed > 1e-9 ?
        (circularConfig.initialAngularVelocity > 0.0 ? 1.0 : -1.0) :
        (circularConfig.angularAcceleration > 0.0 ? 1.0 : -1.0);
    info.accelerationAlongMotion = type == KinematicsType::UniformCircularMotion ? 0.0 :
        circularConfig.angularAcceleration*sign;
    return info;
}

KinematicsConfig Kinematics::getConfiguration() const { return circularConfig; }

// A later circular phase inherits the numerical state, including Euler radial drift.
KinematicsConfig Kinematics::inheritCircularConfig(KinematicsConfig config,const PhysicalObject& object,const Kinematics* previous) {
    Vector2D r = object.getPosition()-config.circularCenter;
    double radius = r.magnitude(),speed = object.getVelocity().magnitude();
    if (!std::isfinite(radius) || radius <= 1e-9) throw std::runtime_error("Inherited circular radius must be non-zero.");
    bool continuation = previous && previous->isCircular() &&
        previous->circularConfig.circularCenter.distanceTo(config.circularCenter) <=
            1e-9*std::max({1.0,previous->circularConfig.circularRadius,radius});
    if (!continuation && speed > 1e-9 && std::abs(r.dot(object.getVelocity())) > 1e-8*radius*speed)
        throw std::runtime_error("Inherited velocity is not tangent to the requested circular motion.");
    config.circularRadius = continuation ? previous->circularConfig.circularRadius : radius;
    config.initialAngleDegrees = std::atan2(r.y,r.x)*180.0/std::acos(-1.0);
    config.initialAngularVelocity = r.cross(object.getVelocity())/r.magnitudeSquared();
    return config;
}
