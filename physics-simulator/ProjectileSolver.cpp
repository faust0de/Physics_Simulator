#include "ProjectileSolver.hpp"
#include "Kinematics.hpp"
#include "SimulationConfig.hpp"
#include <algorithm>
#include <cmath>

// Candidate filtering uses velocity at arrival and the shared analytical impact metrics.
ProjectileSolution ProjectileSolver::validate(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,const Vector2D& velocity,double time) {
    ProjectileSolution result;
    if (gravity <= 0.0 || time <= 0.0 || !std::isfinite(time) ||
        !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || start.y < ground - 1e-9) return result;
    double verticalSpeed = velocity.y - gravity * time;
    const double tolerance = 1e-8;
    if ((target.crossing == TargetCrossing::Ascending && verticalSpeed <= tolerance) ||
        (target.crossing == TargetCrossing::Descending && verticalSpeed >= -tolerance)) return result;
    PhysicalObject launch(1.0,start,velocity);
    SimulationConfig config;
    config.phenomenon = "projectile_motion";
    config.gravityMagnitude = gravity;
    config.groundHeight = ground;
    double impact = Kinematics(config).getProjectileMetrics(launch).impactTime;
    if (time > impact + tolerance * std::max(1.0,impact)) return result;
    result.valid = true;
    result.initialVelocity = velocity;
    result.speed = velocity.magnitude();
    result.angleDegrees = std::atan2(velocity.y,std::abs(velocity.x)) * 180.0 / std::acos(-1.0);
    result.timeToTarget = time;
    result.descendingAtTarget = verticalSpeed < -tolerance;
    return result;
}

// Solves the quadratic in tangent of elevation for both horizontal directions.
std::vector<ProjectileSolution> ProjectileSolver::knownSpeed(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,double speed) {
    std::vector<ProjectileSolution> solutions;
    double dx = target.position.x - start.x;
    double dy = target.position.y - start.y;
    double distance = std::abs(dx);
    if (gravity <= 0.0 || speed <= 0.0 || distance <= 1e-9 || !std::isfinite(speed)) return solutions;
    double squaredSpeed = speed * speed;
    double discriminant = squaredSpeed * squaredSpeed - gravity * (gravity * distance * distance + 2.0 * dy * squaredSpeed);
    double tolerance = 1e-10 * std::max(1.0,squaredSpeed * squaredSpeed);
    if (discriminant < -tolerance) return solutions;
    double root = std::sqrt(std::max(0.0,discriminant));
    for (int branch = 0; branch < 2; ++branch) {
        if (branch == 1 && root <= 1e-12 * std::max(1.0,squaredSpeed)) break;
        double phi = std::atan((squaredSpeed + (branch == 0 ? -root : root)) / (gravity * distance));
        Vector2D velocity((dx > 0.0 ? 1.0 : -1.0) * speed * std::cos(phi),speed * std::sin(phi));
        auto solution = validate(start,target,gravity,ground,velocity,distance / std::abs(velocity.x));
        solution.arc = branch == 0 ? "low" : "high";
        if (solution.valid) solutions.push_back(solution);
    }
    return solutions;
}

// An angle is an elevation toward the target, rather than an absolute world angle.
ProjectileSolution ProjectileSolver::knownAngle(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,double angleDegrees) {
    double dx = target.position.x - start.x;
    double distance = std::abs(dx);
    if (!std::isfinite(angleDegrees) || std::abs(angleDegrees) >= 89.0 || distance <= 1e-9 || gravity <= 0.0) return {};
    double phi = angleDegrees * std::acos(-1.0) / 180.0;
    double cosine = std::cos(phi);
    double denominator = 2.0 * cosine * cosine * (distance * std::tan(phi) - (target.position.y - start.y));
    if (denominator <= 0.0) return {};
    double speed = std::sqrt(gravity * distance * distance / denominator);
    Vector2D velocity((dx > 0.0 ? 1.0 : -1.0) * speed * cosine,speed * std::sin(phi));
    return validate(start,target,gravity,ground,velocity,distance / std::abs(velocity.x));
}

// Rearranges the two independent position equations for a fixed arrival time.
ProjectileSolution ProjectileSolver::knownTime(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,double time) {
    if (time <= 0.0 || !std::isfinite(time) || std::abs(target.position.x-start.x) <= 1e-9) return {};
    Vector2D velocity((target.position.x-start.x)/time,
        (target.position.y-start.y + 0.5*gravity*time*time)/time);
    return validate(start,target,gravity,ground,velocity,time);
}
