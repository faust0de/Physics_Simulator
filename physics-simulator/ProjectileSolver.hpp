#pragma once
#include "TrajectoryTarget.hpp"
#include <string>
#include <vector>

// Stores a launch solution aimed analytically at a target center.
struct ProjectileSolution {
    // Indicates that a physical solution passed ground and crossing checks.
    bool valid = false;
    // Stores the computed launch velocity.
    Vector2D initialVelocity;
    // Stores launch speed in meters per second.
    double speed = 0.0;
    // Stores elevation relative to the horizontal direction toward the target.
    double angleDegrees = 0.0;
    // Stores analytical arrival time at the target center.
    double timeToTarget = 0.0;
    // Reports whether arrival occurs on the descending branch.
    bool descendingAtTarget = false;
    // Identifies the low or high branch for known-speed solutions.
    std::string arc;
};

// Solves initial conditions without evolving a simulation or altering numerical states.
class ProjectileSolver {
private:
    // Filters a candidate using target crossing and ground impact time.
    static ProjectileSolution validate(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,const Vector2D& velocity,double time);
public:
    // Returns every valid low/high branch for a fixed launch speed.
    static std::vector<ProjectileSolution> knownSpeed(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,double speed);
    // Solves speed for an elevation angle toward the target's horizontal direction.
    static ProjectileSolution knownAngle(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,double angleDegrees);
    // Solves the velocity required to arrive at a specified time.
    static ProjectileSolution knownTime(const Vector2D& start,const TrajectoryTarget& target,double gravity,double ground,double time);
};
