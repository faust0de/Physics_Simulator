#pragma once
#include "TrajectoryTarget.hpp"

// Records the first accepted numerical contact with a circular target.
struct TargetHitResult {
    // Reports whether a crossing-qualified contact has occurred.
    bool hit = false;
    // Stores the contact fraction along the supplied numerical segment.
    double fraction = 0.0;
    // Stores interpolated physical contact time.
    double time = 0.0;
    // Stores the numerical segment contact position.
    Vector2D position;
    // Stores interpolated velocity at contact.
    Vector2D velocity;
    // Stores the contact distance to the target center.
    double distanceFromCenter = 0.0;
    // Stores the target position at numerical contact.
    Vector2D targetPosition;
    // Stores interpolated target velocity at numerical contact.
    Vector2D targetVelocity;
};

// Observes numerical trajectory segments without modifying physical state.
class TargetDetector {
public:
    // Finds first contact in relative space while testing the primary vertical velocity.
    static TargetHitResult detectRelative(const TrajectoryTarget& target,const Vector2D& start,const Vector2D& end,const Vector2D& targetStart,const Vector2D& targetEnd,const Vector2D& startVelocity,const Vector2D& endVelocity,const Vector2D& targetStartVelocity,const Vector2D& targetEndVelocity,double startTime,double endTime);

    // Measures closest center distance along a complete numerical segment.
    static double minimumDistance(const Vector2D& start,const Vector2D& end,const Vector2D& center);
    // Intersects a segment with the circle and the allowed velocity direction interval.
    static TargetHitResult detect(const TrajectoryTarget& target,const Vector2D& start,const Vector2D& end,const Vector2D& startVelocity,const Vector2D& endVelocity,double startTime,double endTime);
};
