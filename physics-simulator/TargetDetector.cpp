#include "TargetDetector.hpp"
#include <algorithm>
#include <cmath>

// Orthogonal projection is clamped to the finite segment, including stationary steps.
double TargetDetector::minimumDistance(const Vector2D& start,const Vector2D& end,const Vector2D& center) {
    Vector2D delta = end-start;
    double lengthSquared = delta.dot(delta);
    double u = lengthSquared > 0.0 ? std::clamp((center-start).dot(delta)/lengthSquared,0.0,1.0) : 0.0;
    return (start+delta*u).distanceTo(center);
}

// Circle entry and exit bound the interval in which crossing can be accepted.
TargetHitResult TargetDetector::detect(const TrajectoryTarget& target,const Vector2D& start,const Vector2D& end,const Vector2D& startVelocity,const Vector2D& endVelocity,double startTime,double endTime) {
    return detectRelative(target,start,end,target.position,target.position,startVelocity,endVelocity,Vector2D(),Vector2D(),startTime,endTime);
}

// Relative position reduces two moving bodies to one point and a fixed circle.
TargetHitResult TargetDetector::detectRelative(const TrajectoryTarget& target,const Vector2D& start,const Vector2D& end,const Vector2D& targetStart,const Vector2D& targetEnd,const Vector2D& startVelocity,const Vector2D& endVelocity,const Vector2D& targetStartVelocity,const Vector2D& targetEndVelocity,double startTime,double endTime) {
    TargetHitResult result;
    if (!target.enabled || target.radius <= 0.0) return result;
    Vector2D delta = (end-targetEnd)-(start-targetStart);
    Vector2D offset = start-targetStart;
    double a = delta.dot(delta);
    double lo = 0.0;
    double hi = 1.0;
    if (a <= 1e-24) {
        if (offset.magnitude() > target.radius+1e-12*std::max(1.0,target.radius)) return result;
    }
    else {
        // Closest-point roots avoid subtracting huge nearly equal discriminant terms.
        double center = -offset.dot(delta)/a;
        Vector2D perpendicular = offset+delta*center;
        double perpendicularSquared = perpendicular.dot(perpendicular);
        double gap = target.radius*target.radius-perpendicularSquared;
        double tolerance = 1e-12*std::max({1.0,target.radius*target.radius,perpendicularSquared});
        if (gap < -tolerance) return result;
        double halfWidth = std::sqrt(std::max(0.0,gap)/a);
        lo = std::max(0.0,center-halfWidth);
        hi = std::min(1.0,center+halfWidth);
        if (lo > hi) return result;
    }
    if (target.crossing != TargetCrossing::Any) {
        double sign = target.crossing == TargetCrossing::Ascending ? 1.0 : -1.0;
        double initial = sign*startVelocity.y;
        double slope = sign*(endVelocity.y-startVelocity.y);
        const double threshold = 1e-8;
        if (std::abs(slope) <= 1e-20) {
            if (initial <= threshold) return result;
        }
        else {
            double boundary = (threshold-initial)/slope;
            if (slope > 0.0) lo = std::max(lo,boundary+1e-12);
            else hi = std::min(hi,boundary-1e-12);
            if (lo > hi || hi < 0.0 || lo > 1.0) return result;
        }
    }
    double u = std::clamp(lo,0.0,1.0);
    result.hit = true;
    result.fraction = u;
    result.position = start+(end-start)*u;
    result.targetPosition = targetStart+(targetEnd-targetStart)*u;
    result.targetVelocity = targetStartVelocity+(targetEndVelocity-targetStartVelocity)*u;
    result.velocity = startVelocity+(endVelocity-startVelocity)*u;
    result.time = startTime+(endTime-startTime)*u;
    result.distanceFromCenter = result.position.distanceTo(result.targetPosition);
    return result;
}
