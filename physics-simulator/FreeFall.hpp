#pragma once

#include "PhysicalPhenomenon.hpp"

// Models vertical motion under constant gravity.
class FreeFall:public PhysicalPhenomenon{
private:
    // Stores the constant vertical gravitational acceleration in meters per second squared.
    const float gravity=-9.8;
    // Stores the ground position in world meters.
    const double groundHeight = 0.0;
public:
    // Applies the gravitational force to the object.
    void apply(PhysicalObject& object, double deltatime)const override;

    // Checks whether the object has reached or passed ground level.
    bool isFinished(const PhysicalObject& object) const override;

    // Places the object exactly at ground level while preserving impact velocity.
    void finish(PhysicalObject& object) const override;

    // Describes a height scale between ground level and the supplied object position.
    std::vector<Scale> getScales(const PhysicalObject& object) const override;

    // Estimates the fall duration from the supplied height assuming release from rest.
    double getExpectedDuration(const PhysicalObject& object) const override;

    // Returns the constant vertical gravitational acceleration.
    Vector2D getAcceleration(const PhysicalObject& object) const override;
};
