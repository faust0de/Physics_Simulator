#pragma once
#include "PhysicalObject.hpp"
#include "Scale.hpp"
// Defines the abstract interface for physical behavior with optional default hooks.
class PhysicalPhenomenon{
public:
    // Virtual destructor for derived classes
    virtual ~PhysicalPhenomenon()=default;

    // Must be implemented by derived phenomena.
    // Applies the physical phenomenon to the object for one physics step.
    virtual void apply(PhysicalObject& object, double deltatime)const=0;

    // Defaults to an unfinished phenomenon unless a derived class provides a condition.
    virtual bool isFinished(const PhysicalObject& object) const{
        return false;
    }

    // Provides an optional finalization hook that does nothing by default.
    virtual void finish(PhysicalObject& object) const{    
    }
    // Provides optional measurement scales, returning none by default.
    virtual std::vector<Scale> getScales(const PhysicalObject& object) const {
        return {};
    }

    // Provides an optional duration estimate, returning zero by default.
    virtual double getExpectedDuration(const PhysicalObject& object) const {
        return 0.0;
    }

    // Provides the phenomenon acceleration, returning a zero vector by default.
    virtual Vector2D getAcceleration(const PhysicalObject& object) const {
        return Vector2D();
    }
};
