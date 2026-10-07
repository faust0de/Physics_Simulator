#pragma once
#include "PhysicalObject.hpp"
class PhysicalPhenomenon{
public:
    virtual ~PhysicalPhenomenon()=default; // Virtual destructor for derived classes

    virtual void apply(PhysicalObject& object, double deltatime)const=0; // Must be implemented by derived phenomena.
};
