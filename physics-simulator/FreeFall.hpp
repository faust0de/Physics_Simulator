#include "PhysicalPhenomenon.hpp"

class FreeFall:public PhysicalPhenomenon{
private:
    const float gravity=-9.8;
public:
    void apply(PhysicalObject& object, double deltatime)const override; // Applies the gravitational force to the object.
};
