#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cstddef>
#include <vector>

#include "PhysicalObject.hpp"

class Simulation {
public:
    explicit Simulation(double stepSize);

    void addObject(const PhysicalObject& object);
    void step();
    void run(std::size_t steps);

    std::vector<PhysicalObject>& getObjects();
    const std::vector<PhysicalObject>& getObjects() const;

private:
    double timeStep;
    std::vector<PhysicalObject> objects;
};

#endif
