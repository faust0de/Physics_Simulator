#include "Simulation.hpp"

Simulation::Simulation(double stepSize) : timeStep(stepSize) {}

void Simulation::addObject(const PhysicalObject& object) {
    objects.push_back(object);
}

void Simulation::step() {
    for (std::size_t i = 0; i < objects.size(); ++i) {
        objects[i].integrate(timeStep);
    }
}

void Simulation::run(std::size_t steps) {
    for (std::size_t i = 0; i < steps; ++i) {
        step();
    }
}

std::vector<PhysicalObject>& Simulation::getObjects() {
    return objects;
}

const std::vector<PhysicalObject>& Simulation::getObjects() const {
    return objects;
}
