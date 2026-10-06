#include <iostream>

#include "Simulation.hpp"

int main() {
    Simulation simulation(0.1);
    simulation.addObject(PhysicalObject(1.0, Vector2D(0.0, 0.0), Vector2D(1.0, 0.0)));

    for (int step = 0; step < 5; ++step) {
        simulation.getObjects()[0].applyForce(Vector2D(0.0, -9.8));
        simulation.step();

        const Vector2D& position = simulation.getObjects()[0].getPosition();
        std::cout << "Step " << step + 1 << ": (" << position.x << ", " << position.y << ")\n";
    }

    return 0;
}
