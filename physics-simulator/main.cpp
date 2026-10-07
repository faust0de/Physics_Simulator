#include <iostream>
#include <iomanip>

#include "PhysicalObject.hpp"
#include "FreeFall.hpp"
#include "EulerIntegrator.hpp"

int main() {

    // Ball of 2 kg, initially at 10 m, at rest.
    PhysicalObject ball(
        2.0,
        Vector2D(0.0, 10.0),
        Vector2D(0.0, 0.0)
    );

    FreeFall freeFall;
    EulerIntegrator integrator;

    const double deltaTime = 0.1;
    double time = 0.0;

    std::cout << std::fixed << std::setprecision(3);

    // Initial state
    Vector2D position = ball.getPosition();
    Vector2D velocity = ball.getVelocity();

    std::cout
        << "t = " << time
        << " | position = (" << position.x << ", " << position.y << ")"
        << " | velocity = (" << velocity.x << ", " << velocity.y << ")"
        << '\n';

    // Simulate 1 second
    for (int i = 0; i < 10; i++) {

        // Apply the physical phenomenon
        freeFall.apply(ball, deltaTime);

        // Advance the state using Euler
        integrator.integrate(ball, deltaTime);

        time += deltaTime;

        position = ball.getPosition();
        velocity = ball.getVelocity();

        std::cout
            << "t = " << time
            << " | position = (" << position.x << ", " << position.y << ")"
            << " | velocity = (" << velocity.x << ", " << velocity.y << ")"
            << '\n';
    }

    return 0;
}