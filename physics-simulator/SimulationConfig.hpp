#pragma once

#include <string>

#include "Vector2D.hpp"

// Stores the validated configuration required to build a simulation.
struct SimulationConfig {

    // Identifies the physical phenomenon to simulate.
    std::string phenomenon;

    // Stores the mass of the simulated object in kilograms.
    double mass = 0.0;

    // Stores the initial position of the simulated object in meters.
    Vector2D initialPosition;

    // Stores the initial velocity of the simulated object in meters per second.
    Vector2D initialVelocity;

    // Identifies the numerical integration method.
    std::string integrator;

    // Defines how many real seconds the complete simulation should take to play.
    double playbackDuration = 0.0;
};