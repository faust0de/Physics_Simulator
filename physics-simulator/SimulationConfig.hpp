#pragma once

#include <string>
#include <vector>
#include <cstddef>
#include <optional>


#include "Vector2D.hpp"
#include "TerminationCondition.hpp"
#include "TrajectoryTarget.hpp"
#include "Kinematics.hpp"
#include "ProjectileSolver.hpp"
#include "Integrator.hpp"

inline constexpr std::size_t maximumMotionPhases = 64;

// Stores a regime and its local exit condition without owning a physical object.
struct MotionPhase {
    KinematicsConfig kinematics;
    TerminationCondition until;
    bool accelerationAlongVelocity = false;
    double accelerationValue = 0.0;
    std::optional<TrajectoryTarget> target;
    IntegratorType integrator = IntegratorType::Euler;
};

// Stores the validated configuration required to build a simulation.
struct SimulationConfig {

    // Identifies the physical phenomenon to simulate.
    std::string phenomenon;
    std::vector<MotionPhase> motionSequence;
    bool explicitMotionSequence = false;

    // Stores the mass of the simulated object in kilograms.
    double mass = 0.0;

    // Stores the initial position of the simulated object in meters.
    Vector2D initialPosition;

    // Stores the initial velocity of the simulated object in meters per second.
    Vector2D initialVelocity;

    // Stores the prescribed initial acceleration in meters per second squared.
    Vector2D initialAcceleration;

    // Stores the inclined plane angle in degrees from the positive X axis.
    double inclinedPlaneAngleDegrees = 0.0;
    // Stores the gravity magnitude used by the inclined plane phenomenon.
    double gravityMagnitude = 9.8;

    // Stores the ground height used for projectile impact.
    double groundHeight = 0.0;
    // Stores an optional physical target independent of termination.
    TrajectoryTarget target;
    // Shares circular parameters with reusable target kinematics.
    KinematicsConfig circularMotion;
    // Stores analytical launch metrics for the application information display.
    ProjectileMetrics projectileMetrics;
    // Stores the selected inverse solution when a solver mode was requested.
    ProjectileSolution projectileSolution;

    // Identifies the numerical integration method.
    std::string integrator;
    IntegratorType integratorType = IntegratorType::Euler;

    //Defines the type of termination condition and termination value.
    TerminationCondition termination;

    // Defines how many real seconds the complete simulation should take to play.
    double playbackDuration = 0.0;
};