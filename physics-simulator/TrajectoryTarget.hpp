#pragma once
#include "Vector2D.hpp"
#include "Kinematics.hpp"

// Selects the permitted vertical direction when a trajectory enters the target.
enum class TargetCrossing { Any, Ascending, Descending };

// Describes a generic circular target in physical world coordinates.
struct TrajectoryTarget {
    // Enables target observation without changing termination.
    bool enabled = false;
    // Stores the physical center of the target.
    Vector2D position;
    // Stores the physical acceptance radius in meters.
    double radius = 0.25;
    // Restricts the vertical crossing direction.
    TargetCrossing crossing = TargetCrossing::Any;
    // Enables a second physical body using the same numerical integration strategy.
    bool moving = false;
    // Stores the optional physical mass of the target body.
    double mass = 1.0;
    // Stores the configured initial target velocity.
    Vector2D velocity;
    // Stores reusable motion parameters without duplicating application configuration.
    KinematicsConfig motion;
};
