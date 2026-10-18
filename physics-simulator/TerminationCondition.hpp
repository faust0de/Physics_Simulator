#pragma once

enum class TerminationType{
    Natural,
    Time,
    Distance,
    Speed,
    Target,
    Angle
};

struct TerminationCondition{
    TerminationType Type= TerminationType::Natural;
    double value=0.0;
    // Limits target-seeking simulations in physical seconds.
    double maxTime=0.0;
};