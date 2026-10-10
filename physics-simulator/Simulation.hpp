#pragma once

#include "PhysicalObject.hpp"
#include "PhysicalPhenomenon.hpp"
#include "Integrator.hpp"

// Coordinates physical evolution and playback controls.
class Simulation {
private:
    // Stores the elapsed physical simulation time in seconds.
    double current_time;
    // Stores the ratio of physical time to real playback time.
    double timeScale;

    // Stores the physical state used to reconstruct reset and seek operations.
    PhysicalObject initialState;
    // Indicates whether a state has been saved for reset and seek.
    bool hasInitialState;
    // Tracks whether playback is paused.
    bool paused;
public:
    // Initializes physical time and playback state before configuration.
    Simulation();

    // Coordinates one physics step and finalizes the phenomenon when it ends.
    void step(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double frameDeltaTime);

    // Returns the elapsed physical simulation time in seconds.
    double getCurrentTime() const;
    // Returns the ratio of physical time to real playback time.
    double getTimeScale() const;

    // Returns the estimated phenomenon duration from the saved initial state.
    double getDuration(const PhysicalPhenomenon& phenomenon) const;

    // Restores the saved initial physical state and resets physical time.
    void reset(PhysicalObject& object);

    // Saves the initial state and relates physical duration to real playback duration.
    void configureTimeScale(const PhysicalObject& object,const PhysicalPhenomenon& phenomenon,double playbackDuration);

    // Reconstructs the physical state at the requested physical time.
    void seek(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double targetTime);

    // Resumes playback.
    void play();

    // Pauses playback.
    void pause();

    // Toggles the playback pause state.
    void togglePause();

    // Reports whether playback is paused.
    bool isPaused() const;
};