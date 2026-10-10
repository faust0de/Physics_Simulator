#include "Simulation.hpp"

// Initializes physical time and playback state before configuration.
Simulation::Simulation(): current_time(0.0),timeScale(1.0),hasInitialState(false),paused(false) {
}

// Coordinates one physics step and finalizes the phenomenon when it ends.
void Simulation::step(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double frameDeltaTime) {
    if (paused) {
        return;
    }

    if (phenomenon.isFinished(object)) {
        return;
    }

    // Converts real frame duration into elapsed physical time.
    double physicalDeltaTime =frameDeltaTime * timeScale;

    phenomenon.apply(object,physicalDeltaTime);

    integrator.integrate(object,physicalDeltaTime);

    current_time += physicalDeltaTime;

    if (phenomenon.isFinished(object)) {
        phenomenon.finish(object);
    }
}

// Returns the elapsed physical simulation time in seconds.
double Simulation:: getCurrentTime() const{
    return current_time;
}

// Returns the ratio of physical time to real playback time.
double Simulation:: getTimeScale() const{
    return timeScale;
}

// Restores the saved initial physical state and resets physical time.
void Simulation::reset(PhysicalObject& object) {
    if (!hasInitialState) {
        return;
    }

    object = initialState;

    current_time = 0.0;
}

// Saves the initial state and relates physical duration to real playback duration.
void Simulation::configureTimeScale(const PhysicalObject& object,const PhysicalPhenomenon& phenomenon,double playbackDuration) {
    initialState = object;
    hasInitialState = true;

    double physicalDuration =phenomenon.getExpectedDuration(object);

    if (playbackDuration > 0.0 &&physicalDuration > 0.0) {
        timeScale =physicalDuration /playbackDuration;
    }else {timeScale = 1.0;}
}

// Reconstructs the physical state at the requested physical time.
void Simulation::seek(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double targetTime) {
    if (!hasInitialState) {
        return;
    }

    if (targetTime < 0.0) {
        targetTime = 0.0;
    }

    reset(object);

    // Replays small physics steps to reconstruct the requested time.
    const double physicsStep =1.0 / 240.0;

    while (current_time < targetTime &&!phenomenon.isFinished(object)) {
        double deltaTime = physicsStep;

        if (current_time + deltaTime >targetTime) {
            deltaTime =
                targetTime - current_time;
        }

        phenomenon.apply(object,deltaTime);

        integrator.integrate(object,deltaTime);

        current_time += deltaTime;

        if (phenomenon.isFinished(object)) {
            phenomenon.finish(object);
        }
    }
}

// Resumes playback.
void Simulation::play() {
    paused = false;
}

// Pauses playback.
void Simulation::pause() {
    paused = true;
}

// Toggles the playback pause state.
void Simulation::togglePause() {
    paused = !paused;
}

// Reports whether playback is paused.
bool Simulation::isPaused() const {
    return paused;
}

// Returns the estimated phenomenon duration from the saved initial state.
double Simulation::getDuration(const PhysicalPhenomenon& phenomenon) const {
    if (!hasInitialState) {
        return 0.0;
    }
    return phenomenon.getExpectedDuration(initialState);
}