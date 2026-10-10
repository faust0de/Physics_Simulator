#include <raylib.h>
#include <vector>

#include "PhysicalObject.hpp"
#include "FreeFall.hpp"
#include "EulerIntegrator.hpp"
#include "Simulation.hpp"
#include "Renderer.hpp"
#include "Scale.hpp"

// Creates the simulation, runs playback, and handles graphical controls.
int main() {

    // ========================================
    // Initial physical conditions
    // ========================================

    // Creates the physical object used in the simulation.
    PhysicalObject ball(
        2.0,                        // mass: 2 kg
        Vector2D(0.0, 1000.0),     // initial position
        Vector2D(0.0, 0.0)         // initial velocity
    );

    // ========================================
    // Physics
    // ========================================

    // Defines the constant-gravity physical phenomenon.
    FreeFall freeFall;

    // Selects explicit Euler as the numerical integration strategy.
    EulerIntegrator integrator;

    // Coordinates physical evolution and playback state.
    Simulation simulation;

    // Visual playback duration in real seconds.
    const double playbackDuration = 4.0;

    // Configures the relationship between physical and real playback time.
    simulation.configureTimeScale(
        ball,
        freeFall,
        playbackDuration
    );

    // ========================================
    // Renderer
    // ========================================

    // Creates the rendering window and reserves space for playback controls.
    Renderer renderer(
        1000,       // width
        700,        // height
        40.0,       // fallback scale
        140         // control panel height
    );

    // ========================================
    // Scales
    // ========================================

    // Retrieves the physical measurement scales required by the phenomenon.
    std::vector<Scale> scales =
        freeFall.getScales(ball);

    renderer.fitToScales(
        scales
    );

    // Physical duration of the phenomenon.
    double simulationDuration =
        simulation.getDuration(
            freeFall
        );

    // ========================================
    // Control panel
    // ========================================

    bool running = true;

    float panelTop =
        static_cast<float>(
            renderer.getControlPanelTop()
        );

    Rectangle restartButton = {
        250.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    Rectangle playPauseButton = {
        430.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    Rectangle exitButton = {
        610.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    // ========================================
    // Main loop
    // ========================================

    while (
        !renderer.shouldClose() &&
        running
    ) {

        double frameDeltaTime =
            GetFrameTime();

        // ====================================
        // Update physics
        // ====================================

        simulation.step(
            ball,
            freeFall,
            integrator,
            frameDeltaTime
        );

        // ====================================
        // Begin rendering
        // ====================================

        renderer.beginFrame();

        // ====================================
        // Simulation area
        // ====================================

        renderer.drawScales(
            scales
        );

        renderer.drawObject(
            ball
        );

        // Velocity vector
        renderer.drawVelocityVector(
            ball,
            simulationDuration
        );

        // Acceleration vector
        renderer.drawAccelerationVector(
            ball,
            freeFall.getAcceleration(ball),
            simulationDuration
        );

        // Numerical information
        renderer.drawSimulationInfo(
            ball,
            simulation.getCurrentTime()
        );

        // ====================================
        // Control panel
        // ====================================

        renderer.drawControlPanel();

        // ------------------------------------
        // Timeline
        // ------------------------------------

        double selectedTime = 0.0;

        bool seeking =
            renderer.drawTimeline(
                simulation.getCurrentTime(),
                simulationDuration,
                selectedTime
            );

        if (seeking) {

            simulation.seek(
                ball,
                freeFall,
                integrator,
                selectedTime
            );
        }

        // ------------------------------------
        // Restart button
        // ------------------------------------

        if (
            renderer.drawButton(
                restartButton,
                "Restart"
            )
        ) {
            simulation.reset(ball);
        }

        // ------------------------------------
        // Play / Pause button
        // ------------------------------------

        const char* playPauseText =
            simulation.isPaused()
                ? "Play"
                : "Pause";

        if (
            renderer.drawButton(
                playPauseButton,
                playPauseText
            )
        ) {
            simulation.togglePause();
        }

        // ------------------------------------
        // Exit button
        // ------------------------------------

        if (
            renderer.drawButton(
                exitButton,
                "Exit"
            )
        ) {
            running = false;
        }

        // ====================================
        // Finish rendering
        // ====================================

        renderer.endFrame();
    }

    return 0;
}