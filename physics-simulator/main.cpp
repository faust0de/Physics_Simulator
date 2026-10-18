#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <raylib.h>

#include "Renderer.hpp"
#include "CommandEditor.hpp"
#include "CommandParser.hpp"

#include "IntegratorFactory.hpp"

#include "PhysicalObject.hpp"
#include "Kinematics.hpp"
#include <optional>
#include "Integrator.hpp"
#include "Simulation.hpp"
#include "Scale.hpp"




// Draws text using automatic word wrapping inside a rectangle.
void drawWrappedText(
    const std::string& text,
    Rectangle bounds,
    int fontSize,
    Color color
) {
    Font font = GetFontDefault();

    std::istringstream stream(text);

    std::string word;
    std::string line;

    float y = bounds.y;

    float lineHeight =
        static_cast<float>(
            fontSize + 5
        );

    while (stream >> word) {

        std::string testLine =
            line.empty()
                ? word
                : line + " " + word;

        float width =
            MeasureTextEx(
                font,
                testLine.c_str(),
                static_cast<float>(fontSize),
                1.0f
            ).x;

        if (
            width > bounds.width &&
            !line.empty()
        ) {

            DrawTextEx(
                font,
                line.c_str(),
                Vector2{
                    bounds.x,
                    y
                },
                static_cast<float>(fontSize),
                1.0f,
                color
            );

            line = word;

            y += lineHeight;
        }
        else {

            line = testLine;
        }
    }

    if (!line.empty()) {

        DrawTextEx(
            font,
            line.c_str(),
            Vector2{
                bounds.x,
                y
            },
            static_cast<float>(fontSize),
            1.0f,
            color
        );
    }
}

int main() {

    // Creates the application window.
    Renderer renderer(
        1000,
        700,
        40.0,
        140
    );

    // Creates the simulation command editor.
    CommandEditor editor(
        Rectangle{
            50.0f,
            80.0f,
            900.0f,
            450.0f
        },
        18
    );

    // Converts JSON command text into simulation configuration data.
    CommandParser parser;

    // Stores the physical object used by the current simulation.
    PhysicalObject ball;

    // Stores the physical phenomenon selected by the command.
    std::optional<Kinematics> kinematics;

    // Stores the numerical integrator selected by the command.
    std::unique_ptr<Integrator>
        integrator;

    // Coordinates physical evolution and playback.
    Simulation simulation;

    // Stores the physical scales required by the current phenomenon.
    std::vector<Scale> scales;
    // Stores measurement decomposition and the optional geometric angle guide.
    KinematicsVisuals visuals;

    // Stores the expected physical duration of the simulation.
    double simulationDuration = 0.0;

    // Indicates whether a valid simulation has been created.
    TrajectoryTarget target;
    ProjectileMetrics projectileMetrics;
    bool showProjectileMetrics = false;
    bool simulationLoaded = false;

    // Controls whether the command editor or simulation is displayed.
    bool showEditor = true;

    // Controls the application main loop.
    bool running = true;

    // Stores feedback from command parsing.
    std::string statusMessage =
        "Enter a simulation command.";

    // Indicates whether the latest command was valid.
    bool commandValid = false;

    // Defines the command execution button.
    Rectangle runButton = {
        400.0f,
        555.0f,
        200.0f,
        50.0f
    };

    // Gets the top position of the simulation control panel.
    float panelTop =
        static_cast<float>(
            renderer.getControlPanelTop()
        );

    // Defines the simulation restart button.
    Rectangle restartButton = {
        170.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    // Defines the simulation play and pause button.
    Rectangle playPauseButton = {
        330.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    // Defines the button used to return to command editing.
    Rectangle editButton = {
        490.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    // Defines the application exit button.
    Rectangle exitButton = {
        650.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    while (
        !renderer.shouldClose() &&
        running
    ) {

        // Updates text input only while the editor is visible.
        if (showEditor) {
            editor.update();
        }

        // Gets the real elapsed time since the previous frame.
        double frameDeltaTime =
            GetFrameTime();

        // Advances the active simulation.
        if (
            simulationLoaded &&
            !showEditor &&
            kinematics &&
            integrator
        ) {

            simulation.step(
                ball,
                *kinematics,
                *integrator,
                frameDeltaTime
            );
        }

        renderer.beginFrame();

        // ========================================
        // Command editor screen
        // ========================================

        if (showEditor) {

            DrawText(
                "Simulation Command",
                50,
                35,
                26,
                DARKGRAY
            );

            editor.draw();

            // Parses and loads the command.
            if (
                renderer.drawButton(
                    runButton,
                    "Run Command"
                )
            ) {

                CommandParseResult result =
                    parser.parse(
                        editor.getText()
                    );

                if (result.success) {

                    // Creates the integrator requested by the command.
                    std::unique_ptr<Integrator>
                        newIntegrator =
                            IntegratorFactory::create(
                                result.config.integratorType
                            );

                    PhysicalObject newBall(result.config.mass,result.config.initialPosition,result.config.initialVelocity);
                    Simulation newSimulation;
                    std::string sequenceError;
                    // Preflight completes before any live state is replaced.
                    // Verifies that the requested integrator can be created.
                    if (!newIntegrator) {

                        commandValid = false;

                        statusMessage =
                            "Unable to create integrator: " +
                            result.config.integrator +
                            ".";
                    }

                    else if (!newSimulation.configureSequence(newBall,result.config,*newIntegrator,sequenceError)) {
                        commandValid = false;
                        statusMessage = sequenceError;
                    }
                    else {

                        kinematics = newSimulation.getCurrentKinematics();

                        // Replaces the current numerical integrator.
                        integrator =
                            std::move(
                                newIntegrator
                            );

                        // Install the prepared initial state after successful validation.
                        ball = newBall;
                        simulation = std::move(newSimulation);
                        target = simulation.getCurrentTarget();
                        projectileMetrics = simulation.getCurrentProjectileMetrics();
                        showProjectileMetrics = kinematics->isProjectile();

                        // Loads the simulation without starting it automatically.
                        simulation.pause();

                        // Retrieves physical reference scales and the optional angle guide.
                        visuals = simulation.getCurrentVisuals();
                        scales = visuals.scales;

                        // Adjusts the renderer to the physical dimensions.
                        auto previewBounds = simulation.getTargetPreviewBounds();
                        const auto& sequenceBounds = simulation.getSequenceBounds();
                        previewBounds.insert(previewBounds.end(),sequenceBounds.begin(),sequenceBounds.end());
                        if (kinematics->isCircular()) {
                            auto bounds = kinematics->getPreviewBounds(ball,0.0);
                            previewBounds.insert(previewBounds.end(),bounds.begin(),bounds.end());
                        }
                        renderer.fitToScales(
                            scales,
                            ball.getPosition(),
                            target,
                            previewBounds
                        );

                        // Calculates the expected physical duration.
                        simulationDuration =
                            simulation.getDuration(
                                *kinematics
                            );

                        // Marks the new simulation as available.
                        simulationLoaded = true;

                        // Switches from command editing to simulation view.
                        showEditor = false;

                        commandValid = true;

                        statusMessage =
                            "Command loaded successfully.";

                        // Prints the parsed configuration for debugging.
                        std::cout
                            << "\n===== SIMULATION CONFIG =====\n";

                        std::cout
                            << "Phenomenon: "
                            << result.config.phenomenon
                            << '\n';

                        std::cout
                            << "Mass: "
                            << result.config.mass
                            << " kg\n";

                        std::cout
                            << "Position: ("
                            << result.config.initialPosition.x
                            << ", "
                            << result.config.initialPosition.y
                            << ") m\n";

                        std::cout
                            << "Velocity: ("
                            << result.config.initialVelocity.x
                            << ", "
                            << result.config.initialVelocity.y
                            << ") m/s\n";

                        std::cout
                            << "Integrator: "
                            << result.config.integrator
                            << '\n';

                        std::cout
                            << "Playback duration: "
                            << result.config.playbackDuration
                            << " s\n";

                        std::cout
                            << "=============================\n";
                    }
                }
                else {

                    commandValid = false;

                    statusMessage =
                        result.errorMessage;

                    std::cout
                        << "Command error: "
                        << result.errorMessage
                        << '\n';
                }
            }

            // Displays parser and validation feedback.
            drawWrappedText(
                statusMessage,
                Rectangle{
                    50.0f,
                    620.0f,
                    900.0f,
                    70.0f
                },
                18,
                commandValid
                    ? DARKGREEN
                    : MAROON
            );
        }

        // ========================================
        // Simulation screen
        // ========================================

        else if (
            simulationLoaded &&
            kinematics &&
            integrator
        ) {

            // Only the active regime supplies guides; the complete numerical trail remains.
            kinematics = simulation.getCurrentKinematics();
            visuals = simulation.getCurrentVisuals();
            scales = visuals.scales;
            projectileMetrics = simulation.getCurrentProjectileMetrics();
            showProjectileMetrics = kinematics->isProjectile();
            // Draws the physical measurement scales.
            renderer.drawCircularGuide(visuals.circularGuide);
            renderer.drawAngleGuide(visuals.angleGuide);
            renderer.drawTrajectory(simulation.getTrajectoryHistory());
            renderer.drawTarget(simulation.getCurrentTarget(),simulation.getActiveTargetHit().hit);
            renderer.drawScales(
                scales
            );

            // Draws the simulated object.
            renderer.drawObject(
                ball
            );

            // Draws the current velocity vector.
            renderer.drawVelocityVector(
                ball,
                simulationDuration
            );

            // Draws the acceleration provided by the current phenomenon.
            renderer.drawAccelerationVector(
                ball,
                kinematics->getAcceleration(
                    ball
                ),
                simulationDuration
            );

            // Displays numerical simulation information.
            renderer.drawSimulationInfo(
                ball,
                simulation.getCurrentTime()
            );

            target = simulation.getCurrentTarget();
            const TargetHitResult& displayedTargetHit = simulation.getActiveTargetHit();
            renderer.drawTargetStatus(target,displayedTargetHit,
                simulation.getMinimumTargetDistance(),simulation.isFinished(ball,*kinematics));
            if (simulation.usesMotionSequence()) {
                DrawText(TextFormat("Phase %i / %i: %s",static_cast<int>(simulation.getCurrentPhaseIndex()+1),
                    static_cast<int>(simulation.getPhaseCount()),kinematics->getConfiguration().phenomenon.c_str()),20,104,16,DARKGRAY);
                DrawText(TextFormat("Phase time: %.3f s / distance: %.3f m",simulation.getPhaseElapsedTime(),simulation.getPhaseTraveledDistance()),20,124,16,DARKGRAY);
                DrawText(TextFormat("Total distance: %.3f m / phase angle: %.2f deg",simulation.getTraveledDistance(),simulation.getPhaseTraveledAngle()*180.0/std::acos(-1.0)),20,144,16,DARKGRAY);
                const char* untilName = "Natural";
                switch (simulation.getCurrentUntil().Type) {
                    case TerminationType::Time: untilName = "Time"; break;
                    case TerminationType::Distance: untilName = "Distance"; break;
                    case TerminationType::Speed: untilName = "Speed"; break;
                    case TerminationType::Angle: untilName = "Angle"; break;
                    case TerminationType::Target: untilName = "Target"; break;
                    case TerminationType::Natural: break;
                }
                const char* targetStatus = !simulation.hasActiveTarget() ? "None" :
                    (simulation.getActiveTargetHit().hit ? "HIT" : (simulation.isSequenceComplete() ? "MISS" : "Pending"));
                DrawText(TextFormat("Until: %s / Target: %s / %s",untilName,targetStatus,
                    IntegratorFactory::label(simulation.getCurrentIntegratorType())),20,164,16,DARKGRAY);
                if (simulation.isSequenceComplete() && !simulation.hasReachedAngularStop())
                    DrawText(simulation.isTargetMiss() ? "Sequence: MISS (target condition not reached)" : "Sequence complete",20,186,16,simulation.isTargetMiss() ? MAROON : DARKGREEN);
            }
            if (!simulation.usesMotionSequence())
                DrawText(TextFormat("Integrator: %s",IntegratorFactory::label(simulation.getCurrentIntegratorType())),20,110,16,DARKGRAY);
            if (kinematics->isCircular()) {
                CircularMetrics metrics = kinematics->getCircularMetrics(ball);
                DrawText(TextFormat("Radius: %.3f / current: %.3f m",metrics.configuredRadius,metrics.currentRadius),600,16,16,DARKGRAY);
                DrawText(TextFormat("Radial error: %+.6f m",metrics.radialError),600,34,16,DARKGRAY);
                DrawText(TextFormat("Angle: %.2f / traveled: %.2f deg",metrics.angleDegrees,simulation.getTraveledAngle()*180.0/std::acos(-1.0)),600,52,16,DARKGRAY);
                DrawText(TextFormat("Omega: %.4f rad/s / alpha: %.4f rad/s^2",metrics.angularVelocity,metrics.angularAcceleration),600,70,16,DARKGRAY);
                DrawText(TextFormat("Tangential speed: %.4f m/s",metrics.tangentialSpeed),600,88,16,DARKGRAY);
                DrawText(TextFormat(simulation.usesMotionSequence() ? "aC: %.4f / aT: %.4f m/s^2" : "a centripetal: %.4f / tangential: %.4f m/s^2",metrics.centripetalAcceleration,metrics.tangentialAcceleration),simulation.usesMotionSequence() ? 600 : 20,simulation.usesMotionSequence() ? 172 : 148,16,DARKGRAY);
                if (simulation.hasReachedAngularStop())
                    DrawText("Angular stop: radial Euler velocity preserved",20,simulation.usesMotionSequence() ? 186 : 170,16,MAROON);
            }
            if (showProjectileMetrics) {
                DrawText(TextFormat("Initial height: %.3f m",projectileMetrics.initialHeightAboveGround),600,16,16,DARKGRAY);
                DrawText(TextFormat("Rise / maximum: %.3f / %.3f m",projectileMetrics.riseAboveLaunch,projectileMetrics.maximumHeightAboveGround),600,36,16,DARKGRAY);
                DrawText(TextFormat("Apex: (%.2f, %.2f), t=%.3f s",projectileMetrics.apexPosition.x,projectileMetrics.apexPosition.y,projectileMetrics.apexTime),600,56,16,DARKGRAY);
                DrawText(TextFormat("Impact: (%.2f, %.2f), t=%.3f s",projectileMetrics.impactPosition.x,projectileMetrics.impactPosition.y,projectileMetrics.impactTime),600,76,16,DARKGRAY);
                DrawText(TextFormat("Horizontal range: %.3f m",projectileMetrics.horizontalRange),600,96,16,DARKGRAY);
            }

            // Draws the simulation control area.
            renderer.drawControlPanel();

            // ====================================
            // Timeline
            // ====================================

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
                    *kinematics,
                    *integrator,
                    selectedTime
                );
            }

            // ====================================
            // Restart
            // ====================================

            if (
                renderer.drawButton(
                    restartButton,
                    "Restart"
                )
            ) {

                simulation.reset(
                    ball
                );
            }

            // ====================================
            // Play / Pause
            // ====================================

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

            // ====================================
            // Edit command
            // ====================================

            if (
                renderer.drawButton(
                    editButton,
                    "Edit Command"
                )
            ) {

                simulation.pause();

                showEditor = true;

                statusMessage =
                    "Edit the command and press Run Command.";

                commandValid = true;
            }

            // ====================================
            // Exit
            // ====================================

            if (
                renderer.drawButton(
                    exitButton,
                    "Exit"
                )
            ) {

                running = false;
            }
        }

        renderer.endFrame();
    }

    return 0;
}