#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <raylib.h>

#include "Renderer.hpp"
#include "CommandEditor.hpp"
#include "CommandParser.hpp"

#include "PhenomenonFactory.hpp"
#include "IntegratorFactory.hpp"

#include "PhysicalObject.hpp"
#include "PhysicalPhenomenon.hpp"
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
    std::unique_ptr<PhysicalPhenomenon>
        phenomenon;

    // Stores the numerical integrator selected by the command.
    std::unique_ptr<Integrator>
        integrator;

    // Coordinates physical evolution and playback.
    Simulation simulation;

    // Stores the physical scales required by the current phenomenon.
    std::vector<Scale> scales;

    // Stores the expected physical duration of the simulation.
    double simulationDuration = 0.0;

    // Indicates whether a valid simulation has been created.
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
            phenomenon &&
            integrator
        ) {

            simulation.step(
                ball,
                *phenomenon,
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

                    // Creates the phenomenon requested by the command.
                    std::unique_ptr<PhysicalPhenomenon>
                        newPhenomenon =
                            PhenomenonFactory::create(
                                result.config.phenomenon
                            );

                    // Creates the integrator requested by the command.
                    std::unique_ptr<Integrator>
                        newIntegrator =
                            IntegratorFactory::create(
                                result.config.integrator
                            );

                    // Verifies that the requested phenomenon can be created.
                    if (!newPhenomenon) {

                        commandValid = false;

                        statusMessage =
                            "Unable to create phenomenon: " +
                            result.config.phenomenon +
                            ".";
                    }

                    // Verifies that the requested integrator can be created.
                    else if (!newIntegrator) {

                        commandValid = false;

                        statusMessage =
                            "Unable to create integrator: " +
                            result.config.integrator +
                            ".";
                    }

                    else {

                        // Replaces the current phenomenon.
                        phenomenon =
                            std::move(
                                newPhenomenon
                            );

                        // Replaces the current numerical integrator.
                        integrator =
                            std::move(
                                newIntegrator
                            );

                        // Creates the physical object from the parsed configuration.
                        ball =
                            PhysicalObject(
                                result.config.mass,
                                result.config.initialPosition,
                                result.config.initialVelocity
                            );

                        // Creates a fresh simulation state.
                        simulation =
                            Simulation();

                        // Configures physical time relative to playback time.
                        simulation.configureTimeScale(
                            ball,
                            *phenomenon,
                            result.config.playbackDuration
                        );

                        // Loads the simulation without starting it automatically.
                        simulation.pause();

                        // Retrieves the physical scales required by the phenomenon.
                        scales =
                            phenomenon->getScales(
                                ball
                            );

                        // Adjusts the renderer to the physical dimensions.
                        renderer.fitToScales(
                            scales
                        );

                        // Calculates the expected physical duration.
                        simulationDuration =
                            simulation.getDuration(
                                *phenomenon
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
            phenomenon &&
            integrator
        ) {

            // Draws the physical measurement scales.
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
                phenomenon->getAcceleration(
                    ball
                ),
                simulationDuration
            );

            // Displays numerical simulation information.
            renderer.drawSimulationInfo(
                ball,
                simulation.getCurrentTime()
            );

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
                    *phenomenon,
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