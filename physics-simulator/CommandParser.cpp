#include "CommandParser.hpp"

#include <exception>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Parses and validates a simulation command written in JSON format.
CommandParseResult CommandParser::parse(
    const std::string& command
) const {

    CommandParseResult result;

    // Rejects an empty command before attempting JSON parsing.
    if (command.empty()) {
        result.errorMessage =
            "Command cannot be empty.";

        return result;
    }

    try {

        json data =
            json::parse(command);

        // The root JSON value must be an object.
        if (!data.is_object()) {
            result.errorMessage =
                "The simulation command must be a JSON object.";

            return result;
        }

        SimulationConfig config;

        // Validates the required phenomenon field.
        if (!data.contains("phenomenon")) {
            result.errorMessage =
                "Missing required field: phenomenon.";

            return result;
        }

        if (!data["phenomenon"].is_string()) {
            result.errorMessage =
                "phenomenon must be a string.";

            return result;
        }

        config.phenomenon =
            data["phenomenon"].get<std::string>();

        // Only free fall is currently supported.
        if (config.phenomenon != "free_fall") {
            result.errorMessage =
                "Unknown phenomenon: " +
                config.phenomenon +
                ".";

            return result;
        }

        // Validates the required object section.
        if (!data.contains("object")) {
            result.errorMessage =
                "Missing required field: object.";

            return result;
        }

        if (!data["object"].is_object()) {
            result.errorMessage =
                "object must be a JSON object.";

            return result;
        }

        const json& object =
            data["object"];

        // Uses 1 kg as the default object mass.
        config.mass = 1.0;

        if (object.contains("mass")) {

            if (!object["mass"].is_number()) {
                result.errorMessage =
                    "object.mass must be a number.";

                return result;
            }

            config.mass =
                object["mass"].get<double>();
        }

        if (config.mass <= 0.0) {
            result.errorMessage =
                "object.mass must be greater than 0.";

            return result;
        }

        // Validates the required position section.
        if (!object.contains("position")) {
            result.errorMessage =
                "Missing required field: object.position.";

            return result;
        }

        if (!object["position"].is_object()) {
            result.errorMessage =
                "object.position must be a JSON object.";

            return result;
        }

        const json& position =
            object["position"];

        // Uses zero as the default horizontal position.
        double positionX = 0.0;

        if (position.contains("x")) {

            if (!position["x"].is_number()) {
                result.errorMessage =
                    "object.position.x must be a number.";

                return result;
            }

            positionX =
                position["x"].get<double>();
        }

        // Vertical position is required for free fall.
        if (!position.contains("y")) {
            result.errorMessage =
                "Missing required field: object.position.y.";

            return result;
        }

        if (!position["y"].is_number()) {
            result.errorMessage =
                "object.position.y must be a number.";

            return result;
        }

        double positionY =
            position["y"].get<double>();

        if (positionY <= 0.0) {
            result.errorMessage =
                "object.position.y must be greater than 0 for free_fall.";

            return result;
        }

        config.initialPosition =
            Vector2D(
                positionX,
                positionY
            );

        // Uses zero initial velocity when no velocity is provided.
        double velocityX = 0.0;
        double velocityY = 0.0;

        if (object.contains("velocity")) {

            if (!object["velocity"].is_object()) {
                result.errorMessage =
                    "object.velocity must be a JSON object.";

                return result;
            }

            const json& velocity =
                object["velocity"];

            if (velocity.contains("x")) {

                if (!velocity["x"].is_number()) {
                    result.errorMessage =
                        "object.velocity.x must be a number.";

                    return result;
                }

                velocityX =
                    velocity["x"].get<double>();
            }

            if (velocity.contains("y")) {

                if (!velocity["y"].is_number()) {
                    result.errorMessage =
                        "object.velocity.y must be a number.";

                    return result;
                }

                velocityY =
                    velocity["y"].get<double>();
            }
        }

        // Free fall currently supports only objects released from rest.
        if (velocityX != 0.0) {
            result.errorMessage =
                "free_fall currently requires initial velocity.x = 0.";

            return result;
        }

        if (velocityY != 0.0) {
            result.errorMessage =
                "free_fall currently requires initial velocity.y = 0.";

            return result;
        }

        config.initialVelocity =
            Vector2D(
                velocityX,
                velocityY
            );

        // Applies default simulation settings.
        config.integrator = "euler";
        config.playbackDuration = 4.0;

        if (data.contains("simulation")) {

            if (!data["simulation"].is_object()) {
                result.errorMessage =
                    "simulation must be a JSON object.";

                return result;
            }

            const json& simulation =
                data["simulation"];

            if (simulation.contains("integrator")) {

                if (!simulation["integrator"].is_string()) {
                    result.errorMessage =
                        "simulation.integrator must be a string.";

                    return result;
                }

                config.integrator =
                    simulation["integrator"].get<std::string>();
            }

            if (simulation.contains("playback_duration")) {

                if (!simulation["playback_duration"].is_number()) {
                    result.errorMessage =
                        "simulation.playback_duration must be a number.";

                    return result;
                }

                config.playbackDuration =
                    simulation["playback_duration"].get<double>();
            }
        }

        // Only the Euler integrator is currently implemented.
        if (config.integrator != "euler") {
            result.errorMessage =
                "Unknown integrator: " +
                config.integrator +
                ".";

            return result;
        }

        if (config.playbackDuration <= 0.0) {
            result.errorMessage =
                "simulation.playback_duration must be greater than 0.";

            return result;
        }

        // Returns the fully validated configuration.
        result.success = true;
        result.config = config;
        result.errorMessage = "";

        return result;
    }
    catch (const json::parse_error& error) {

        result.errorMessage =
            std::string("Invalid JSON: ") +
            error.what();

        return result;
    }
    catch (const std::exception& error) {

        result.errorMessage =
            std::string("Command parsing error: ") +
            error.what();

        return result;
    }
}