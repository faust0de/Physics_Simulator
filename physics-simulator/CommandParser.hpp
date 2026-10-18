#pragma once

#include <string>

#include "SimulationConfig.hpp"

// Stores the result produced after parsing a simulation command.
struct CommandParseResult {

    // Indicates whether the command was parsed and validated successfully.
    bool success = false;

    // Stores the validated simulation configuration.
    SimulationConfig config;

    // Stores the error description when parsing or validation fails.
    std::string errorMessage;
};

// Converts structured command text into a validated simulation configuration.
class CommandParser {
private:
    CommandParseResult parseCommand(const std::string& command,bool deferTargetHorizon) const;
public:

    // Parses and validates a simulation command written in JSON format.
    CommandParseResult parse(
        const std::string& command
    ) const;
};