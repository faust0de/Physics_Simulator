#pragma once

#include <memory>
#include <string>
#include <optional>

#include "Integrator.hpp"

// Creates numerical integrator implementations from command identifiers.
class IntegratorFactory {
public:
    static std::unique_ptr<Integrator> create(IntegratorType type);
    static std::optional<IntegratorType> parseType(const std::string& name);
    static const char* name(IntegratorType type);
    static const char* label(IntegratorType type);
    // Creates the numerical integrator associated with the specified identifier.
    static std::unique_ptr<Integrator> create(
        const std::string& integratorName
    );
};