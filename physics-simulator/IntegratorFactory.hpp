#pragma once

#include <memory>
#include <string>

#include "Integrator.hpp"

// Creates numerical integrator implementations from command identifiers.
class IntegratorFactory {
public:
    // Creates the numerical integrator associated with the specified identifier.
    static std::unique_ptr<Integrator> create(
        const std::string& integratorName
    );
};