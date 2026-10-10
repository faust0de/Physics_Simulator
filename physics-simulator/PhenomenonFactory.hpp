#pragma once

#include <memory>
#include <string>

#include "PhysicalPhenomenon.hpp"

// Creates physical phenomenon implementations from command identifiers.
class PhenomenonFactory {
public:
    // Creates the physical phenomenon associated with the specified identifier.
    static std::unique_ptr<PhysicalPhenomenon> create(
        const std::string& phenomenonName
    );
};