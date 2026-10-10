#include "PhenomenonFactory.hpp"

#include "FreeFall.hpp"

// Creates the physical phenomenon associated with the specified identifier.
std::unique_ptr<PhysicalPhenomenon>
PhenomenonFactory::create(
    const std::string& phenomenonName
) {

    if (phenomenonName == "free_fall") {
        return std::make_unique<FreeFall>();
    }

    return nullptr;
}