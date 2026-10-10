#include "IntegratorFactory.hpp"

#include "EulerIntegrator.hpp"

// Creates the numerical integrator associated with the specified identifier.
std::unique_ptr<Integrator>
IntegratorFactory::create(
    const std::string& integratorName
) {

    if (integratorName == "euler") {
        return std::make_unique<EulerIntegrator>();
    }

    return nullptr;
}