#include "IntegratorFactory.hpp"
#include "EulerIntegrator.hpp"
#include "RK2Integrator.hpp"
#include "RK4Integrator.hpp"

std::optional<IntegratorType> IntegratorFactory::parseType(const std::string& name) {
    if (name == "euler") return IntegratorType::Euler;
    if (name == "rk2") return IntegratorType::RK2;
    if (name == "rk4") return IntegratorType::RK4;
    return std::nullopt;
}

std::unique_ptr<Integrator> IntegratorFactory::create(IntegratorType type) {
    switch (type) {
        case IntegratorType::Euler: return std::make_unique<EulerIntegrator>();
        case IntegratorType::RK2: return std::make_unique<RK2Integrator>();
        case IntegratorType::RK4: return std::make_unique<RK4Integrator>();
    }
    return nullptr;
}

std::unique_ptr<Integrator> IntegratorFactory::create(const std::string& name) {
    auto type = parseType(name);
    return type ? create(*type) : nullptr;
}

const char* IntegratorFactory::name(IntegratorType type) {
    switch (type) {
        case IntegratorType::Euler: return "euler";
        case IntegratorType::RK2: return "rk2";
        case IntegratorType::RK4: return "rk4";
    }
    return "unknown";
}

const char* IntegratorFactory::label(IntegratorType type) {
    switch (type) {
        case IntegratorType::Euler: return "Euler";
        case IntegratorType::RK2: return "RK2 Midpoint";
        case IntegratorType::RK4: return "RK4";
    }
    return "Unknown";
}
