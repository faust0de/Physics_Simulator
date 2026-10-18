#include "CommandParser.hpp"
#include "Simulation.hpp"
#include "IntegratorFactory.hpp"

#include <exception>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <functional>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {
double number(const json& value,const char* key,double fallback,bool required = false) {
    if (!value.contains(key)) {
        if (required) throw std::runtime_error(std::string("Missing required field: ")+key+".");
        return fallback;
    }
    if (!value[key].is_number()) throw std::runtime_error(std::string(key)+" must be a number.");
    double result = value[key].get<double>();
    if (!std::isfinite(result)) throw std::runtime_error(std::string(key)+" must be finite.");
    return result;
}

PhysicalObject parseCircular(const json& parent,KinematicsConfig& motion,double mass) {
    if (!parent.contains("circular_motion") || !parent["circular_motion"].is_object())
        throw std::runtime_error("circular_motion must be a JSON object.");
    const json& c = parent["circular_motion"];
    if (!c.contains("center") || !c["center"].is_object())
        throw std::runtime_error("circular_motion.center must be a JSON object.");
    motion.circularCenter = Vector2D(number(c["center"],"x",0.0,true),number(c["center"],"y",0.0,true));
    motion.circularRadius = number(c,"radius",0.0,true);
    motion.initialAngleDegrees = number(c,"initial_angle_degrees",0.0);
    motion.initialAngularVelocity = number(c,"angular_velocity",0.0,true);
    motion.angularAcceleration = number(c,"angular_acceleration",0.0,
        motion.phenomenon == "uniformly_accelerated_circular_motion");
    if (motion.phenomenon == "uniform_circular_motion" && c.contains("angular_acceleration"))
        throw std::runtime_error("angular_acceleration is only configurable for MCUA.");
    Kinematics model(motion);
    TerminationCondition check;
    check.Type = TerminationType::Time;
    check.value = 0.0;
    std::string error = model.validate(PhysicalObject(),check);
    if (!error.empty()) throw std::runtime_error(error);
    PhysicalObject state = model.calculateCircularInitialState(mass);
    for (const Vector2D& value : {state.getPosition(),state.getVelocity(),state.getAcceleration()})
        if (!std::isfinite(value.x) || !std::isfinite(value.y))
            throw std::runtime_error("Circular configuration produces a non-finite initial state.");
    if (!std::isfinite(mass)) throw std::runtime_error("Circular mass must be finite.");
    return state;
}

TrajectoryTarget parseTarget(const json& target,bool hasSolver,bool phaseLocal = false) {
    TrajectoryTarget result;
    if (target.is_object() && (target.contains("integrator") ||
        (target.contains("motion") && target["motion"].is_object() && target["motion"].contains("integrator"))))
        throw std::runtime_error("A target uses its phase integrator; target.integrator is not supported.");
    if (phaseLocal && target.is_object() && (target.contains("motion_sequence") ||
        (target.contains("motion") && target["motion"].is_object() && target["motion"].contains("motion_sequence"))))
        throw std::runtime_error("Target motion_sequence is not supported.");
    if (!target.is_object()) throw std::runtime_error("target must be a JSON object.");
    bool circularTarget = target.contains("motion") && target["motion"].is_object() &&
        target["motion"].contains("phenomenon") && target["motion"]["phenomenon"].is_string() &&
        (target["motion"]["phenomenon"] == "uniform_circular_motion" ||
         target["motion"]["phenomenon"] == "uniformly_accelerated_circular_motion");
    result.enabled = true;
    if (circularTarget) {
        for (const char* key : {"position","velocity","acceleration"})
            if (target.contains(key)) throw std::runtime_error(std::string("target.")+key+" is derived automatically for circular motion.");
    }
    else {
        if (!target.contains("position") || !target["position"].is_object())
            throw std::runtime_error("target.position must be a JSON object.");
        result.position = Vector2D(number(target["position"],"x",0.0,true),number(target["position"],"y",0.0,true));
    }
    result.radius = number(target,"radius",0.25);
    if (result.radius <= 0.0) throw std::runtime_error("target.radius must be greater than 0.");
    if (target.contains("crossing") && !target["crossing"].is_string())
        throw std::runtime_error("target.crossing must be a string.");
    std::string crossing = target.value("crossing",std::string("any"));
    if (crossing == "ascending") result.crossing = TargetCrossing::Ascending;
    else if (crossing == "descending") result.crossing = TargetCrossing::Descending;
    else if (crossing != "any") throw std::runtime_error("Unknown target crossing.");

    if (target.contains("motion")) {
        const json& motion = target["motion"];
        if (!motion.is_object() || !motion.contains("phenomenon") || !motion["phenomenon"].is_string())
            throw std::runtime_error("target.motion.phenomenon must be a string.");
        if (hasSolver || motion.contains("target_solution") || target.contains("target_solution"))
            throw std::runtime_error("Projectile target_solution currently requires a static target.");
        result.moving = true;
        result.mass = number(target,"mass",1.0);
        if (result.mass <= 0.0) throw std::runtime_error("target.mass must be greater than 0.");
        auto vector = [&](const json& parent,const char* key) {
            if (!parent.contains(key)) return Vector2D();
            if (!parent[key].is_object()) throw std::runtime_error(std::string("target.")+key+" must be an object.");
            return Vector2D(number(parent[key],"x",0.0),number(parent[key],"y",0.0));
        };
        result.velocity = vector(target,"velocity");
        result.motion.phenomenon = motion["phenomenon"].get<std::string>();
        result.motion.gravityMagnitude = number(motion,"gravity",9.8);
        result.motion.groundHeight = number(motion,"ground_height",0.0);
        if (result.motion.gravityMagnitude <= 0.0)
            throw std::runtime_error("target.motion.gravity must be greater than 0.");
        if (result.motion.phenomenon == "uniformly_accelerated_motion") {
            if (!target.contains("acceleration")) throw std::runtime_error("Target MRUA requires acceleration.");
            result.motion.initialAcceleration = vector(target,"acceleration");
        }
        else if (target.contains("acceleration"))
            throw std::runtime_error("target.acceleration is only configurable for uniformly_accelerated_motion.");
        if (result.motion.phenomenon == "inclined_plane_motion") {
            result.motion.inclinedPlaneAngleDegrees = number(motion,"angle_degrees",0.0,true);
            double angle = result.motion.inclinedPlaneAngleDegrees;
            if (std::abs(angle) <= 1e-9 || std::abs(angle) >= 90.0)
                throw std::runtime_error("target.motion.angle_degrees must be between -90 and 90 degrees and cannot be zero.");
        }
        if (circularTarget) {
            PhysicalObject state = parseCircular(motion,result.motion,result.mass);
            result.position = state.getPosition();
            result.velocity = state.getVelocity();
        }
        else if (motion.contains("circular_motion"))
            throw std::runtime_error("target.motion.circular_motion requires a circular phenomenon.");
        // Construction resolves the same identifiers and physical model as the primary.
        Kinematics targetModel(result.motion);
        (void)targetModel;
    }
    if (phaseLocal && result.moving) {
        Kinematics model(result.motion);
        PhysicalObject state(result.mass,result.position,result.velocity);
        TerminationCondition initial;
        initial.Type = TerminationType::Time;
        std::string error = model.validate(state,initial);
        if (!error.empty()) throw std::runtime_error("Invalid phase target: "+error);
    }
    return result;
}

double phaseNumber(const json& parent,const char* key,double fallback,bool required = false) {
    if (!parent.contains(key)) {
        if (required) throw std::runtime_error(std::string("Missing phase field: ")+key+".");
        return fallback;
    }
    if (!parent[key].is_number()) throw std::runtime_error(std::string("Phase ")+key+" must be a number.");
    double value = parent[key].get<double>();
    if (!std::isfinite(value)) throw std::runtime_error(std::string("Phase ")+key+" must be finite.");
    return value;
}

TerminationCondition phaseUntil(const json& phase) {
    if (!phase.contains("until") || !phase["until"].is_object()) throw std::runtime_error("Every motion phase requires until.");
    const auto& until = phase["until"];
    if (!until.contains("type") || !until["type"].is_string()) throw std::runtime_error("Phase until.type must be a string.");
    std::string type = until["type"].get<std::string>();
    TerminationCondition result;
    if (type == "natural") result.Type = TerminationType::Natural;
    else if (type == "time") result.Type = TerminationType::Time;
    else if (type == "distance") result.Type = TerminationType::Distance;
    else if (type == "speed") result.Type = TerminationType::Speed;
    else if (type == "angle") result.Type = TerminationType::Angle;
    else if (type == "target") result.Type = TerminationType::Target;
    else throw std::runtime_error("Unknown phase until.type.");
    if (result.Type == TerminationType::Target) {
        if (until.contains("value")) throw std::runtime_error("Phase Target uses max_time, not value.");
        result.maxTime = phaseNumber(until,"max_time",0.0,true);
        if (result.maxTime <= 0.0) throw std::runtime_error("Phase max_time must be greater than 0.");
    } else if (result.Type != TerminationType::Natural) {
        result.value = phaseNumber(until,"value",0.0,true);
        if (result.value < 0.0 || (result.value == 0.0 && result.Type != TerminationType::Speed))
            throw std::runtime_error("Phase until.value must be positive (or zero for Speed).");
    }
    return result;
}

void phaseAcceleration(const json& phase,MotionPhase& result) {
    if (!phase.contains("acceleration") || !phase["acceleration"].is_object())
        throw std::runtime_error("MRUA phase requires a new acceleration object.");
    const auto& acceleration = phase["acceleration"];
    if (acceleration.contains("mode")) {
        if (acceleration["mode"] != "along_velocity" || acceleration.contains("x") || acceleration.contains("y"))
            throw std::runtime_error("Phase acceleration mode must be along_velocity without vector components.");
        result.accelerationAlongVelocity = true;
        result.accelerationValue = phaseNumber(acceleration,"value",0.0,true);
        if (std::abs(result.accelerationValue) <= 1e-9) throw std::runtime_error("Phase acceleration must be non-zero.");
    } else {
        result.kinematics.initialAcceleration = Vector2D(phaseNumber(acceleration,"x",0.0),phaseNumber(acceleration,"y",0.0));
        if (result.kinematics.initialAcceleration.magnitude() <= 1e-9) throw std::runtime_error("MRUA phase acceleration must be non-zero.");
    }
}

CommandParseResult parseSequence(const json& data,const std::function<CommandParseResult(const std::string&,bool)>& parseFirst) {
    if (data.contains("phenomenon")) throw std::runtime_error("Ambiguous command: phenomenon and motion_sequence cannot coexist.");
    const auto& phases = data["motion_sequence"];
    if (!phases.is_array() || phases.empty()) throw std::runtime_error("motion_sequence must be a non-empty array.");
    if (phases.size() > maximumMotionPhases) throw std::runtime_error("motion_sequence supports at most 64 phases.");
    if (data.contains("simulation") && (!data["simulation"].is_object() || data["simulation"].contains("termination")))
        throw std::runtime_error("motion_sequence cannot contain simulation.termination.");
    for (const char* key : {"circular_motion","inclined_plane","projectile","target_solution"})
        if (data.contains(key)) throw std::runtime_error(std::string("Sequence ")+key+" must belong to a phase.");
    if (data.contains("target") && data["target"].is_object() &&
        (data["target"].contains("motion_sequence") || (data["target"].contains("motion") && data["target"]["motion"].is_object() && data["target"]["motion"].contains("motion_sequence"))))
        throw std::runtime_error("Target motion_sequence is not supported.");
    bool localTargets = std::any_of(phases.begin(),phases.end(),[](const json& phase) { return phase.is_object() && phase.contains("target"); });
    if (data.contains("target") && localTargets)
        throw std::runtime_error("Use either one shared sequence target or phase-local targets, not both.");
    std::string defaultName = "euler";
    if (data.contains("simulation") && data["simulation"].contains("integrator")) {
        if (!data["simulation"]["integrator"].is_string()) throw std::runtime_error("simulation.integrator must be a string.");
        defaultName = data["simulation"]["integrator"].get<std::string>();
    }
    auto defaultType = IntegratorFactory::parseType(defaultName);
    if (!defaultType) throw std::runtime_error("Unknown integrator: "+defaultName+".");
    std::vector<MotionPhase> sequence;
    for (std::size_t i=0;i<phases.size();++i) {
        const auto& phase = phases[i];
        if (!phase.is_object() || !phase.contains("phenomenon") || !phase["phenomenon"].is_string())
            throw std::runtime_error("Each phase requires a phenomenon string.");
        if (phase.contains("object") || phase.contains("mass") || phase.contains("position") || phase.contains("velocity"))
            throw std::runtime_error("Motion phases inherit position, velocity and mass from the global object.");
        for (auto field=phase.begin();field!=phase.end();++field)
            if (field.key() != "phenomenon" && field.key() != "until" && field.key() != "acceleration" &&
                field.key() != "circular_motion" && field.key() != "projectile" && field.key() != "inclined_plane" && field.key() != "target_solution" && field.key() != "target" && field.key() != "integrator")
                throw std::runtime_error("Unsupported motion phase field: "+field.key()+".");
        if (phase.contains("termination")) throw std::runtime_error("Motion phases use until, not termination.");
        MotionPhase entry;
        entry.integrator = *defaultType;
        if (phase.contains("integrator")) {
            if (!phase["integrator"].is_string()) throw std::runtime_error("Phase integrator must be a string.");
            std::string name = phase["integrator"].get<std::string>();
            auto type = IntegratorFactory::parseType(name);
            if (!type) throw std::runtime_error("Unknown integrator: "+name+".");
            entry.integrator = *type;
        }
        entry.kinematics.phenomenon = phase["phenomenon"].get<std::string>();
        entry.until = phaseUntil(phase);
        if (phase.contains("target")) entry.target = parseTarget(phase["target"],phase.contains("target_solution"),true);
        Kinematics model(entry.kinematics);
        bool circular = model.isCircular();
        if (i > 0) {
            if (phase.contains("target_solution")) throw std::runtime_error("target_solution is only supported in the first phase.");
            if (circular) {
                if (!phase.contains("circular_motion") || !phase["circular_motion"].is_object())
                    throw std::runtime_error("Circular phase requires circular_motion.");
                const auto& c = phase["circular_motion"];
                for (const char* key : {"radius","initial_angle_degrees","angular_velocity"})
                    if (c.contains(key)) throw std::runtime_error("Subsequent circular phases inherit radius, angle and angular velocity.");
                if (!c.contains("center") || !c["center"].is_object()) throw std::runtime_error("Circular phase requires center.");
                entry.kinematics.circularCenter = Vector2D(phaseNumber(c["center"],"x",0.0,true),phaseNumber(c["center"],"y",0.0,true));
                entry.kinematics.angularAcceleration = phaseNumber(c,"angular_acceleration",0.0,entry.kinematics.phenomenon == "uniformly_accelerated_circular_motion");
                if (entry.kinematics.phenomenon == "uniformly_accelerated_circular_motion" && std::abs(entry.kinematics.angularAcceleration) <= 1e-9)
                    throw std::runtime_error("MCUA phase requires non-zero angular_acceleration.");
                if (entry.kinematics.phenomenon == "uniform_circular_motion" && c.contains("angular_acceleration"))
                    throw std::runtime_error("MCU phase does not accept angular_acceleration.");
            } else if (phase.contains("circular_motion")) throw std::runtime_error("circular_motion requires a circular phase.");
            if (entry.kinematics.phenomenon == "projectile_motion") {
                auto projectile = phase.value("projectile",json::object());
                if (!projectile.is_object()) throw std::runtime_error("Phase projectile must be an object.");
                for (const char* key : {"launch_speed","launch_angle_degrees","launch_angle","target_solution","velocity","position"})
                    if (projectile.contains(key) || phase.contains(key)) throw std::runtime_error("Subsequent projectile phases inherit position and velocity; launch overrides are forbidden.");
                entry.kinematics.gravityMagnitude = phaseNumber(projectile,"gravity",9.8);
                entry.kinematics.groundHeight = phaseNumber(projectile,"ground_height",0.0);
                if (entry.kinematics.gravityMagnitude <= 0.0) throw std::runtime_error("Phase projectile gravity must be positive.");
            } else if (phase.contains("projectile")) throw std::runtime_error("projectile parameters require a projectile phase.");
            if (entry.kinematics.phenomenon == "inclined_plane_motion") {
                if (!phase.contains("inclined_plane") || !phase["inclined_plane"].is_object()) throw std::runtime_error("Inclined phase requires inclined_plane.");
                const auto& incline = phase["inclined_plane"];
                entry.kinematics.inclinedPlaneAngleDegrees = phaseNumber(incline,"angle_degrees",0.0,true);
                entry.kinematics.gravityMagnitude = phaseNumber(incline,"gravity",9.8);
                double angle = entry.kinematics.inclinedPlaneAngleDegrees;
                if (std::abs(angle) <= 1e-9 || std::abs(angle) >= 90.0 || entry.kinematics.gravityMagnitude <= 0.0)
                    throw std::runtime_error("Invalid inclined phase angle or gravity.");
            } else if (phase.contains("inclined_plane")) throw std::runtime_error("inclined_plane requires an inclined phase.");
        }
        if (entry.kinematics.phenomenon == "uniformly_accelerated_motion") phaseAcceleration(phase,entry);
        else if (phase.contains("acceleration")) throw std::runtime_error("Only MRUA phases accept acceleration.");
        sequence.push_back(entry);
    }
    // Reuse existing first-phase launch parsing and preserve the legacy external contract.
    json first = data;
    first.erase("motion_sequence");
    for (auto it=phases[0].begin();it!=phases[0].end();++it)
        if (it.key() != "until" && it.key() != "acceleration" && it.key() != "integrator") first[it.key()] = it.value();
    first["simulation"]["termination"] = phases[0]["until"];
    first["simulation"]["integrator"] = IntegratorFactory::name(sequence[0].integrator);
    if (sequence[0].kinematics.phenomenon == "uniformly_accelerated_motion") {
        if (sequence[0].accelerationAlongVelocity) {
            first["phenomenon"] = "uniform_motion";
            first["simulation"]["termination"] = {{"type","time"},{"value",1.0}};
        } else first["object"]["acceleration"] = phases[0]["acceleration"];
    }
    auto result = parseFirst(first.dump(),localTargets);
    if (!result.success) return result;
    KinematicsConfig initial = Kinematics(result.config).getConfiguration();
    initial.phenomenon = sequence[0].kinematics.phenomenon;
    sequence[0].kinematics = initial;
    result.config.phenomenon = initial.phenomenon;
    result.config.termination = sequence[0].until;
    result.config.motionSequence = sequence;
    result.config.explicitMotionSequence = true;
    result.config.integrator = defaultName;
    result.config.integratorType = *defaultType;
    if (localTargets) result.config.target = TrajectoryTarget();
    for (const MotionPhase& phase : sequence)
        if (phase.until.Type == TerminationType::Target && !(localTargets ? phase.target.has_value() : result.config.target.enabled))
            throw std::runtime_error("Phase until Target requires a target in that phase or a shared target.");
    PhysicalObject object(result.config.mass,result.config.initialPosition,result.config.initialVelocity);
    Simulation preview;
    std::string error;
    auto integrator = IntegratorFactory::create(*defaultType);
    if (!integrator) throw std::runtime_error("Unable to create selected integrator.");
    if (!preview.configureSequence(object,result.config,*integrator,error)) throw std::runtime_error(error);
    result.config.initialAcceleration = object.getAcceleration();
    return result;
}
}

// Parses and validates a simulation command written in JSON format.
CommandParseResult CommandParser::parse(const std::string& command) const {
    return parseCommand(command,false);
}

CommandParseResult CommandParser::parseCommand(const std::string& command,bool deferTargetHorizon) const {

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

        if (data.contains("motion_sequence"))
            return parseSequence(data,[&](const std::string& first,bool defer) { return parseCommand(first,defer); });

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

        bool circular = config.phenomenon == "uniform_circular_motion" ||
            config.phenomenon == "uniformly_accelerated_circular_motion";
        // Accepts the currently implemented physical phenomena.
        if (config.phenomenon != "free_fall" && config.phenomenon != "uniform_motion" &&
            config.phenomenon != "uniformly_accelerated_motion" && config.phenomenon != "inclined_plane_motion" && config.phenomenon != "projectile_motion" && !circular) {
            result.errorMessage = "Unknown phenomenon.";

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

        if (circular) {
            for (const char* key : {"position","velocity","acceleration"})
                if (object.contains(key)) throw std::runtime_error(std::string("object.")+key+" is derived automatically for circular motion.");
        }
        if (!circular) {
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


            config.initialPosition =
                Vector2D(
                    positionX,
                    positionY
                );

            // Uses zero initial velocity when no velocity is provided.
            double velocityX = 0.0;
            double velocityY = 0.0;

            // Uniform motion requires an explicit velocity object.
            if (config.phenomenon == "uniform_motion" && !object.contains("velocity")) {
                result.errorMessage = "uniform_motion requires an explicit object.velocity.";
                return result;
            }

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



            config.initialVelocity =
                Vector2D(
                    velocityX,
                    velocityY
                );

        }

        if (config.phenomenon == "inclined_plane_motion") {
            if (!data.contains("inclined_plane")) {
                result.errorMessage = "Missing required field: inclined_plane.";
                return result;
            }
            const json& inclinedPlane = data["inclined_plane"];
            if (!inclinedPlane.is_object()) {
                result.errorMessage = "inclined_plane must be a JSON object.";
                return result;
            }
            if (!inclinedPlane.contains("angle_degrees")) {
                result.errorMessage = "Missing required field: inclined_plane.angle_degrees.";
                return result;
            }
            if (!inclinedPlane["angle_degrees"].is_number()) {
                result.errorMessage = "inclined_plane.angle_degrees must be a number.";
                return result;
            }
            config.inclinedPlaneAngleDegrees = inclinedPlane["angle_degrees"].get<double>();
            const double epsilon = 1e-9;
            if (!std::isfinite(config.inclinedPlaneAngleDegrees) ||
                std::abs(config.inclinedPlaneAngleDegrees) <= epsilon ||
                std::abs(config.inclinedPlaneAngleDegrees) >= 90.0) {
                result.errorMessage = "inclined_plane.angle_degrees must be between -90 and 90 degrees and cannot be zero.";
                return result;
            }
            if (inclinedPlane.contains("gravity")) {
                if (!inclinedPlane["gravity"].is_number()) {
                    result.errorMessage = "inclined_plane.gravity must be a number.";
                    return result;
                }
                config.gravityMagnitude = inclinedPlane["gravity"].get<double>();
            }
            if (!std::isfinite(config.gravityMagnitude) || config.gravityMagnitude <= 0.0) {
                result.errorMessage = "inclined_plane.gravity must be greater than 0.";
                return result;
            }
            if (object.contains("acceleration")) {
                result.errorMessage = "object.acceleration is derived automatically for inclined_plane_motion.";
                return result;
            }

        }

        // Only accelerated rectilinear motion accepts prescribed acceleration.
        if (config.phenomenon != "uniformly_accelerated_motion" && object.contains("acceleration")) {
            result.errorMessage = "object.acceleration is only configurable for uniformly_accelerated_motion.";
            return result;
        }
        if (config.phenomenon == "uniformly_accelerated_motion") {
            if (!object.contains("acceleration")) {
                result.errorMessage = "Missing required field: object.acceleration.";
                return result;
            }
            const json& acceleration = object["acceleration"];
            if (!acceleration.is_object()) {
                result.errorMessage = "object.acceleration must be a JSON object.";
                return result;
            }
            double accelerationX = 0.0;
            double accelerationY = 0.0;
            if (acceleration.contains("x")) {
                if (!acceleration["x"].is_number()) {
                    result.errorMessage = "object.acceleration.x must be a number.";
                    return result;
                }
                accelerationX = acceleration["x"].get<double>();
            }
            if (acceleration.contains("y")) {
                if (!acceleration["y"].is_number()) {
                    result.errorMessage = "object.acceleration.y must be a number.";
                    return result;
                }
                accelerationY = acceleration["y"].get<double>();
            }
            config.initialAcceleration = Vector2D(accelerationX,accelerationY);

        }

        // Shared numeric parsing is reused by primary launch and target motion fields.
        // Primary and target commands share the same circular configuration parser.
        if (circular) {
            config.circularMotion.phenomenon = config.phenomenon;
            PhysicalObject state = parseCircular(data,config.circularMotion,config.mass);
            config.initialPosition = state.getPosition();
            config.initialVelocity = state.getVelocity();
            config.initialAcceleration = state.getAcceleration();
        }
        else if (data.contains("circular_motion"))
            throw std::runtime_error("circular_motion requires a circular phenomenon.");

        if (data.contains("target")) config.target = parseTarget(data["target"],data.contains("target_solution"));

        // Projectile launch preparation delegates all inverse equations to the solver.
        if (config.phenomenon == "projectile_motion") {
            json projectile = data.value("projectile",json::object());
            if (!projectile.is_object()) throw std::runtime_error("projectile must be a JSON object.");
            config.gravityMagnitude = number(projectile,"gravity",9.8);
            config.groundHeight = number(projectile,"ground_height",0.0);
            if (config.gravityMagnitude <= 0.0) throw std::runtime_error("projectile.gravity must be greater than 0.");
            if (!std::isfinite(config.initialPosition.x) || !std::isfinite(config.initialPosition.y) ||
                config.initialPosition.y < config.groundHeight-1e-9)
                throw std::runtime_error("Initial position must be finite and at or above ground_height.");

            if (config.target.enabled && !config.target.moving) {
                if (config.target.position.y < config.groundHeight-1e-9)
                    throw std::runtime_error("Target must be at or above ground_height.");
                bool targetStop = data.contains("simulation") && data["simulation"].is_object() &&
                    data["simulation"].contains("termination") && data["simulation"]["termination"].is_object() &&
                    data["simulation"]["termination"].value("type",json()) == "target";
                if ((!targetStop || data.contains("target_solution")) &&
                    std::abs(config.target.position.x-config.initialPosition.x) <= 1e-9)
                    throw std::runtime_error("Vertically aligned targets are not supported yet.");
            }

            bool vectorLaunch = object.contains("velocity");
            bool speedLaunch = projectile.contains("launch_speed") || projectile.contains("launch_angle_degrees");
            if (data.contains("target_solution")) {
                if (!config.target.enabled) throw std::runtime_error("target_solution requires a target.");
                if (vectorLaunch || speedLaunch) throw std::runtime_error("target_solution cannot be combined with a direct launch.");
                const json& request = data["target_solution"];
                if (!request.is_object() || !request.contains("mode") || !request["mode"].is_string())
                    throw std::runtime_error("target_solution.mode must be a string.");
                std::string mode = request["mode"].get<std::string>();
                if (mode == "known_speed") {
                    double speed = number(request,"speed",0.0,true);
                    if (speed <= 0.0) throw std::runtime_error("target_solution.speed must be greater than 0.");
                    auto solutions = ProjectileSolver::knownSpeed(config.initialPosition,config.target,config.gravityMagnitude,config.groundHeight,speed);
                    if (request.contains("arc")) {
                        if (!request["arc"].is_string()) throw std::runtime_error("target_solution.arc must be low or high.");
                        std::string arc = request["arc"].get<std::string>();
                        if (arc != "low" && arc != "high") throw std::runtime_error("target_solution.arc must be low or high.");
                        for (const auto& solution : solutions) if (solution.arc == arc) config.projectileSolution = solution;
                    }
                    else {
                        if (solutions.size() > 1) throw std::runtime_error("Two valid projectile solutions exist. Specify target_solution.arc as low or high.");
                        if (!solutions.empty()) config.projectileSolution = solutions.front();
                    }
                }
                else if (mode == "known_angle") {
                    double angle = number(request,"angle_degrees",0.0,true);
                    config.projectileSolution = ProjectileSolver::knownAngle(config.initialPosition,config.target,config.gravityMagnitude,config.groundHeight,angle);
                }
                else if (mode == "known_time") {
                    double time = number(request,"time",0.0,true);
                    if (time <= 0.0) throw std::runtime_error("target_solution.time must be greater than 0.");
                    config.projectileSolution = ProjectileSolver::knownTime(config.initialPosition,config.target,config.gravityMagnitude,config.groundHeight,time);
                }
                else throw std::runtime_error("Unknown target_solution mode.");
                if (!config.projectileSolution.valid) throw std::runtime_error("No physical projectile solution satisfies the requested arc, crossing, and ground constraints.");
                config.initialVelocity = config.projectileSolution.initialVelocity;
            }
            else if (speedLaunch) {
                if (vectorLaunch) throw std::runtime_error("Provide object.velocity or projectile launch speed/angle, not both.");
                double speed = number(projectile,"launch_speed",0.0,true);
                double angle = number(projectile,"launch_angle_degrees",0.0,true)*std::acos(-1.0)/180.0;
                if (speed <= 0.0) throw std::runtime_error("projectile.launch_speed must be greater than 0.");
                config.initialVelocity = Vector2D(speed*std::cos(angle),speed*std::sin(angle));
            }
            else if (!vectorLaunch) throw std::runtime_error("Projectile launch requires velocity, speed/angle, or target_solution.");
            if (!std::isfinite(config.initialVelocity.x) || !std::isfinite(config.initialVelocity.y))
                throw std::runtime_error("Projectile velocity must be finite.");
            PhysicalObject launch(config.mass,config.initialPosition,config.initialVelocity);
            config.projectileMetrics = Kinematics(config).getProjectileMetrics(launch);
            if (!std::isfinite(config.projectileMetrics.impactTime) || config.projectileMetrics.impactTime <= 0.0)
                throw std::runtime_error("Projectile launch must have a positive flight time.");
        }
        else if (data.contains("target_solution") || data.contains("projectile")) {
            throw std::runtime_error("Projectile launch and target options currently require projectile_motion.");
        }

        // Applies default simulation settings.
        config.termination = TerminationCondition{};
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

            // Validates the optional physical termination condition.
            if (simulation.contains("termination")) {
                const json& termination = simulation["termination"];

                if (!termination.is_object()) {
                    result.errorMessage = "simulation.termination must be an object.";
                    return result;
                }
                if (!termination.contains("type")) {
                    result.errorMessage = "Missing required field: simulation.termination.type.";
                    return result;
                }
                if (!termination["type"].is_string()) {
                    result.errorMessage = "simulation.termination.type must be a string.";
                    return result;
                }

                std::string type = termination["type"].get<std::string>();
                if (type == "natural") {
                    config.termination.Type = TerminationType::Natural;
                    config.termination.value = 0.0;
                }
                else if (type == "target") {
                    config.termination.Type = TerminationType::Target;
                    if (termination.contains("value")) throw std::runtime_error("Target termination uses max_time, not value.");
                    config.termination.maxTime = number(termination,"max_time",0.0,true);
                    if (config.termination.maxTime <= 0.0) throw std::runtime_error("simulation.termination.max_time must be greater than 0.");
                }
                else {
                    if (type == "time") {
                        config.termination.Type = TerminationType::Time;
                    }
                    else if (type == "distance") {
                        config.termination.Type = TerminationType::Distance;
                    }
                    else if (type == "angle") {
                        config.termination.Type = TerminationType::Angle;
                    }
                    else if (type == "speed") {
                        config.termination.Type = TerminationType::Speed;
                    }
                    else {
                        result.errorMessage = "Unknown termination type.";
                        return result;
                    }

                    if (!termination.contains("value")) {
                        result.errorMessage = "Missing required field: simulation.termination.value.";
                        return result;
                    }
                    if (!termination["value"].is_number()) {
                        result.errorMessage = "simulation.termination.value must be a number.";
                        return result;
                    }
                    config.termination.value = number(termination,"value",0.0,true);
                    if (config.termination.Type == TerminationType::Speed && config.termination.value < 0.0) {
                        result.errorMessage = "simulation.termination.value must be greater than or equal to 0 for speed termination.";
                        return result;
                    }
                    if (config.termination.Type != TerminationType::Speed && config.termination.value <= 0.0) {
                        result.errorMessage = "simulation.termination.value must be greater than 0.";
                        return result;
                    }
                }

                // Preserve the legacy free-fall error priority without duplicating its rules.
                if (config.phenomenon == "free_fall") {
                    PhysicalObject state(config.mass,config.initialPosition,config.initialVelocity);
                    std::string error = Kinematics(config).validate(state,config.termination);
                    if (!error.empty()) {
                        result.errorMessage = error;
                        return result;
                    }
                }
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

        auto integratorType = IntegratorFactory::parseType(config.integrator);
        if (!integratorType) {
            result.errorMessage =
                "Unknown integrator: " +
                config.integrator +
                ".";

            return result;
        }

        config.integratorType = *integratorType;

        if (config.playbackDuration <= 0.0) {
            result.errorMessage =
                "simulation.playback_duration must be greater than 0.";

            return result;
        }

        if (config.termination.Type == TerminationType::Target && !config.target.enabled) {
            result.errorMessage = "Target termination requires a target.";
            return result;
        }

        // Semantic scenario rules are independent of JSON representation.
        PhysicalObject objectState(config.mass,config.initialPosition,config.initialVelocity);
        objectState.setAcceleration(config.initialAcceleration);
        std::string validationError = Kinematics(config).validate(objectState,config.termination);
        if (!validationError.empty()) {
            result.errorMessage = validationError;
            return result;
        }
        if (config.projectileSolution.valid && config.termination.Type == TerminationType::Time &&
            config.termination.value+1e-8*std::max(1.0,config.projectileMetrics.impactTime) < config.projectileSolution.timeToTarget) {
            result.errorMessage = "Time termination occurs before the target_solution arrival.";
            return result;
        }

        if (config.target.moving && !deferTargetHorizon) {
            Kinematics targetModel(config.target.motion);
            PhysicalObject targetState(config.target.mass,config.target.position,config.target.velocity);
            targetState.setAcceleration(targetModel.getAcceleration(targetState));
            TerminationCondition horizon;
            horizon.Type = TerminationType::Target;
            horizon.maxTime = Kinematics(config).getExpectedDuration(objectState,config.termination);
            std::string error = targetModel.validate(targetState,horizon);
            if (!error.empty()) {
                result.errorMessage = "Invalid target motion: " + error;
                return result;
            }
        }

        // Legacy commands use the same phase data model with one local termination.
        config.motionSequence = {MotionPhase{Kinematics(config).getConfiguration(),config.termination,false,0.0,std::nullopt,config.integratorType}};
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