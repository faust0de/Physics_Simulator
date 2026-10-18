#pragma once

#include "PhysicalObject.hpp"
#include "SimulationConfig.hpp"
#include "Kinematics.hpp"
#include "Integrator.hpp"
#include "TerminationCondition.hpp"
#include "TargetDetector.hpp"
#include <limits>
#include <optional>
#include <memory>

// Coordinates physical evolution and playback controls.
class Simulation {
private:
    // Stores the elapsed physical simulation time in seconds.
    double current_time;
    // Stores the ratio of physical time to real playback time.
    double timeScale;

    // Stores the physical state used to reconstruct reset and seek operations.
    PhysicalObject initialState;
    // Indicates whether a state has been saved for reset and seek.
    bool hasInitialState;
    // Tracks whether playback is paused.
    bool paused;
    // Stores the configured external termination condition.
    TerminationCondition termination;
    // Stores the total path length accumulated across physics steps.
    double traveledDistance;
    // Accumulates unsigned numerical angular increments in radians.
    double traveledAngle = 0.0;
    bool angularStopReached = false;
    // Stores the expected physical duration used for playback and external step limits.
    double physicalDuration;

    // Stores numerical positions, including the initial state and corrected impact endpoint.
    std::vector<Vector2D> trajectoryHistory;
    // Stores the optional target observed during numerical integration.
    TrajectoryTarget target;
    // Shared targets persist; local targets are replaced only when their phase activates.
    TrajectoryTarget sharedTarget;
    bool phaseLocalTargets = false;
    void initializeTarget(const TrajectoryTarget& config,const PhysicalObject& primary);
    void validateActiveTargetHorizon() const;
    // Stores the first crossing-qualified hit since reset.
    TargetHitResult targetHit;
    // Stores minimum center distance over numerical trajectory segments.
    double minimumTargetDistance = std::numeric_limits<double>::infinity();

    // Stores the current target body without a target trajectory history.
    PhysicalObject targetObject;
    // Stores its initial physical state for reset and seek.
    PhysicalObject initialTargetState;
    // Reuses the same kinematic model for optional target motion.
    std::optional<Kinematics> targetKinematics;
    // Freezes the target after its natural boundary without ending the primary motion.
    bool targetMotionFinished = false;

    static constexpr double physicsStep = 1.0/240.0;
    static constexpr std::size_t maximumPhases = maximumMotionPhases;
    static constexpr std::size_t maximumTransitionsPerStep = 16;
    static constexpr std::size_t maximumPreviewSteps = 2000000;
    std::vector<MotionPhase> motionSequence;
    std::optional<Kinematics> currentKinematics;
    std::shared_ptr<const Integrator> currentIntegrator;
    std::optional<MotionState> preflightFinalState;
    std::size_t currentPhaseIndex = 0;
    double phaseElapsedTime = 0.0;
    double phaseTraveledDistance = 0.0;
    double phaseTraveledAngle = 0.0;
    double phaseExpectedDuration = 0.0;
    double pendingPhysicalTime = 0.0;
    // Retains the preceding full physical step when seek displays a partial step.
    std::shared_ptr<Simulation> seekCheckpoint;
    PhysicalObject seekCheckpointObject;
    PhysicalObject phaseInitialState;
    TargetHitResult phaseTargetHit;
    KinematicsVisuals currentVisuals;
    ProjectileMetrics currentProjectileMetrics;
    bool explicitMotionSequence = false;
    bool sequenceComplete = false;
    bool targetMiss = false;
    bool preflightTargetMiss = false;
    std::vector<Vector2D> sequenceBounds;
    void activatePhase(std::size_t index,PhysicalObject& object);
    void consumeTime(PhysicalObject& object,const Integrator& integrator,double dt,double stopTime = std::numeric_limits<double>::infinity());
    bool phaseEnded(const PhysicalObject& object) const;
    void includePreviewPoint(const Vector2D& point,double radius = 0.0);
    void includeCurrentGuides();
    void runPreflight(const Integrator& integrator);

    // Advances both bodies using one physical step and localizes any terminating contact.
    double advance(PhysicalObject& object,const Kinematics& kinematics,const Integrator& integrator,double physicalDeltaTime,double stopTime = std::numeric_limits<double>::infinity());

    // Checks whether the configured external termination condition is reached.
    bool hasReachedTermination(const PhysicalObject& object) const;
public:
    // Initializes physical time and playback state before configuration.
    Simulation();
    bool configureSequence(PhysicalObject& object,const SimulationConfig& config,const Integrator& integrator,std::string& error);
    const Kinematics& getCurrentKinematics() const;
    const Integrator& getCurrentIntegrator() const;
    const PhysicalObject& getPhaseInitialState() const;
    IntegratorType getCurrentIntegratorType() const;
    const std::optional<MotionState>& getPreflightFinalState() const;
    const KinematicsVisuals& getCurrentVisuals() const;
    const ProjectileMetrics& getCurrentProjectileMetrics() const;
    const std::vector<Vector2D>& getSequenceBounds() const;
    std::size_t getCurrentPhaseIndex() const;
    std::size_t getPhaseCount() const;
    double getPhaseElapsedTime() const;
    double getPhaseTraveledDistance() const;
    double getPhaseTraveledAngle() const;
    const TargetHitResult& getPhaseTargetHit() const;
    bool isTargetMiss() const;
    bool isPreflightTargetMiss() const;
    bool isSequenceComplete() const;
    bool usesMotionSequence() const;

    // Coordinates one physics step and finalizes the kinematics when it ends.
    void step(PhysicalObject& object,const Kinematics& kinematics,const Integrator& integrator,double frameDeltaTime);

    // Returns the elapsed physical simulation time in seconds.
    double getCurrentTime() const;
    // Returns the ratio of physical time to real playback time.
    double getTimeScale() const;

    // Returns the estimated kinematics duration from the saved initial state.
    double getDuration(const Kinematics& kinematics) const;

    // Restores the saved initial physical state and resets physical time.
    void reset(PhysicalObject& object);

    // Saves the initial state and relates physical duration to real playback duration.
    void configureTimeScale(const PhysicalObject& object,const Kinematics& kinematics,const TerminationCondition& terminationCondition,double playbackDuration,IntegratorType integratorType = IntegratorType::Euler);

    // Reconstructs the physical state at the requested physical time.
    void seek(PhysicalObject& object,const Kinematics& kinematics,const Integrator& integrator,double targetTime);

    // Configures target observation without changing the termination condition.
    void configureTarget(const TrajectoryTarget& targetCondition);
    // Returns a visual target copy at the current simulated position.
    TrajectoryTarget getCurrentTarget() const;
    const TerminationCondition& getCurrentUntil() const;
    bool hasActiveTarget() const;
    const PhysicalObject* getActiveTargetObject() const;
    const TrajectoryTarget* getActiveTargetConfig() const;
    const TargetHitResult& getActiveTargetHit() const;
    // Returns camera-only target preview points over the primary physical horizon.
    std::vector<Vector2D> getTargetPreviewBounds() const;
    // Exposes the current target body for physical state inspection.
    const PhysicalObject& getTargetObject() const;
    // Reports whether target motion has reached its natural boundary.
    bool isTargetMotionFinished() const;
    // Returns accumulated primary path length, including partial contact steps.
    double getTraveledDistance() const;
    double getTraveledAngle() const;
    bool hasReachedAngularStop() const;
    // Returns the real numerical trajectory accumulated since reset.
    const std::vector<Vector2D>& getTrajectoryHistory() const;
    // Returns the first accepted target hit, if any.
    const TargetHitResult& getTargetHit() const;
    // Returns the minimum observed physical distance to the target center.
    double getMinimumTargetDistance() const;
    // Reports whether natural or external termination has been reached.
    bool isFinished(const PhysicalObject& object,const Kinematics& kinematics) const;

    // Resumes playback.
    void play();

    // Pauses playback.
    void pause();

    // Toggles the playback pause state.
    void togglePause();

    // Reports whether playback is paused.
    bool isPaused() const;
};