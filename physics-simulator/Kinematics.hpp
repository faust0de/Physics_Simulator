#pragma once
#include "PhysicalObject.hpp"
#include "Scale.hpp"
#include "TerminationCondition.hpp"
#include <string>
#include <vector>

struct SimulationConfig;

// Stores reusable kinematic parameters without application or target state.
struct KinematicsConfig {
    // Selects the existing scenario identifier.
    std::string phenomenon = "uniform_motion";
    // Prescribes acceleration for uniformly accelerated motion.
    Vector2D initialAcceleration;
    // Stores the inclined plane angle in degrees.
    double inclinedPlaneAngleDegrees = 0.0;
    // Stores the positive gravity magnitude.
    double gravityMagnitude = 9.8;
    // Stores the natural boundary height.
    double groundHeight = 0.0;
    Vector2D circularCenter;
    double circularRadius = 0.0;
    double initialAngleDegrees = 0.0;
    double initialAngularVelocity = 0.0;
    double angularAcceleration = 0.0;
};

// Stores analytical predictions from the launch state, independent of integration.
struct ProjectileMetrics {
    // Measures launch height relative to the configured ground.
    double initialHeightAboveGround = 0.0;
    // Measures additional ascent above the launch point.
    double riseAboveLaunch = 0.0;
    // Measures the highest altitude relative to the ground.
    double maximumHeightAboveGround = 0.0;
    // Stores the time of the highest point after launch.
    double apexTime = 0.0;
    // Stores the analytical highest point after launch.
    Vector2D apexPosition;
    // Stores the predicted time of ground impact.
    double impactTime = 0.0;
    // Stores the predicted impact position.
    Vector2D impactPosition;
    // Stores the unsigned horizontal displacement until impact.
    double horizontalRange = 0.0;
};

// Describes an angle geometrically without requiring scenario knowledge in rendering.
struct AngleGuide {
    // Enables the optional visual guide.
    bool visible = false;
    // Stores the lower endpoint of the slope in world coordinates.
    Vector2D vertex;
    // Points horizontally toward the interior of the reference triangle.
    Vector2D referenceDirection;
    // Points from the lower endpoint toward the upper endpoint.
    Vector2D direction;
    // Stores the unsigned geometric angle in degrees.
    double angleDegrees = 0.0;
    // Stores optional descriptive text for the angle.
    std::string label;
};

// Describes an ideal reference circle, independent of the numerical trajectory.
struct CircularGuide {
    bool visible = false;
    Vector2D center;
    double radius = 0.0;
    bool showCenter = true;
    bool showRadius = true;
    std::string radiusLabel = "R";
};

struct CircularMetrics {
    double configuredRadius = 0.0;
    double currentRadius = 0.0;
    double radialError = 0.0;
    double angleRadians = 0.0;
    double angleDegrees = 0.0;
    double angularVelocity = 0.0;
    double angularAcceleration = 0.0;
    double tangentialSpeed = 0.0;
    double centripetalAcceleration = 0.0;
    double tangentialAcceleration = 0.0;
};

// Groups physical measurement guides without prescribing drawing operations.
struct KinematicsVisuals {
    // Stores up to three nonduplicated linear reference scales.
    std::vector<Scale> scales;
    // Stores an optional geometric slope angle.
    AngleGuide angleGuide;
    CircularGuide circularGuide;
};

// Configures kinematic scenarios without parsing, integration, or rendering.
class Kinematics {
private:
    // Identifies the scenario after resolving its JSON identifier once.
    enum class KinematicsType { ConstantVelocity, ConstantAcceleration, VerticalFall, Slope, Projectile, UniformCircularMotion, UniformlyAcceleratedCircularMotion };
    // Stores the resolved scenario semantics.
    KinematicsType type;
    // Stores the constant acceleration derived from configuration.
    Vector2D acceleration;
    // Stores the plane tangent used for rounded incline input projection.
    Vector2D planeDirection;
    // Stores the gravity magnitude used in analytical references.
    double gravityMagnitude;
    // Stores the natural ground level for falling scenarios.
    double groundHeight;
    // Retains circular parameters without constraining the integrated state.
    KinematicsConfig circularConfig;
    struct AngularMotionInfo {
        double initialAngularSpeed = 0.0;
        double accelerationAlongMotion = 0.0;
    };
    AngularMotionInfo getAngularMotionInfo() const;
    double getCircularExpectedDuration(const TerminationCondition& termination) const;
    std::string validateCircularMotion(const PhysicalObject& object,const TerminationCondition& termination) const;

    // Describes one scalar frame for rectilinear analytical references.
    struct LinearMotionInfo {
        // Stores the direction of forward motion before reversal.
        Vector2D direction;
        // Stores the nonnegative initial speed along that direction.
        double initialSpeed = 0.0;
        // Stores the signed acceleration along the direction of motion.
        double accelerationAlongMotion = 0.0;
        // Indicates a defined motion direction or stationary state.
        bool valid = false;
    };
    // Maps existing JSON identifiers to an internal enum.
    static KinematicsType resolveType(const std::string& name);
    // Shares rectilinear frame calculations across the three linear scenarios.
    LinearMotionInfo getLinearMotionInfo(const PhysicalObject& object) const;
    // Solves the existing stable scalar duration equations.
    double getAcceleratedDuration(const PhysicalObject& object,const TerminationCondition& termination) const;
    // Checks linear reachability without allowing direction reversal.
    std::string validateLinearMotion(const PhysicalObject& object,const TerminationCondition& termination) const;
    // Decomposes a linear path into nonduplicated path and projection rulers.
    std::vector<Scale> buildLinearScales(const Vector2D& start,const Vector2D& end,double pathDistance) const;
    // Builds only horizontal and height references for a curved path.
    std::vector<Scale> buildProjectileScales(const PhysicalObject& object,const TerminationCondition& termination) const;
    // Anchors a slope angle at its lower endpoint, independently of motion direction.
    AngleGuide buildInclinedAngleGuide(const Vector2D& start,const Vector2D& end) const;
public:
    KinematicsConfig getConfiguration() const;
    static KinematicsConfig inheritCircularConfig(KinematicsConfig config,const PhysicalObject& object,const Kinematics* previous);
    bool isCircular() const;
    CircularMetrics getCircularMetrics(const PhysicalObject& object) const;
    PhysicalObject calculateCircularInitialState(double mass) const;
    double getAngularIncrement(const Vector2D& previous,const Vector2D& current) const;
    bool hasAngularReversal(const PhysicalObject& object) const;
    // Resolves the scenario and its acceleration parameters.
    explicit Kinematics(const SimulationConfig& config);
    // Configures a reusable body independently of the application configuration.
    explicit Kinematics(const KinematicsConfig& config);
    // Locates a natural boundary along a numerical segment, without analytical correction.
    double getNaturalBoundaryFraction(const PhysicalObject& previous,const PhysicalObject& current) const;
    // Predicts camera-only bounds without generating scales, angle guides, or a trail.
    std::vector<Vector2D> getPreviewBounds(const PhysicalObject& object,double horizon) const;
    // Provides acceleration directly to the numerical coordinator.
    Vector2D getAcceleration(const PhysicalObject& object) const;
    Vector2D getAcceleration(const Vector2D& position,const Vector2D& velocity) const;
    // Checks only scenario-specific natural termination.
    bool isFinished(const PhysicalObject& object) const;
    // Clamps ground penetration while preserving numerical impact velocity and X.
    void finish(PhysicalObject& object) const;
    // Predicts duration without replacing numerical termination conditions.
    double getExpectedDuration(const PhysicalObject& object,const TerminationCondition& termination) const;
    // Returns scenario-independent visual scale and angle descriptions.
    KinematicsVisuals getVisuals(const PhysicalObject& object,const TerminationCondition& termination) const;
    // Calculates analytical launch metrics without evolving the physical state.
    ProjectileMetrics getProjectileMetrics(const PhysicalObject& object) const;
    // Reports applicability of projectile reference metrics.
    bool isProjectile() const;
    // Returns a semantic validation error or an empty string for a valid scenario.
    std::string validate(const PhysicalObject& object,const TerminationCondition& termination) const;
};
