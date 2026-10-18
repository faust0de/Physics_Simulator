#pragma once

#include <raylib.h>
#include "PhysicalObject.hpp"
#include "Scale.hpp"
#include "TargetDetector.hpp"
#include "Kinematics.hpp"
// Handles graphical representation and playback controls using raylib.
class Renderer{
private:
    // Stores the rendering window width in pixels.
    int screenWidth;
    // Stores the rendering window height in pixels.
    int screenHeight;

    // Stores the number of screen pixels per physical world unit.
    double scaleValue;
    // Stores the characteristic world distance used to scale vector graphics.
    double referenceDistance;

    // Stores the screen position corresponding to the physical origin.
    Vector2 origin;

    // Tracks whether the user is dragging the timeline control.
    bool timelineDragging;
    // Stores the height reserved for playback controls in pixels.
    int controlPanelHeight;

    // Shares label occupancy across a group of generic measurement scales.
    void drawScale(const Scale& scale,std::vector<Rectangle>& occupiedLabels) const;

    // Calculates a horizontal label position from the vector direction and chosen side.
    Vector2 getVectorLabelPosition(
        const Vector2& start,
        const Vector2& end,
        int textWidth,
        int fontSize,
        float side
    ) const;

public:
    // Creates the raylib rendering window and initializes the scene mapping.
    Renderer(int width, int height, double scale,int panelHeight);

    // Closes the rendering window if it was initialized.
    ~Renderer();

    // Converts a position from physical world coordinates to screen coordinates.
    Vector2 toScreen(const Vector2D& worldPosition) const;

    // Reports whether the rendering window is unavailable or should close.
    bool shouldClose()const;

    // Begins a rendering frame and clears its background.
    void beginFrame()const;
    // Presents the completed rendering frame.
    void endFrame()const;

    // Draws consecutive stored numerical positions without reconstructing physics.
    void drawTrajectory(const std::vector<Vector2D>& points) const;
    // Draws a generic target at its physical radius with a visible center mark.
    void drawTarget(const TrajectoryTarget& target,bool hit) const;
    // Displays target observation independently of playback pause state.
    void drawTargetStatus(const TrajectoryTarget& target,const TargetHitResult& hit,double minimumDistance,bool finished) const;

    // Draws a geometric angle using a fixed pixel radius without scenario knowledge.
    void drawAngleGuide(const AngleGuide& guide) const;
    void drawCircularGuide(const CircularGuide& guide) const;

    // Draws the world coordinate axes in screen space.
    void drawAxes()const;

    // Draws the physical object at its current world position.
    void drawObject(const PhysicalObject& object)const;

    // Draws the velocity vector and its physical magnitude using a capped visual length.
    void drawVelocityVector(const PhysicalObject& object,double simulationDuration) const;

    // Displays physical time, position, and velocity.
    void drawSimulationInfo(const PhysicalObject& object,double simulationTime) const;

    // Draws a physical measurement scale with ticks and unit labels.
    void drawScale(const Scale& scale) const;

    // Draws the measurement scales supplied by the phenomenon.
    void drawScales(const std::vector<Scale>& scales) const;

    // Draws a button and reports a mouse press within its bounds.
    bool drawButton(Rectangle bounds,const char* text) const;

    // Draws the timeline and reports the physical time selected while dragging.
    bool drawTimeline(double currentTime,double duration,double& selectedTime);

    // Draws the background and border of the playback control panel.
    void drawControlPanel() const;

    // Returns the top screen coordinate of the control panel.
    int getControlPanelTop() const;

    // Fits physical scales and a reference position inside a uniformly scaled 2D viewport.
    void fitToScales(const std::vector<Scale>& scales,const Vector2D& fallbackPosition,const TrajectoryTarget& target = TrajectoryTarget(),const std::vector<Vector2D>& previewPoints = {});

    // Draws the acceleration vector and its physical magnitude using a capped visual length.
    void drawAccelerationVector(const PhysicalObject& object,const Vector2D& acceleration,double simulationDuration) const;
};


