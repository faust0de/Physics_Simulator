# Commented project source files

All 17 source files are reproduced in full, each in a separate section. Executable code is unchanged. All added comments use single-line English // comments.

## EulerIntegrator.cpp

```cpp
#include "EulerIntegrator.hpp"

// Updates position and velocity using explicit Euler integration and clears accumulated forces.
void EulerIntegrator::integrate(PhysicalObject& object,double deltatime)const{
    if(object.getMass()<=0||deltatime<=0){
        return;
    }
    //Gets actual values
    Vector2D position= object.getPosition();
    Vector2D velocity= object.getVelocity();
    Vector2D acceleration= object.getAcceleration();
    // Converts the accumulated force to acceleration using Newton's second law.
    Vector2D ForceAcceleration=object.getAccumulatedForce()/object.getMass();
    Vector2D TotalAcceleration= acceleration+ForceAcceleration;

    //Explicit Euler Method
    Vector2D NewPosition= velocity*deltatime + position;
    Vector2D NewVelocity= velocity + TotalAcceleration*deltatime;
    
    //Update State
    object.setPosition(NewPosition);
    object.setVelocity(NewVelocity);

    object.clearForces();
}
```

## EulerIntegrator.hpp

```cpp
#pragma once

#include "Integrator.hpp"

// Implements the explicit Euler numerical integration strategy.
class EulerIntegrator:public Integrator{
public:
    // Updates position and velocity using explicit Euler integration and clears accumulated forces.
    void integrate(PhysicalObject& object, double deltatime)const override;
};
```

## FreeFall.cpp

```cpp
#include"FreeFall.hpp"
#include <cmath>

// Applies the gravitational force to the object.
void FreeFall::apply(PhysicalObject& object, double /*deltatime*/)const{
    double mass=object.getMass();
    Vector2D gravityForce(0.0,mass*gravity);
    object.applyForce(gravityForce);
}

// Checks whether the object has reached or passed ground level.
bool FreeFall::isFinished(const PhysicalObject& object) const {
    return object.getPosition().y <= groundHeight;
}

// Places the object exactly at ground level while preserving impact velocity.
void FreeFall::finish(PhysicalObject& object) const {
    Vector2D position =object.getPosition();

    // Correct the numerical overshoot
    position.y = groundHeight;

    object.setPosition(position);

    // Keep the velocity at the instant of impact.
    object.clearForces();
}

// Describes a height scale between ground level and the supplied object position.
std::vector<Scale> FreeFall::getScales(const PhysicalObject& object) const {

    double initialHeight =object.getPosition().y;

    double rulerX =-initialHeight * 0.20;

    Scale heightScale(Vector2D(rulerX,groundHeight),Vector2D(rulerX,initialHeight),groundHeight,initialHeight,5,"m");

    return {
        heightScale
    };
}

// Estimates the fall duration from the supplied height assuming release from rest.
double FreeFall::getExpectedDuration(const PhysicalObject& object) const {
    double initialHeight =object.getPosition().y - groundHeight;

    if (initialHeight <= 0.0) {
        return 0.0;
    }

    return std::sqrt((2.0 * initialHeight) /(-gravity));
}

// Returns the constant vertical gravitational acceleration.
Vector2D FreeFall::getAcceleration(const PhysicalObject& object) const {
    return Vector2D(0.0,gravity);
}
```

## FreeFall.hpp

```cpp
#pragma once

#include "PhysicalPhenomenon.hpp"

// Models vertical motion under constant gravity.
class FreeFall:public PhysicalPhenomenon{
private:
    // Stores the constant vertical gravitational acceleration in meters per second squared.
    const float gravity=-9.8;
    // Stores the ground position in world meters.
    const double groundHeight = 0.0;
public:
    // Applies the gravitational force to the object.
    void apply(PhysicalObject& object, double deltatime)const override;

    // Checks whether the object has reached or passed ground level.
    bool isFinished(const PhysicalObject& object) const override;

    // Places the object exactly at ground level while preserving impact velocity.
    void finish(PhysicalObject& object) const override;

    // Describes a height scale between ground level and the supplied object position.
    std::vector<Scale> getScales(const PhysicalObject& object) const override;

    // Estimates the fall duration from the supplied height assuming release from rest.
    double getExpectedDuration(const PhysicalObject& object) const override;

    // Returns the constant vertical gravitational acceleration.
    Vector2D getAcceleration(const PhysicalObject& object) const override;
};
```

## Integrator.hpp

```cpp
#pragma once
#include "PhysicalObject.hpp"

// Defines the abstract interface for numerical integration strategies.
class Integrator{
public:
    // Allows derived integrators to be destroyed through a base class pointer.
    virtual ~Integrator()=default;

    // Defines how an integrator updates an object over a time step.
    virtual void integrate(PhysicalObject& object, double deltatime)const=0;
};
```

## PhysicalObject.cpp

```cpp
#include "PhysicalObject.hpp"

// Initializes an object with the specified mass, position, and velocity.
PhysicalObject::PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity)
    : mass(massValue), velocity(initialVelocity), position(initialPosition) {}

// Adds a force to the accumulated force.
void PhysicalObject::applyForce(const Vector2D& force) {
    // Combines forces until the numerical integrator clears the step total.
    accumulatedForce += force;
}

// Returns the acceleration set for the object.
Vector2D PhysicalObject::getAcceleration() const{
    return acceleration;
}

// Returns the velocity of the object.
Vector2D PhysicalObject::getVelocity()const{
    return velocity;
}

// Returns the position of the object.
Vector2D PhysicalObject::getPosition()const{
    return position;
}

// Returns the mass of the object.
double PhysicalObject:: getMass()const {
    return mass;
}

// Sets the acceleration of the object.
void PhysicalObject:: setAcceleration(const Vector2D& NewAcceleration){
    acceleration=NewAcceleration;
}

// Sets the position of the object.
void PhysicalObject:: setPosition(const Vector2D& NewPosition){
    position=NewPosition;
}

// Sets the velocity of the object.
void PhysicalObject:: setVelocity(const Vector2D& NewVelocity){
    velocity=NewVelocity;
}

//Returns the AccumulatedForce of the object
Vector2D PhysicalObject:: getAccumulatedForce()const{
    return accumulatedForce;
}

//Sets the AccumulatedForce in (0,0)
void PhysicalObject:: clearForces(){
    accumulatedForce.x=0;
    accumulatedForce.y=0;
}
```

## PhysicalObject.hpp

```cpp
#pragma once

#include "Vector2D.hpp"

// Stores the physical state of an object with mass and motion.
class PhysicalObject {
private:
    // Stores the mass in kilograms.
    double mass=1.0;
    // Stores the velocity in meters per second.
    Vector2D velocity;
    // Stores the world position in meters.
    Vector2D position;
    // Stores the prescribed acceleration in meters per second squared.
    Vector2D acceleration;
    // Stores the sum of forces applied during the current physics step.
    Vector2D accumulatedForce;
public:
    // Initializes an object with unit mass and zero vectors.
    PhysicalObject() = default;
    // Initializes an object with the specified mass, position, and velocity.
    PhysicalObject(double massValue, const Vector2D& initialPosition, const Vector2D& initialVelocity);
    // Adds a force to the accumulated force.
    void applyForce(const Vector2D& force);
    // Returns the position of the object.
    Vector2D getPosition() const;
    // Returns the velocity of the object.
    Vector2D getVelocity() const;
    // Returns the acceleration set for the object.
    Vector2D getAcceleration() const;
    // Returns the mass of the object.
    double getMass()const;
    // Returns the AccumulatedForce of the object
    Vector2D getAccumulatedForce() const;
    
    // Set the AcumulatedForce in (0,0)
    void clearForces();

    // Sets the position of the object.
    void setPosition(const Vector2D& newPosition);
    // Sets the velocity of the object.
    void setVelocity(const Vector2D& newVelocity);
    // Sets the acceleration of the object.
    void setAcceleration(const Vector2D& newAcceleration);
};
```

## PhysicalPhenomenon.hpp

```cpp
#pragma once
#include "PhysicalObject.hpp"
#include "Scale.hpp"
// Defines the abstract interface for physical behavior with optional default hooks.
class PhysicalPhenomenon{
public:
    // Virtual destructor for derived classes
    virtual ~PhysicalPhenomenon()=default;

    // Must be implemented by derived phenomena.
    // Applies the physical phenomenon to the object for one physics step.
    virtual void apply(PhysicalObject& object, double deltatime)const=0;

    // Defaults to an unfinished phenomenon unless a derived class provides a condition.
    virtual bool isFinished(const PhysicalObject& object) const{
        return false;
    }

    // Provides an optional finalization hook that does nothing by default.
    virtual void finish(PhysicalObject& object) const{    
    }
    // Provides optional measurement scales, returning none by default.
    virtual std::vector<Scale> getScales(const PhysicalObject& object) const {
        return {};
    }

    // Provides an optional duration estimate, returning zero by default.
    virtual double getExpectedDuration(const PhysicalObject& object) const {
        return 0.0;
    }

    // Provides the phenomenon acceleration, returning a zero vector by default.
    virtual Vector2D getAcceleration(const PhysicalObject& object) const {
        return Vector2D();
    }
};
```

## Renderer.cpp

```cpp
#include "Renderer.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

#include <iostream>
#include "Renderer.hpp"

// Creates the raylib rendering window and initializes the scene mapping.
Renderer::Renderer(
    int width,
    int height,
    double scale,
    int panelHeight
)
    : screenWidth(width),
      screenHeight(height),
      controlPanelHeight(panelHeight),
      scaleValue(scale),
      referenceDistance(1.0),
      timelineDragging(false) {

    std::cerr << "Renderer: llamando InitWindow...\n";

    InitWindow(
        screenWidth,
        screenHeight,
        "Physics Simulator"
    );

    std::cerr
        << "Renderer: IsWindowReady = "
        << IsWindowReady()
        << '\n';

    if (!IsWindowReady()) {
        std::cerr
            << "ERROR: raylib no pudo inicializar la ventana\n";
        return;
    }

    SetTargetFPS(60);

    origin = {
        static_cast<float>(screenWidth) / 2.0f,
        static_cast<float>(
            screenHeight -
            controlPanelHeight -
            40
        )
    };
}

// Closes the rendering window if it was initialized.
Renderer::~Renderer() {
    if (IsWindowReady()) {
        CloseWindow();
    }
}

// Reports whether the rendering window is unavailable or should close.
bool Renderer::shouldClose() const {
    if (!IsWindowReady()) {
        return true;
    }
    return WindowShouldClose();
}

// Begins a rendering frame and clears its background.
void Renderer:: beginFrame()const{
    BeginDrawing();
    ClearBackground(RAYWHITE);
}

// Presents the completed rendering frame.
void Renderer:: endFrame()const{
    EndDrawing();
}

// Converts physical coordinates in meters to screen coordinates in pixels.
Vector2 Renderer::toScreen(const Vector2D& worldPosition) const {

    float screenX =origin.x + static_cast<float>(worldPosition.x * scaleValue);

    float screenY =origin.y - static_cast<float>(worldPosition.y * scaleValue);

    return Vector2{screenX, screenY};
}

// Draws the X and Y axes.
void Renderer::drawAxes() const {

    // X axis
    DrawLine(
        0,
        static_cast<int>(origin.y),
        screenWidth,
        static_cast<int>(origin.y),
        BLACK
    );

    // Y axis
    DrawLine(
        static_cast<int>(origin.x),
        0,
        static_cast<int>(origin.x),
        screenHeight,
        BLACK
    );
}

// Draws a physical object at its current position.
void Renderer::drawObject(const PhysicalObject& object) const {

    Vector2D worldPosition = object.getPosition();

    Vector2 screenPosition =toScreen(worldPosition);

    DrawCircleV(
        screenPosition,
        10.0f,
        BLUE
    );
}

// Calculates a horizontal label position from the vector direction and chosen side.
Vector2 Renderer::getVectorLabelPosition(
    const Vector2& start,
    const Vector2& end,
    int textWidth,
    int fontSize,
    float side
) const {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    const float forwardOffset = 12.0f;
    const float sideOffset = 28.0f;

    if (length <= 0.001f) {
        // A zero vector has no direction; keep its horizontal label nearby.
        return Vector2{
            end.x + side * (sideOffset + textWidth / 2.0f) - textWidth / 2.0f,
            end.y - fontSize / 2.0f
        };
    }

    const float directionX = dx / length;
    const float directionY = dy / length;
    const float normalX = -directionY;
    const float normalY = directionX;

    // Include the text's extent along the normal so wide labels stay
    // entirely on their chosen side, even for inclined vectors.
    const float labelExtent =
        std::abs(normalX) * textWidth / 2.0f +
        std::abs(normalY) * fontSize / 2.0f;
    const float normalOffset = sideOffset + labelExtent;

    return Vector2{
        end.x + directionX * forwardOffset + normalX * normalOffset * side
            - textWidth / 2.0f,
        end.y + directionY * forwardOffset + normalY * normalOffset * side
            - fontSize / 2.0f
    };
}

// Draws the velocity vector and its physical magnitude using a capped visual length.
void Renderer::drawVelocityVector(const PhysicalObject& object,double simulationDuration) const {

    Vector2D position =object.getPosition();

    Vector2D velocity =object.getVelocity();

    double speed =velocity.magnitude();

    if (simulationDuration <= 0.0 ||referenceDistance <= 0.0) {
        return;
    }

    const int fontSize = 16;
    const char* velocityText = TextFormat("v = %.2f m/s", speed);
    const int velocityTextWidth = MeasureText(velocityText, fontSize);

    // Fraction of the simulation duration used
    // to convert velocity into a visual vector.
    const double vectorTimeFraction = 0.15;

    Vector2D visualVector =velocity *(simulationDuration *vectorTimeFraction);

    double visualLength =visualVector.magnitude();

    // The arrow can occupy at most 30%
    // of the characteristic distance.
    const double maxVectorFraction = 0.30;

    double maxWorldLength =referenceDistance *maxVectorFraction;

    if (visualLength > maxWorldLength) {

        visualVector =visualVector.normalized() *maxWorldLength;}

    Vector2D endPosition =position + visualVector;

    Vector2 start =toScreen(position);

    Vector2 end =toScreen(endPosition);

    float dx =end.x - start.x;

    float dy =end.y - start.y;

    float length =std::sqrt(dx * dx +dy * dy);

    const Vector2 velocityLabelPosition = getVectorLabelPosition(
        start, end, velocityTextWidth, fontSize, -1.0f
    );

    if (length <= 0.001f) {
        DrawText(velocityText, static_cast<int>(velocityLabelPosition.x),
            static_cast<int>(velocityLabelPosition.y), fontSize, RED);
        return;
    }

    // Main vector line
    DrawLineV(
        start,
        end,
        RED
    );

    // Normalize screen direction
    dx /= length;
    dy /= length;

    // Perpendicular direction
    float perpendicularX = -dy;
    float perpendicularY = dx;

    const float arrowLength = 12.0f;
    const float arrowWidth = 6.0f;

    Vector2 leftPoint = {
        end.x
            - dx * arrowLength
            + perpendicularX * arrowWidth,

        end.y
            - dy * arrowLength
            + perpendicularY * arrowWidth
    };

    Vector2 rightPoint = {
        end.x
            - dx * arrowLength
            - perpendicularX * arrowWidth,

        end.y
            - dy * arrowLength
            - perpendicularY * arrowWidth
    };

    DrawLineV(
        end,
        leftPoint,
        RED
    );

    DrawLineV(
        end,
        rightPoint,
        RED
    );

    DrawText(
        velocityText,
        static_cast<int>(velocityLabelPosition.x),
        static_cast<int>(velocityLabelPosition.y),
        fontSize,
        RED
    );
}

// Draws the acceleration vector and its physical magnitude using a capped visual length.
void Renderer::drawAccelerationVector(
    const PhysicalObject& object,
    const Vector2D& acceleration,
    double simulationDuration
) const {

    Vector2D position =
        object.getPosition();

    double accelerationMagnitude =
        acceleration.magnitude();

    if (
        accelerationMagnitude <= 0.000001 ||
        simulationDuration <= 0.0 ||
        referenceDistance <= 0.0
    ) {
        return;
    }

    // Converts acceleration into a visually useful
    // displacement-like vector.
    const double accelerationTimeFraction = 0.08;

    Vector2D visualVector =
        acceleration *
        (
            simulationDuration *
            simulationDuration *
            accelerationTimeFraction
        );

    double visualLength =
        visualVector.magnitude();

    // Maximum 20% of the characteristic
    // simulation distance.
    const double maxAccelerationFraction = 0.20;

    double maxWorldLength =
        referenceDistance *
        maxAccelerationFraction;

    if (visualLength > maxWorldLength) {

        double factor =
            maxWorldLength /
            visualLength;

        visualVector =
            visualVector * factor;
    }

    Vector2D endPosition =
        position + visualVector;

    Vector2 start =
        toScreen(position);

    Vector2 end =
        toScreen(endPosition);

    float dx =
        end.x - start.x;

    float dy =
        end.y - start.y;

    float length =
        std::sqrt(
            dx * dx +
            dy * dy
        );

    if (length <= 0.001f) {
        return;
    }

    // Main acceleration vector
    DrawLineV(
        start,
        end,
        DARKGREEN
    );

    dx /= length;
    dy /= length;

    float perpendicularX = -dy;
    float perpendicularY = dx;

    const float arrowLength = 12.0f;
    const float arrowWidth = 6.0f;

    Vector2 leftPoint = {
        end.x
            - dx * arrowLength
            + perpendicularX * arrowWidth,

        end.y
            - dy * arrowLength
            + perpendicularY * arrowWidth
    };

    Vector2 rightPoint = {
        end.x
            - dx * arrowLength
            - perpendicularX * arrowWidth,

        end.y
            - dy * arrowLength
            - perpendicularY * arrowWidth
    };

    DrawLineV(
        end,
        leftPoint,
        DARKGREEN
    );

    DrawLineV(
        end,
        rightPoint,
        DARKGREEN
    );

    const int fontSize = 16;
    const char* accelerationText = TextFormat(
        "a = %.2f m/s^2", accelerationMagnitude
    );
    const int accelerationTextWidth = MeasureText(accelerationText, fontSize);
    const Vector2 accelerationLabelPosition = getVectorLabelPosition(
        start, end, accelerationTextWidth, fontSize, 1.0f
    );

    DrawText(
        accelerationText,
        static_cast<int>(accelerationLabelPosition.x),
        static_cast<int>(accelerationLabelPosition.y),
        fontSize,
        DARKGREEN
    );
}

// Displays physical time, position, and velocity.
void Renderer::drawSimulationInfo(const PhysicalObject& object,double simulationTime) const {
    Vector2D position =
        object.getPosition();

    Vector2D velocity =
        object.getVelocity();

    DrawText(
        TextFormat(
            "Time: %.2f s",
            simulationTime
        ),
        20,
        20,
        20,
        BLACK
    );

    DrawText(
        TextFormat(
            "Position: (%.2f, %.2f) m",
            position.x,
            position.y
        ),
        20,
        50,
        20,
        BLACK
    );

    DrawText(
        TextFormat(
            "Velocity: (%.2f, %.2f) m/s",
            velocity.x,
            velocity.y
        ),
        20,
        80,
        20,
        BLACK
    );
}

// Draws a physical measurement scale with ticks and unit labels.
void Renderer::drawScale(const Scale& scale) const {
    Vector2D startWorld = scale.getStart();

    Vector2D endWorld = scale.getEnd();

    Vector2 start =toScreen(startWorld);

    Vector2 end =toScreen(endWorld);

    // Main scale line
    DrawLineV(start,end,DARKGRAY);

    float dx = end.x - start.x;
    float dy = end.y - start.y;

    float length = std::sqrt(dx * dx + dy * dy);

    if (length == 0.0f) {
        return;
    }

    // Perpendicular unit vector
    float normalX = -dy / length;
    float normalY = dx / length;

    int divisions = scale.getDivisions();

    for (int i = 0; i <= divisions; ++i) {

        float t =static_cast<float>(i) / static_cast<float>(divisions);

        float pointX = start.x + dx * t;

        float pointY = start.y + dy * t;

        const float tickSize = 6.0f;

        Vector2 tickStart = {pointX - normalX * tickSize,pointY - normalY * tickSize};

        Vector2 tickEnd = {pointX + normalX * tickSize,pointY + normalY * tickSize};

        DrawLineV(tickStart,tickEnd,DARKGRAY);

        double value =scale.getMinValue() + (scale.getMaxValue() -scale.getMinValue()) * t;

        DrawText(TextFormat(
                "%.1f %s",
                value,
                scale.getUnit().c_str()
            ),
            static_cast<int>(
                pointX + normalX * 15.0f
            ),
            static_cast<int>(
                pointY + normalY * 15.0f
            ),
            18,
            DARKGRAY
        );
    }
}

// Draws the measurement scales supplied by the phenomenon.
void Renderer::drawScales(const std::vector<Scale>& scales) const {
    for (const Scale& scale : scales) {
        drawScale(scale);
    }
}

// Draws a button and reports a mouse press within its bounds.
bool Renderer::drawButton(Rectangle bounds,const char* text) const {
    Vector2 mousePosition =GetMousePosition();

    bool hovered =CheckCollisionPointRec(mousePosition,bounds);

    Color buttonColor =hovered ? GRAY : LIGHTGRAY;

    DrawRectangleRec(bounds,buttonColor);

    DrawRectangleLinesEx(bounds,2.0f,DARKGRAY);

    const int fontSize = 20;

    int textWidth =MeasureText(text,fontSize);

    int textX =
        static_cast<int>(
            bounds.x +
            (bounds.width - textWidth) / 2.0f
        );

    int textY =
        static_cast<int>(
            bounds.y +
            (bounds.height - fontSize) / 2.0f
        );

    DrawText(
        text,
        textX,
        textY,
        fontSize,
        BLACK
    );

    return
        hovered &&
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        );
}

// Draws the timeline and reports the physical time selected while dragging.
bool Renderer::drawTimeline(double currentTime,double duration,double& selectedTime) {

    if (duration <= 0.0) {
        return false;
    }

    const float panelTop =static_cast<float>(screenHeight -controlPanelHeight);

    const float barX = 100.0f;

    const float barY =panelTop + 35.0f;

    const float barWidth =static_cast<float>(screenWidth) -200.0f;

    const float barThickness = 6.0f;

    // Larger invisible area to make clicking easier.
    Rectangle hitBox = {barX,barY - 12.0f,barWidth,24.0f};

    Vector2 mousePosition =GetMousePosition();

    // Start dragging.
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&CheckCollisionPointRec(mousePosition,hitBox)) {
        timelineDragging = true;
    }

    bool seeking = false;

    double displayTime = currentTime;

    if (timelineDragging) {
        float mouseX =std::clamp(
                mousePosition.x,
                barX,
                barX + barWidth
            );

        float progress =(mouseX - barX) /barWidth;

        selectedTime =progress * duration;

        displayTime =selectedTime;

        seeking = true;

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            timelineDragging = false;
        }
    }

    float progress =
        static_cast<float>(
            displayTime / duration
        );

    progress =
        std::clamp(
            progress,
            0.0f,
            1.0f
        );

    float knobX =
        barX +
        barWidth * progress;

    // Background timeline
    DrawLineEx(
        Vector2{barX, barY},
        Vector2{
            barX + barWidth,
            barY
        },
        barThickness,
        LIGHTGRAY
    );

    // Completed part
    DrawLineEx(
        Vector2{barX, barY},
        Vector2{
            knobX,
            barY
        },
        barThickness,
        RED
    );

    // Draggable circle
    DrawCircleV(
        Vector2{
            knobX,
            barY
        },
        9.0f,
        RED
    );

    // Current time
    DrawText(
        TextFormat(
            "%.2f s",
            displayTime
        ),
        static_cast<int>(barX),
        static_cast<int>(barY - 35.0f),
        18,
        DARKGRAY
    );

    // Total duration
    const char* durationText =
        TextFormat(
            "%.2f s",
            duration
        );

    int durationTextWidth =
        MeasureText(
            durationText,
            18
        );

    DrawText(
        durationText,
        static_cast<int>(
            barX +
            barWidth -
            durationTextWidth
        ),
        static_cast<int>(
            barY - 35.0f
        ),
        18,
        DARKGRAY
    );

    return seeking;
}

// Draws the background and border of the playback control panel.
void Renderer::drawControlPanel() const {

    int panelY =screenHeight -controlPanelHeight;

    DrawRectangle(0,panelY,screenWidth,controlPanelHeight,Color{235, 235, 235, 255});

    DrawLine(0,panelY,screenWidth,panelY,DARKGRAY);
}

// Returns the top screen coordinate of the control panel.
int Renderer::getControlPanelTop() const {
    return screenHeight - controlPanelHeight;
}

// Adjusts the world-to-screen scale to fit the vertical extent of the physical scales.
void Renderer::fitToScales(const std::vector<Scale>& scales) {

    if (scales.empty()) {
        return;
    }

    referenceDistance = 0.0;

    double minY =std::numeric_limits<double>::max();

    double maxY =std::numeric_limits<double>::lowest();

    for (const Scale& scale : scales) {

        Vector2D start =scale.getStart();

        Vector2D end =scale.getEnd();
        
        double scaleDistance =(end - start).magnitude();

        referenceDistance =std::max(referenceDistance,scaleDistance);

        minY =std::min(minY,std::min(start.y, end.y));

        maxY =std::max(maxY,std::max(start.y, end.y));
    }

    double worldHeight =maxY - minY;

    if (worldHeight <= 0.0) {
        return;
    }

    const double topMargin = 60.0;

    double availableHeight =origin.y - topMargin;

    double newScale =availableHeight /worldHeight;

    // Prevent excessive zoom for very small distances.
    const double maxScale = 60.0;

    scaleValue =std::min(newScale,maxScale);
}
```

## Renderer.hpp

```cpp
#pragma once

#include <raylib.h>
#include "PhysicalObject.hpp"
#include "Scale.hpp"
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

    // Adjusts the world-to-screen scale to fit the vertical extent of the physical scales.
    void fitToScales(const std::vector<Scale>& scales);

    // Draws the acceleration vector and its physical magnitude using a capped visual length.
    void drawAccelerationVector(const PhysicalObject& object,const Vector2D& acceleration,double simulationDuration) const;
};
```

## Scale.cpp

```cpp
#include "Scale.hpp"

// Creates a world-space measurement scale with a range, intervals, and unit.
Scale::Scale(const Vector2D& startPoint,const Vector2D& endPoint,double minimum,double maximum,int divisionCount,const std::string& unitValue)
    : start(startPoint),end(endPoint),minValue(minimum),maxValue(maximum),divisions(divisionCount),unit(unitValue) {
}

// Returns the starting point in world coordinates.
Vector2D Scale::getStart() const {
    return start;
}

// Returns the ending point in world coordinates.
Vector2D Scale::getEnd() const {
    return end;
}

// Returns the measurement value at the starting point.
double Scale::getMinValue() const {
    return minValue;
}

// Returns the measurement value at the ending point.
double Scale::getMaxValue() const {
    return maxValue;
}

// Returns the number of labeled intervals.
int Scale::getDivisions() const {
    return divisions;
}

// Returns the measurement unit used by the labels.
std::string Scale::getUnit() const {
    return unit;
}
```

## Scale.hpp

```cpp
#pragma once

#include <string>
#include "Vector2D.hpp"
#include <vector>

// Describes a physical measurement range in world coordinates.
class Scale {
private:
    // Stores the starting point of the scale in world coordinates.
    Vector2D start;
    // Stores the ending point of the scale in world coordinates.
    Vector2D end;

    // Stores the measurement value at the starting point.
    double minValue;
    // Stores the measurement value at the ending point.
    double maxValue;

    // Stores the number of labeled intervals along the scale.
    int divisions;

    // Stores the unit displayed with the scale labels.
    std::string unit;

public:
    // Creates a world-space measurement scale with a range, intervals, and unit.
    Scale(const Vector2D& startPoint,const Vector2D& endPoint,double minimum,double maximum,int divisionCount,const std::string& unitValue);

    // Returns the starting point in world coordinates.
    Vector2D getStart() const;
    // Returns the ending point in world coordinates.
    Vector2D getEnd() const;

    // Returns the measurement value at the starting point.
    double getMinValue() const;
    // Returns the measurement value at the ending point.
    double getMaxValue() const;

    // Returns the number of labeled intervals.
    int getDivisions() const;

    // Returns the measurement unit used by the labels.
    std::string getUnit() const;
};
```

## Simulation.cpp

```cpp
#include "Simulation.hpp"

// Initializes physical time and playback state before configuration.
Simulation::Simulation(): current_time(0.0),timeScale(1.0),hasInitialState(false),paused(false) {
}

// Coordinates one physics step and finalizes the phenomenon when it ends.
void Simulation::step(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double frameDeltaTime) {
    if (paused) {
        return;
    }

    if (phenomenon.isFinished(object)) {
        return;
    }

    // Converts real frame duration into elapsed physical time.
    double physicalDeltaTime =frameDeltaTime * timeScale;

    phenomenon.apply(object,physicalDeltaTime);

    integrator.integrate(object,physicalDeltaTime);

    current_time += physicalDeltaTime;

    if (phenomenon.isFinished(object)) {
        phenomenon.finish(object);
    }
}

// Returns the elapsed physical simulation time in seconds.
double Simulation:: getCurrentTime() const{
    return current_time;
}

// Returns the ratio of physical time to real playback time.
double Simulation:: getTimeScale() const{
    return timeScale;
}

// Restores the saved initial physical state and resets physical time.
void Simulation::reset(PhysicalObject& object) {
    if (!hasInitialState) {
        return;
    }

    object = initialState;

    current_time = 0.0;
}

// Saves the initial state and relates physical duration to real playback duration.
void Simulation::configureTimeScale(const PhysicalObject& object,const PhysicalPhenomenon& phenomenon,double playbackDuration) {
    initialState = object;
    hasInitialState = true;

    double physicalDuration =phenomenon.getExpectedDuration(object);

    if (playbackDuration > 0.0 &&physicalDuration > 0.0) {
        timeScale =physicalDuration /playbackDuration;
    }else {timeScale = 1.0;}
}

// Reconstructs the physical state at the requested physical time.
void Simulation::seek(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double targetTime) {
    if (!hasInitialState) {
        return;
    }

    if (targetTime < 0.0) {
        targetTime = 0.0;
    }

    reset(object);

    // Replays small physics steps to reconstruct the requested time.
    const double physicsStep =1.0 / 240.0;

    while (current_time < targetTime &&!phenomenon.isFinished(object)) {
        double deltaTime = physicsStep;

        if (current_time + deltaTime >targetTime) {
            deltaTime =
                targetTime - current_time;
        }

        phenomenon.apply(object,deltaTime);

        integrator.integrate(object,deltaTime);

        current_time += deltaTime;

        if (phenomenon.isFinished(object)) {
            phenomenon.finish(object);
        }
    }
}

// Resumes playback.
void Simulation::play() {
    paused = false;
}

// Pauses playback.
void Simulation::pause() {
    paused = true;
}

// Toggles the playback pause state.
void Simulation::togglePause() {
    paused = !paused;
}

// Reports whether playback is paused.
bool Simulation::isPaused() const {
    return paused;
}

// Returns the estimated phenomenon duration from the saved initial state.
double Simulation::getDuration(const PhysicalPhenomenon& phenomenon) const {
    if (!hasInitialState) {
        return 0.0;
    }
    return phenomenon.getExpectedDuration(initialState);
}
```

## Simulation.hpp

```cpp
#pragma once

#include "PhysicalObject.hpp"
#include "PhysicalPhenomenon.hpp"
#include "Integrator.hpp"

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
public:
    // Initializes physical time and playback state before configuration.
    Simulation();

    // Coordinates one physics step and finalizes the phenomenon when it ends.
    void step(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double frameDeltaTime);

    // Returns the elapsed physical simulation time in seconds.
    double getCurrentTime() const;
    // Returns the ratio of physical time to real playback time.
    double getTimeScale() const;

    // Returns the estimated phenomenon duration from the saved initial state.
    double getDuration(const PhysicalPhenomenon& phenomenon) const;

    // Restores the saved initial physical state and resets physical time.
    void reset(PhysicalObject& object);

    // Saves the initial state and relates physical duration to real playback duration.
    void configureTimeScale(const PhysicalObject& object,const PhysicalPhenomenon& phenomenon,double playbackDuration);

    // Reconstructs the physical state at the requested physical time.
    void seek(PhysicalObject& object,const PhysicalPhenomenon& phenomenon,const Integrator& integrator,double targetTime);

    // Resumes playback.
    void play();

    // Pauses playback.
    void pause();

    // Toggles the playback pause state.
    void togglePause();

    // Reports whether playback is paused.
    bool isPaused() const;
};
```

## Vector2D.cpp

```cpp
#include "Vector2D.hpp"
#include <cmath>

// Initializes a vector with the specified components.
Vector2D::Vector2D(double xValue, double yValue){
    x=xValue;
    y=yValue;
}

// Returns the sum of this vector and another vector.
Vector2D Vector2D::operator+(const Vector2D& other)const{
    return Vector2D (x+other.x,y+other.y);
}

// Returns the result of subtracting another vector from this vector.
Vector2D Vector2D:: operator-(const Vector2D& other)const{
    return Vector2D (x-other.x,y-other.y);
}

// Returns this vector multiplied by a scalar.
Vector2D Vector2D:: operator*(const double& other)const{
    return Vector2D (x*other,y*other);
}

// Returns this vector divided by a scalar.
Vector2D Vector2D:: operator/(const double& other)const{
    return Vector2D (x/other,y/other);
}

// Returns a unit vector in the same direction, or a zero vector if the magnitude is zero.
Vector2D Vector2D::normalized()const{
    double mag= magnitude();
    // Avoids division by zero when normalizing a zero vector.
    if(mag==0){
        return Vector2D(0.0,0.0);
    }
    return Vector2D(x/mag,y/mag);
}

// Adds another vector to this vector and returns a reference to the result.
Vector2D& Vector2D::operator+=(const Vector2D& other){
    x+=other.x;
    y+=other.y;
    return *this;
}
// Subtracts another vector from this vector and returns a reference to the result.
Vector2D& Vector2D::operator-=(const Vector2D& other){
    x-=other.x;
    y-=other.y;
    return *this;
}

// Multiplies this vector by a scalar and returns a reference to the result.
Vector2D& Vector2D::operator*=(const double& other){
    x*=other;
    y*=other;
    return *this;
}

// Divides this vector by a scalar and returns a reference to the result.
Vector2D& Vector2D::operator/=(const double& other){
    x/=other;
    y/=other;
    return *this;
}

// Returns the squared magnitude of this vector.
double Vector2D::magnitudeSquared() const{
    double mag= pow(x,2)+pow(y,2);
    return mag;
}

// Returns the magnitude of this vector.
double Vector2D::magnitude() const{
    double mag= sqrt(pow(x,2)+pow(y,2));
    return mag;
}

// Returns the dot product of this vector and another vector.
double Vector2D:: dot(const Vector2D& other)const{
    double product= x*other.x+y*other.y;
    return product;
}

// Returns the scalar cross product of this vector and another vector.
double Vector2D::cross(const Vector2D& other)const{
    double product= x*other.y-other.x*y;
    return product;
}

// Returns the distance between this vector and another vector.
double Vector2D::distanceTo(const Vector2D&other)const{
    double d= sqrt(pow(other.x-x,2)+pow(other.y-y,2));
    return d;
}
```

## Vector2D.hpp

```cpp
#pragma once

// Represents the mathematical two-dimensional vector used by the physics engine.
class Vector2D {
public:
    // Stores the horizontal vector component.
    double x;
    // Stores the vertical vector component.
    double y;

    // Initializes a vector with the specified components or zero by default.
    Vector2D(double xValue=0.0, double yValue=0.0);

    // Returns the sum of this vector and another vector.
    Vector2D operator+(const Vector2D& other)const;
    // Returns the result of subtracting another vector from this vector.
    Vector2D operator-(const Vector2D& other)const;
    // Returns this vector multiplied by a scalar.
    Vector2D operator*(const double& other)const;
    // Returns this vector divided by a scalar.
    Vector2D operator/(const double& other)const;
    // Adds another vector to this vector and returns a reference to the result.
    Vector2D& operator+=(const Vector2D& other);
    // Subtracts another vector from this vector and returns a reference to the result.
    Vector2D& operator-=(const Vector2D& other);
    // Multiplies this vector by a scalar and returns a reference to the result.
    Vector2D& operator*=(const double& other);
    // Divides this vector by a scalar and returns a reference to the result.
    Vector2D& operator/=(const double& other);
    // Returns a unit vector in the same direction, or a zero vector if the magnitude is zero.
    Vector2D normalized()const;

    // Returns the squared magnitude of this vector.
    double magnitudeSquared() const;
    // Returns the magnitude of this vector.
    double magnitude() const;
    // Returns the dot product of this vector and another vector.
    double dot(const Vector2D& other)const;
    // Returns the scalar cross product of this vector and another vector.
    double cross(const Vector2D& other)const;
    // Returns the distance between this vector and another vector.
    double distanceTo(const Vector2D& other)const;
};
```

## main.cpp

```cpp
#include <raylib.h>
#include <vector>

#include "PhysicalObject.hpp"
#include "FreeFall.hpp"
#include "EulerIntegrator.hpp"
#include "Simulation.hpp"
#include "Renderer.hpp"
#include "Scale.hpp"

// Creates the simulation, runs playback, and handles graphical controls.
int main() {

    // ========================================
    // Initial physical conditions
    // ========================================

    // Creates the physical object used in the simulation.
    PhysicalObject ball(
        2.0,                        // mass: 2 kg
        Vector2D(0.0, 1000.0),     // initial position
        Vector2D(0.0, 0.0)         // initial velocity
    );

    // ========================================
    // Physics
    // ========================================

    // Defines the constant-gravity physical phenomenon.
    FreeFall freeFall;

    // Selects explicit Euler as the numerical integration strategy.
    EulerIntegrator integrator;

    // Coordinates physical evolution and playback state.
    Simulation simulation;

    // Visual playback duration in real seconds.
    const double playbackDuration = 4.0;

    // Configures the relationship between physical and real playback time.
    simulation.configureTimeScale(
        ball,
        freeFall,
        playbackDuration
    );

    // ========================================
    // Renderer
    // ========================================

    // Creates the rendering window and reserves space for playback controls.
    Renderer renderer(
        1000,       // width
        700,        // height
        40.0,       // fallback scale
        140         // control panel height
    );

    // ========================================
    // Scales
    // ========================================

    // Retrieves the physical measurement scales required by the phenomenon.
    std::vector<Scale> scales =
        freeFall.getScales(ball);

    renderer.fitToScales(
        scales
    );

    // Physical duration of the phenomenon.
    double simulationDuration =
        simulation.getDuration(
            freeFall
        );

    // ========================================
    // Control panel
    // ========================================

    bool running = true;

    float panelTop =
        static_cast<float>(
            renderer.getControlPanelTop()
        );

    Rectangle restartButton = {
        250.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    Rectangle playPauseButton = {
        430.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    Rectangle exitButton = {
        610.0f,
        panelTop + 80.0f,
        140.0f,
        40.0f
    };

    // ========================================
    // Main loop
    // ========================================

    while (
        !renderer.shouldClose() &&
        running
    ) {

        double frameDeltaTime =
            GetFrameTime();

        // ====================================
        // Update physics
        // ====================================

        simulation.step(
            ball,
            freeFall,
            integrator,
            frameDeltaTime
        );

        // ====================================
        // Begin rendering
        // ====================================

        renderer.beginFrame();

        // ====================================
        // Simulation area
        // ====================================

        renderer.drawScales(
            scales
        );

        renderer.drawObject(
            ball
        );

        // Velocity vector
        renderer.drawVelocityVector(
            ball,
            simulationDuration
        );

        // Acceleration vector
        renderer.drawAccelerationVector(
            ball,
            freeFall.getAcceleration(ball),
            simulationDuration
        );

        // Numerical information
        renderer.drawSimulationInfo(
            ball,
            simulation.getCurrentTime()
        );

        // ====================================
        // Control panel
        // ====================================

        renderer.drawControlPanel();

        // ------------------------------------
        // Timeline
        // ------------------------------------

        double selectedTime = 0.0;

        bool seeking =
            renderer.drawTimeline(
                simulation.getCurrentTime(),
                simulationDuration,
                selectedTime
            );

        if (seeking) {

            simulation.seek(
                ball,
                freeFall,
                integrator,
                selectedTime
            );
        }

        // ------------------------------------
        // Restart button
        // ------------------------------------

        if (
            renderer.drawButton(
                restartButton,
                "Restart"
            )
        ) {
            simulation.reset(ball);
        }

        // ------------------------------------
        // Play / Pause button
        // ------------------------------------

        const char* playPauseText =
            simulation.isPaused()
                ? "Play"
                : "Pause";

        if (
            renderer.drawButton(
                playPauseButton,
                playPauseText
            )
        ) {
            simulation.togglePause();
        }

        // ------------------------------------
        // Exit button
        // ------------------------------------

        if (
            renderer.drawButton(
                exitButton,
                "Exit"
            )
        ) {
            running = false;
        }

        // ====================================
        // Finish rendering
        // ====================================

        renderer.endFrame();
    }

    return 0;
}
```

