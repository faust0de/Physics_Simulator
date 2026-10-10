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
