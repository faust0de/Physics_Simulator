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
      scaleValue(scale),
      referenceDistance(1.0),
      timelineDragging(false),
      controlPanelHeight(panelHeight) {

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

    // Keeps vector labels clear of window edges, information, and controls.
    auto keepInside = [&](Vector2 position) {
        const float margin = 12.0f;
        const float infoBottom = 120.0f;
        position.x = std::clamp(position.x,margin,
            std::max(margin,screenWidth - textWidth - margin));
        position.y = std::clamp(position.y,infoBottom,
            std::max(infoBottom,screenHeight - controlPanelHeight - fontSize - margin));
        return position;
    };

    if (length <= 0.001f) {
        // A zero vector has no direction; keep its horizontal label nearby.
        return keepInside(Vector2{
            end.x + side * (sideOffset + textWidth / 2.0f) - textWidth / 2.0f,
            end.y - fontSize / 2.0f
        });
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

    return keepInside(Vector2{
        end.x + directionX * forwardOffset + normalX * normalOffset * side
            - textWidth / 2.0f,
        end.y + directionY * forwardOffset + normalY * normalOffset * side
            - fontSize / 2.0f
    });
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

    const double maxArrowPixels = 80.0;
    double maxWorldLength =std::min(referenceDistance *maxVectorFraction,maxArrowPixels / scaleValue);

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

    const double maxArrowPixels = 60.0;
    double maxWorldLength = std::min(
        referenceDistance * maxAccelerationFraction,
        maxArrowPixels / scaleValue
    );

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
    std::vector<Rectangle> occupiedLabels;
    drawScale(scale,occupiedLabels);
}

// Places grouped scale labels without overlapping previously drawn quantities.
void Renderer::drawScale(const Scale& scale,std::vector<Rectangle>& occupiedLabels) const {
    Vector2D startWorld = scale.getStart();

    Vector2D endWorld = scale.getEnd();

    Vector2 start =toScreen(startWorld);

    Vector2 end =toScreen(endWorld);

    // Main scale line
    DrawLineV(start,end,DARKGRAY);

    float dx = end.x - start.x;
    float dy = end.y - start.y;

    float length = std::sqrt(dx * dx + dy * dy);

    if (length <= 0.001f || scale.getDivisions() <= 0) {
        return;
    }

    // Perpendicular unit vector
    float normalX = -dy / length;
    float normalY = dx / length;

    // Role metadata places projection labels outside the reference triangle.
    if ((scale.getRole() == ScaleRole::Horizontal && normalY < 0.0f) ||
        (scale.getRole() == ScaleRole::Vertical && normalX < 0.0f) ||
        (scale.getRole() == ScaleRole::Path && dx*dy < 0.0f)) {
        normalX = -normalX;
        normalY = -normalY;
    }

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

        const int fontSize = scale.getRole() == ScaleRole::Generic ? 18 : 16;
        std::string text = TextFormat("%.1f",value);
        bool deltaLabel = false;
        if (i == divisions && !scale.getLabel().empty()) {
            std::string quantity = scale.getLabel();
            deltaLabel = quantity.compare(0,2,"Δ") == 0;
            if (deltaLabel) quantity = quantity.substr(2);
            text = quantity + " = " + text;
        }
        if (i == divisions || scale.getRole() == ScaleRole::Generic) text += " " + scale.getUnit();
        const int deltaWidth = deltaLabel ? 13 : 0;
        int textWidth = MeasureText(text.c_str(),fontSize) + deltaWidth;

        // Project the full label extent onto the normal to keep text off the ruler.
        float labelExtent = std::abs(normalX) * textWidth / 2.0f +
            std::abs(normalY) * fontSize / 2.0f;
        float labelOffset = tickSize + 8.0f + labelExtent;
        float labelX = pointX + normalX * labelOffset - textWidth / 2.0f;
        float labelY = pointY + normalY * labelOffset - fontSize / 2.0f;
        bool placed = false;
        for (int attempt=0;attempt<9 && !placed;++attempt) {
            float offset = labelOffset+attempt*(fontSize+6.0f);
            labelX = pointX+normalX*offset-textWidth/2.0f;
            labelY = pointY+normalY*offset-fontSize/2.0f;
            Rectangle bounds{labelX-2.0f,labelY-2.0f,static_cast<float>(textWidth)+4.0f,static_cast<float>(fontSize)+4.0f};
            if (bounds.x < 0.0f || bounds.x+bounds.width > screenWidth ||
                bounds.y < 110.0f || bounds.y+bounds.height > screenHeight-controlPanelHeight-8.0f) continue;
            bool overlaps = false;
            for (const Rectangle& occupied : occupiedLabels) {
                if (CheckCollisionRecs(bounds,occupied)) { overlaps = true; break; }
            }
            if (!overlaps) {
                occupiedLabels.push_back(bounds);
                placed = true;
            }
        }
        // Keep every tick even when an unusually cramped numeric label cannot fit.
        if (!placed) continue;
        if (deltaLabel) {
            // The default raylib font lacks Greek Delta, so draw its outline explicitly.
            Vector2 top{labelX+5.0f,labelY+2.0f};
            Vector2 left{labelX,labelY+14.0f};
            Vector2 right{labelX+10.0f,labelY+14.0f};
            DrawLineV(left,top,DARKGRAY);
            DrawLineV(top,right,DARKGRAY);
            DrawLineV(right,left,DARKGRAY);
        }
        DrawText(text.c_str(),static_cast<int>(labelX)+deltaWidth,static_cast<int>(labelY),fontSize,DARKGRAY);
    }
}

// Draws the measurement scales supplied by the phenomenon.
void Renderer::drawScales(const std::vector<Scale>& scales) const {
    std::vector<Rectangle> occupiedLabels;
    for (const Scale& scale : scales) {
        drawScale(scale,occupiedLabels);
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

// Centers all physical bounds with one pixels-per-meter scale for both axes.
void Renderer::fitToScales(const std::vector<Scale>& scales,const Vector2D& fallbackPosition,const TrajectoryTarget& target,const std::vector<Vector2D>& previewPoints) {
    // Reserves screen space for information, ruler labels, vectors, and controls.
    const double horizontalMargin = 160.0;
    const double topMargin = 200.0;
    const double bottomMargin = 120.0;
    const double plotLeft = horizontalMargin;
    const double plotRight = screenWidth - horizontalMargin;
    const double plotTop = topMargin;
    const double plotBottom = screenHeight - controlPanelHeight - bottomMargin;
    const double availableWidth = plotRight - plotLeft;
    const double availableHeight = plotBottom - plotTop;
    if (availableWidth <= 0.0 || availableHeight <= 0.0) {
        return;
    }

    // Includes the object even when a ruler is offset from its trajectory.
    double minX = fallbackPosition.x;
    double maxX = fallbackPosition.x;
    double minY = fallbackPosition.y;
    double maxY = fallbackPosition.y;
    referenceDistance = 0.0;
    for (const Scale& scale : scales) {
        Vector2D start = scale.getStart();
        Vector2D end = scale.getEnd();
        minX = std::min(minX,std::min(start.x,end.x));
        maxX = std::max(maxX,std::max(start.x,end.x));
        minY = std::min(minY,std::min(start.y,end.y));
        maxY = std::max(maxY,std::max(start.y,end.y));
        referenceDistance = std::max(referenceDistance,start.distanceTo(end));
    }

    // Circular target bounds are generic world geometry, not phenomenon-specific.
    if (target.enabled) {
        minX = std::min(minX,target.position.x-target.radius);
        maxX = std::max(maxX,target.position.x+target.radius);
        minY = std::min(minY,target.position.y-target.radius);
        maxY = std::max(maxY,target.position.y+target.radius);
    }

    // Preview points expand framing without adding target rulers or a target trail.
    for (const Vector2D& point : previewPoints) {
        double radius = target.enabled ? target.radius : 0.0;
        minX=std::min(minX,point.x-radius);
        maxX=std::max(maxX,point.x+radius);
        minY=std::min(minY,point.y-radius);
        maxY=std::max(maxY,point.y+radius);
    }

    double spanX = maxX - minX;
    double spanY = maxY - minY;
    const double epsilon = 0.000001;
    const double majorSpan = std::max(spanX,spanY);
    const double minimumMinorSpan = std::max(majorSpan * 0.20,1.0);
    const double defaultSpan = 10.0;
    if (majorSpan < epsilon) {
        spanX = defaultSpan;
        spanY = defaultSpan;
    }
    else {
        // Artificial camera spans never become physical ruler distances.
        if (spanX < epsilon) {
            spanX = minimumMinorSpan;
        }
        if (spanY < epsilon) {
            spanY = minimumMinorSpan;
        }
    }
    if (referenceDistance <= epsilon) {
        referenceDistance = std::max(spanX,spanY);
    }

    const double paddingFraction = 0.12;
    const double worldWidth = spanX * (1.0 + 2.0 * paddingFraction);
    const double worldHeight = spanY * (1.0 + 2.0 * paddingFraction);
    scaleValue = std::min(availableWidth / worldWidth,availableHeight / worldHeight);

    double worldCenterX = minX + (maxX - minX) / 2.0;
    double worldCenterY = minY + (maxY - minY) / 2.0;
    double viewportCenterX = (plotLeft + plotRight) / 2.0;
    double viewportCenterY = (plotTop + plotBottom) / 2.0;
    origin = Vector2{
        static_cast<float>(viewportCenterX - worldCenterX * scaleValue),
        static_cast<float>(viewportCenterY + worldCenterY * scaleValue)
    };
}


// Renders only the actual positions supplied by numerical integration.
void Renderer::drawTrajectory(const std::vector<Vector2D>& points) const {
    for (std::size_t i = 1; i < points.size(); ++i) {
        DrawLineV(toScreen(points[i-1]),toScreen(points[i]),GRAY);
    }
}

// The physical radius is never enlarged; only the center marker has a pixel minimum.
void Renderer::drawTarget(const TrajectoryTarget& target,bool hit) const {
    if (!target.enabled) return;
    Vector2 center = toScreen(target.position);
    Color color = hit ? DARKGREEN : MAROON;
    DrawCircleLines(static_cast<int>(center.x),static_cast<int>(center.y),
        static_cast<float>(target.radius*scaleValue),color);
    const float markerSize = 4.0f;
    DrawLineV(Vector2{center.x-markerSize,center.y},Vector2{center.x+markerSize,center.y},color);
    DrawLineV(Vector2{center.x,center.y-markerSize},Vector2{center.x,center.y+markerSize},color);
}

// Pending is shown during playback or pause, and MISS only after termination.
void Renderer::drawTargetStatus(const TrajectoryTarget& target,const TargetHitResult& hit,double minimumDistance,bool finished) const {
    if (!target.enabled) return;
    Color color = hit.hit ? DARKGREEN : MAROON;
    DrawText(hit.hit ? "Target: HIT" : (finished ? "Target: MISS" : "Target: Pending"),600,112,16,color);
    if (std::isfinite(minimumDistance)) DrawText(TextFormat("Target error: %.3f m",minimumDistance),600,132,16,color);
    if (hit.hit) DrawText(TextFormat("Hit time: %.3f s",hit.time),600,152,16,color);
}


// Draws the shortest geometric arc between supplied world directions.
void Renderer::drawAngleGuide(const AngleGuide& guide) const {
    if (!guide.visible || guide.referenceDirection.magnitude() <= 0.0 || guide.direction.magnitude() <= 0.0) return;
    Vector2 vertex = toScreen(guide.vertex);
    double start = std::atan2(-guide.referenceDirection.y,guide.referenceDirection.x);
    double end = std::atan2(-guide.direction.y,guide.direction.x);
    double sweep = std::atan2(std::sin(end-start),std::cos(end-start));
    const float radius = 30.0f;
    const int segments = 16;
    auto point = [&](double angle,float length) {
        return Vector2{vertex.x+length*static_cast<float>(std::cos(angle)),
            vertex.y+length*static_cast<float>(std::sin(angle))};
    };
    DrawLineV(vertex,point(start,radius+10.0f),DARKGRAY);
    DrawLineV(vertex,point(end,radius+10.0f),DARKGRAY);
    for (int i=0;i<segments;++i) {
        DrawLineV(point(start+sweep*i/segments,radius),point(start+sweep*(i+1)/segments,radius),DARKGRAY);
    }
    const int fontSize = 16;
    std::string text = guide.label.empty() ? TextFormat("%.0f°",guide.angleDegrees) : guide.label;
    Vector2 label = point(start+sweep/2.0,radius+23.0f);
    DrawText(text.c_str(),static_cast<int>(label.x)-MeasureText(text.c_str(),fontSize)/2,
        static_cast<int>(label.y)-fontSize/2,fontSize,DARKGRAY);
}

// The ideal circle is only a reference; actual motion comes from trajectoryHistory.
void Renderer::drawCircularGuide(const CircularGuide& guide) const {
    if (!guide.visible || guide.radius <= 0.0) return;
    Vector2 center = toScreen(guide.center);
    float radius = static_cast<float>(guide.radius*scaleValue);
    DrawCircleLines(static_cast<int>(center.x),static_cast<int>(center.y),radius,LIGHTGRAY);
    if (guide.showCenter) {
        DrawLineV({center.x-4,center.y},{center.x+4,center.y},GRAY);
        DrawLineV({center.x,center.y-4},{center.x,center.y+4},GRAY);
    }
    if (guide.showRadius) {
        DrawLineV(center,{center.x+radius,center.y},GRAY);
        DrawText(guide.radiusLabel.c_str(),static_cast<int>(center.x+radius*0.5f),static_cast<int>(center.y+5),16,GRAY);
    }
}
