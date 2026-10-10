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