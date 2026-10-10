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