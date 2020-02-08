#include <iostream>
#include "m_controller/Point.h"

// c'tors and d'tors
Point::Point():x(0), y(0), z(0) { }

Point::Point(const double& x, const double& y, const double& z):x(x), y(y), z(z) { }

Point::Point(const double& x):x(x), y(y), z(z) { }

Point::Point(const Point& point)
{
    x = point.x;
    y = point.y;
    z = point.z;
}

Point::~Point(){}

// Operators
Point& Point::operator=(const Point& point)
{
    x = point.x;
    y = point.y;
    z = point.z;

    return *this;
}

// Setter and Getter
double Point::getX() const
{
    return x;
}

double Point::getY() const
{
    return y;
}


double Point::getZ() const
{
    return z;
}

void Point::setX(const double& value) 
{
    x = value;
}

void Point::setY(const double& value) 
{
    y = value;
}

void Point::setZ(const double& value)
{
    z = value;
}



