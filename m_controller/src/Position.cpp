#include <math.h>
#include <iostream>
#include "m_controller/Position.h"

#include "m_controller/Point.h"
#include "m_controller/Quaternions.h"

// c'tors and d"tors
Position::Position():point(0), angle(0) {}

Position::Position(const Point& pos, const Point& angle):point(pos), angle(angle){}

Position::Position(const Position& euler)
{
    point = euler.getPoint();
    angle = euler.getAngle();
}

Position::~Position() {}

// Operators
Position& Position::operator=(const Position& euler)
{
    point = euler.getPoint();
    angle = euler.getAngle();

    return *this;
}

// Methods
Quaternions Position::eulerToQuaternion() const
{
    Quaternions quat;

    double cy = cos(angle.getZ() * 0.5);
    double sy = sin(angle.getZ() * 0.5);
    double cp = cos(angle.getY() * 0.5);
    double sp = sin(angle.getY() * 0.5);
    double cr = cos(angle.getX() * 0.5);
    double sr = sin(angle.getX() * 0.5);

    quat.setW( cy * cp * cr + sy * sp * sr );
    quat.setX( cy * cp * sr - sy * sp * cr );
    quat.setY( sy * cp * sr + cy * sp * cr );
    quat.setZ( sy * cp * cr - cy * sp * sr );

    return quat;
}


// Getter and Setter
void Position::setPoint(const Point& p)
{
    point = p;
}

void Position::setAngle(const Point& point)
{
    angle = point;
}

Point Position::getPoint() const
{
    return point;
}

Point Position::getAngle() const
{
    return angle;
}


