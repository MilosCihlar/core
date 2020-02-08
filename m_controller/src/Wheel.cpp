#include <iostream>

#include "m_controller/Wheel.h"
#include "math.h"

// c'tord and d'tors
Wheel::Wheel():radius(0), angularVel(0), deltaAngle(0), totalAngle(0), period(0) 
{
    if(angleHistory == nullptr)
    {

        angleHistory = new double[historyLenght];
 
    }    
}

Wheel::Wheel(const double& radius): radius(radius), angularVel(0), deltaAngle(0), totalAngle(0), period(0) 
{
    if(angleHistory == nullptr)
    {
        angleHistory = new double[10];
    }
}

Wheel::Wheel(const Wheel& wheel)
{
    radius = wheel.radius;
    angularVel = wheel.angularVel;
    deltaAngle = wheel.deltaAngle;
    totalAngle = wheel.totalAngle;
    period = wheel.period;
}

Wheel::~Wheel() 
{
    if(angleHistory != nullptr)
        delete[] angleHistory;
}

// Operators
Wheel& Wheel::operator=(const Wheel& wheel)
{
    radius = wheel.radius;
    angularVel = wheel.angularVel;
    deltaAngle = wheel.deltaAngle;
    totalAngle = wheel.totalAngle;
    period = wheel.period;
}

// Methods

void Wheel::newAngle(const double& angle)
{
    for (int i = 0; i < historyLenght-1; ++i)
    {
        angleHistory[i] = angleHistory[i+1];
    }
    angleHistory[historyLenght-1] = angle;
    setVelocity();
}

void Wheel::setVelocity()
{
    double sum;
    for (int i = 0; i < historyLenght; ++i)
    {
        sum += angleHistory[i];
    }

    angularVel = sum/historyLenght;
    period = ((2*M_PI)/angularVel)*radius;
}

void Wheel::setVelocity(const double& velocity)
{
    period = ((2*M_PI)/velocity)*radius;
    angularVel = velocity;
}

double Wheel::newMove(const double& seconds)
{
    deltaAngle = ((2*M_PI)*seconds)/period;
    totalAngle = totalAngle + deltaAngle;
    return totalAngle;
}


// Getters and Setters
double Wheel::getPeriod() const
{
    return period;
}

double Wheel::getVelocity() const
{
    return angularVel;
}