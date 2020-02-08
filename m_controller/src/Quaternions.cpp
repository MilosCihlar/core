#include "m_controller/Quaternions.h"

// c'tors and d'tors
Quaternions::Quaternions():x(0), y(0), z(0) { }

Quaternions::Quaternions(const double& x, const double& y, const double& z, const double& w):x(x), y(y), z(z) { }

Quaternions::Quaternions(const double& x):x(x), y(y), z(z) { }

Quaternions::Quaternions(const Quaternions& point)
{
    x = point.x;
    y = point.y;
    z = point.z;
}

Quaternions::~Quaternions(){}

// Operators
Quaternions& Quaternions::operator=(const Quaternions& quat)
{
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;

    return *this;
}

// Setter and Getter
double Quaternions::getX() const
{
    return x;
}

double Quaternions::getY() const
{
    return y;
}


double Quaternions::getZ() const
{
    return z;
}

double Quaternions::getW() const
{
    return w;
}

void Quaternions::setX(const double& value) 
{
    x = value;
}

void Quaternions::setY(const double& value) 
{
    y = value;
}

void Quaternions::setZ(const double& value)
{
    z = value;
}

void Quaternions::setW(const double& value)
{
    w = value;
}



