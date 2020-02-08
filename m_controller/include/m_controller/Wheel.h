#ifndef __WHEEL_H__
#define __WHEEL_H__

class Wheel
{
private:
    double radius;
    
    double angularVel;
    double deltaAngle;
    double totalAngle;
    double period;

    int historyLenght = 10;
    double* angleHistory = nullptr;

    void setVelocity();
public:
    // c'tord and d'tors
    Wheel();
    Wheel(const double& radius);
    Wheel(const Wheel& wheel);
    ~Wheel();

    Wheel& operator=(const Wheel& wheel);
    
    void newAngle(const double& angle);

    /* Set wheel angular velocity*/
    void setVelocity(const double& velocity);
    /* Return total angle of the wheel -> it depends on how often is calling*/
    double newMove(const double& seconds);

    double getPeriod() const;
    double getVelocity() const;

};

#endif