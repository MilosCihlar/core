#ifndef __ROBOT_H__
#define __ROBOT_H__
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

#include "Point.h"
#include "Position.h"
#include "Wheel.h"

class Robot
{
private:
    Position position;
    Position velocity;
    Wheel left;
    Wheel right;

    double span;          
        
    void robotVelocityFromWheels();
    void wheelVelocityFromRobot();
public:
    Robot();
    Robot(const Wheel& wheel, const Position& pos, const Position& vel, const double& span);
    Robot(const Wheel& wheel, const double& span);
    Robot(const Robot& robot);
    ~Robot();

    Robot& operator=(const Robot& robot);

    void subscribeVelocity(const geometry_msgs::Twist& velocity);
    void subscribeWheelsEncoder(const sensor_msgs::JointState& state);

    void setPosition(const Position& pos);
    void setVelocity(const Position& vel);
    Position getPosition() const;
    Position getVelocity() const;
    Wheel& getLeftWheel();
    Wheel& getRightWheel();
};

#endif