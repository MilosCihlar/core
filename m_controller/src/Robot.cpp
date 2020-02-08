#include <string>
#include <iostream>
#include "m_controller/Robot.h"

#include "m_controller/Point.h"
#include "m_controller/Position.h"
#include "m_controller/Wheel.h"

/*Constructors and destructors*/
Robot::Robot(): position(), velocity(), left(), right(), span(0) {}

Robot::Robot(const Wheel& wheel, const Position& pos, const Position& vel, const double& span)
:left(wheel), right(wheel), position(pos), velocity(vel), span(span) {}

Robot::Robot(const Wheel& wheel, const double& span): left(wheel), right(wheel), position(), velocity(), span(span) {}

Robot::Robot(const Robot& robot)
{
    position = robot.position;
    velocity = robot.velocity;
    left = robot.left;
    right = robot.right;
    span = robot.span;
}

Robot::~Robot(){}

/*Operators*/
Robot& Robot::operator=(const Robot& robot)
{
    position = robot.position;
    velocity = robot.velocity;
    left = robot.left;
    right = robot.right;
    span = robot.span;
}

/*Subscribe function*/
void Robot::subscribeVelocity(const geometry_msgs::Twist& v)
{
    Point lin(v.linear.x, v.linear.y, v.linear.z);
    Point ang(v.angular.x, v.angular.y, v.angular.z);
    Position vel(lin, ang);

    velocity = vel;
    wheelVelocityFromRobot();
}

void Robot::subscribeWheelsEncoder(const sensor_msgs::JointState& state)
{
    left.newAngle(state.position[0]);
    right.newAngle(state.position[1]);
    robotVelocityFromWheels();
}

/*Methods*/
void Robot::robotVelocityFromWheels()
{
    Point lin((left.getVelocity() + right.getVelocity())/2 , 0, 0);
    Point ang(0,0, (left.getVelocity() - right.getVelocity())/span);
    Position vel(lin, ang);

    velocity = vel;
}

void Robot::wheelVelocityFromRobot()
{
    Point lin = velocity.getPoint();
    Point ang = velocity.getAngle();
    left.setVelocity(lin.getX() - (span/2)*ang.getZ());
    right.setVelocity(lin.getX() + (span/2)*ang.getZ());
}


/*Getters and Setters*/
void Robot::setVelocity(const Position& vel)
{
    velocity = vel;
    double x = velocity.getPoint().getX();
    double gama = velocity.getAngle().getZ();

    right.setVelocity(x + (span/2)*gama);
    left.setVelocity(x - (span/2)*gama);
}

void Robot::setPosition(const Position& pos)
{
    position.setPoint(pos.getPoint());
    position.setAngle(pos.getAngle());
}

Position Robot::getPosition() const
{
    return position;
}
Position Robot::getVelocity() const
{
    return velocity;
}
Wheel& Robot::getLeftWheel()
{
        return left;
}

Wheel& Robot::getRightWheel()
{
        return right;

}