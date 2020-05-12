#include <cmath>
#include <iostream>
#include "ros/ros.h"

#include "m_odometry/odometry.h"

Odometry::Odometry(double x, double y, double theta, double radius, double D):x(x), y(y), t(theta), r(radius), D(D)
{
    this->actual_time = ros::Time::now().toSec();
    this->r_wheel_vel_buff = new double[this->wheel_buff];
    this->l_wheel_vel_buff = new double[this->wheel_buff];
    for (int i = 0; i < this->wheel_buff; i++)
    {
        this->r_wheel_vel_buff[i] = 0;
        this->l_wheel_vel_buff[i] = 0;
    }
    
};

Odometry::~Odometry()
{
    delete[] this->r_wheel_vel_buff;
    delete[] this->l_wheel_vel_buff;    
};


void Odometry::calculateOdometry(double l_last_pos, double l_actual_pos, double r_last_pos, double r_actual_pos, double time)
{
    this->number = this->number + 1;
    this->last_time = this->actual_time;
    this->actual_time = time;

    this->calcWheelVelocity(l_last_pos, l_actual_pos, r_last_pos, r_actual_pos);

    this->calcLinearVelocity();
    this->calcAngularVelocity();

    this->calcTheta();
    this->calcLinearSpeed();
    this->calcPosition();
}

void Odometry::calcWheelVelocity(double l_last_pos, double l_actual_pos, double r_last_pos, double r_actual_pos)
{
    //derivation of wheel angle position is angular velocuty and multiplicate with raius get peripheral speed
    for (size_t i = 0; i < this->wheel_buff - 2; i++)
    {
        this->l_wheel_vel_buff[i] = this->l_wheel_vel_buff[i + 1];
        this->r_wheel_vel_buff[i] = this->r_wheel_vel_buff[i + 1];  
    }

    if((this->actual_time - this->last_time) > 0)
    {
        this->l_wheel_vel_buff[this->wheel_buff - 1] = this->r*((l_actual_pos - l_last_pos)/(this->actual_time - this->last_time)); 
        this->r_wheel_vel_buff[this->wheel_buff - 1]= this->r*((r_actual_pos - r_last_pos)/(this->actual_time - this->last_time));

        this->l_wheel_vel = this->l_wheel_vel_buff[this->wheel_buff-1];
        this->r_wheel_vel = this->r_wheel_vel_buff[this->wheel_buff-1];
    }

}

inline void Odometry::calcLinearVelocity()
{
    //from wheels velocity get velocities (linear and angular) of robot
    this->lin_speed = (this->r_wheel_vel + this->l_wheel_vel)/2;
 
}

inline void Odometry::calcAngularVelocity()
{
    this->ang_speed_last = this->ang_speed;
    this->ang_speed = (this->r_wheel_vel - this->l_wheel_vel)/this->D;
}

inline void Odometry::calcTheta()
{
    this->t = this->t + this->oneStepIntegral(this->ang_speed_last, this->ang_speed);
}

inline void Odometry::calcLinearSpeed()
{
    this->x_speed_last = this->x_speed;
    this->x_speed = this->lin_speed*cos(this->t);
    this->y_speed_last = this->y_speed;
    this->y_speed = this->lin_speed*sin(this->t);
}

inline void Odometry::calcPosition()
{
    this->x = this->x + this->oneStepIntegral(this->x_speed_last, this->x_speed);
    this->y = this->y + this->oneStepIntegral(this->y_speed_last, this->y_speed);
}

double Odometry::oneStepIntegral(double last, double now)
{
    double  rectangle = 0;
    double triangle = 0;

    if(((last < 0) && (now > 0)) || ((last > 0) && (now < 0)))
    {   
        double t = (-last*(this->actual_time - this->last_time))/(now - last);
        triangle = 0.5*last*t + 0.5*now*(this->actual_time - this->last_time - t);
    }
    else
    {
        rectangle = last*(this->actual_time - this->last_time);
        triangle = 0.5*(now-last)*(this->actual_time - this->last_time);    
    }
    
    return (rectangle + triangle);
}

void Odometry::getVelocity(double* vel)
{
    vel[0] = this->x_speed;
    vel[1] = this->y_speed;
    vel[2] = this->ang_speed;
}
void Odometry::getPosition(double* pos)
{
    pos[0] = this->x;
    pos[1] = this->y;
    pos[2] = this->t;

}

double Odometry::getTime()
{
    return this->actual_time;
}
unsigned int Odometry::getNumber()
{
    return this->number;
}

double Odometry::getXSpeed()
{
	return x_speed;
}
double Odometry::getYSpeed()
{
	return y_speed;
}

double Odometry::getXSpeedLast()
{
	return x_speed_last;
}

double Odometry::getYSpeedLast()
{
	return y_speed_last;
}

double Odometry::getLinearSpeed()
{
	return lin_speed;
}


void Odometry::setXSpeed(double speed)
{
	x_speed = speed;
}
void Odometry::setYSpeed(double speed)
{
	y_speed = speed;
}

void Odometry::setX(double x)
{
	this->x = x;
}

void Odometry::setY(double y)
{
	this->y = y;
}

void Odometry::setTheta(double t)
{
	this->t = t;
}