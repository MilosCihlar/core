#ifndef __M_CONTROLLER_H__
#define __M_CONTROLLER_H__
#include <iostream>
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "Point.h"
#include "Position.h"
#include "Robot.h"
#include "Wheel.h"

class MController
{
private:
    Robot request;
    Robot actual;

    double lastTime = ros::Time::now().toSec(); 
    ros::Subscriber s_velocity;     // Topic with requested velocity
    ros::Subscriber s_joit;         // Topic wiht information from encoder
    ros::Subscriber s_odom;
    ros::Publisher p_leftJoint;     // Topic which control left Wheel
    ros::Publisher p_rightJoint;    // Topic which control right Wheel
public:
    MController(ros::NodeHandle *nh);
    MController(ros::NodeHandle *nh, const Robot& actual);
    ~MController();

    void PSDF();
    void setpointFilter();
    void sendCommand();

};


#endif