#include <iostream>
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "m_controller/MController.h"

MController::MController(ros::NodeHandle *nh): request(), actual()
{
    s_velocity = nh->subscribe("s_velocity_cmd", 1, &Robot::subscribeVelocity, &request);
    p_leftJoint = nh->advertise<std_msgs::Float64>("p_leftJoint_cmd", 1);
    p_rightJoint = nh->advertise<std_msgs::Float64>("p_rightJoint_cmd", 1);
}

MController::MController(ros::NodeHandle *nh, const Robot& actual): request(actual), actual(actual)
{
    s_velocity = nh->subscribe("s_velocity_cmd", 1, &Robot::subscribeVelocity, &request);
    p_leftJoint = nh->advertise<std_msgs::Float64>("p_leftJoint_cmd", 1);
    p_rightJoint = nh->advertise<std_msgs::Float64>("p_rightJoint_cmd", 1);
}

MController::~MController() {}

void MController::PSDF()
{

}

/*BUG: Dodelat filtr zadane hodnoty -> v ramci testovani jen nastavi pozadovanou hodnotu*/
void MController::setpointFilter()
{ 
    actual.setVelocity(request.getVelocity());
}

void MController::sendCommand()
{
      double sec = ros::Time::now().toSec();

      std_msgs::Float64 left; 
      left.data = actual.getLeftWheel().newMove(sec - lastTime);

      std_msgs::Float64 right; 
      right.data = actual.getRightWheel().newMove(sec - lastTime);

      p_leftJoint.publish(left);
      p_rightJoint.publish(right);
      lastTime = sec;
}
