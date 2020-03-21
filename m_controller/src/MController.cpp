#include <iostream>
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "m_controller/MController.h"

#include "m_controller/Point.h"
#include "m_controller/Position.h"
#include "m_controller/Robot.h"
#include "m_controller/Wheel.h"


MController::MController(ros::NodeHandle *nh): request(), actual(), odom(), manual(false), autonomous(false)
{
    s_velocity = nh->subscribe("s_velocity_cmd", 1, &MController::subscribeVelocity, this);
	s_path = nh->subscribe("s_path", 1, &MController::subscribePath, this);
    s_odom = nh->subscribe("s_odometry", 1, &MController::subscribeOdometry, this);

    p_leftJoint = nh->advertise<std_msgs::Float64>("p_leftJoint_cmd", 1);
    p_rightJoint = nh->advertise<std_msgs::Float64>("p_rightJoint_cmd", 1);
	p_leftVelocity = nh->advertise<std_msgs::Float64>("p_leftWheel/cmd", 1);
	p_rightVelocity = nh->advertise<std_msgs::Float64>("p_rightWheel/cmd", 1);
}

MController::MController(ros::NodeHandle *nh, const Robot& actual): request(actual), actual(actual), odom(), manual(false), autonomous(false)
{
    s_velocity = nh->subscribe("s_velocity_cmd", 1,&MController::subscribeVelocity, this);
	s_path = nh->subscribe("s_path", 1, &MController::subscribePath, this);
    s_odom = nh->subscribe("s_odometry", 1, &MController::subscribeOdometry, this);
	
    p_leftJoint = nh->advertise<std_msgs::Float64>("p_leftJoint_cmd", 1);
    p_rightJoint = nh->advertise<std_msgs::Float64>("p_rightJoint_cmd", 1);
}

MController::~MController() {}

void MController::PSDF()
{

}

void MController::subscribeVelocity(const geometry_msgs::Twist& v)
{
	manual = true;
	autonomous = false;
	request.subscribeVelocity(v);
}

void MController::subscribeOdometry(const nav_msgs::Odometry &odometry)
{
	Point pos(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z);
	double siny_cosp = 2 * (odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z + odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y);
	double cosy_cosp = 1 - 2 * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y + odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z);
	double yaw = std::atan2(siny_cosp, cosy_cosp);
	Point orient(0,0, yaw);
	Position position(pos,orient);

	Point vel(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z);
	Point orvel(0,0, odometry.twist.twist.angular.z);
	Position velocity(vel,orvel);

	odom.setPosition(position);
	odom.setVelocity(velocity);
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
	std_msgs::Float64 leftWheel;
	leftWheel.data = actual.getLeftWheel().getVelocity();
	p_leftJoint.publish(leftWheel);

	std_msgs::Float64 right;
	right.data = actual.getRightWheel().newMove(sec - lastTime);
	std_msgs::Float64 rightWheel;
	rightWheel.data = actual.getRightWheel().getVelocity();
	p_rightJoint.publish(rightWheel);

	p_leftJoint.publish(left);
	p_rightJoint.publish(right);
	lastTime = sec;
}

bool MController::getManual() const
{
	return manual;
}

void MController::setManual(const bool man)
{
	manual = man;
}

bool MController::getAutonomous() const
{
	return autonomous;
}

void MController::setAutonomous(const bool aut)
{
	autonomous = aut;
}

void MController::manualControl()
{
	setpointFilter();
	sendCommand();
}

void MController::subscribePath(const nav_msgs::Path &path)
{
	if(manual)
		autonomous = false;
	else
	{
		autonomous = true;
		manual = false;
	}


}
void MController::autonomousControl()
{

}

void MController::setLastTime()
{
 	lastTime = ros::Time::now().toSec();
}
