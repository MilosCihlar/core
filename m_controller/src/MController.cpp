#include <iostream>
#include <vector>
#include <math.h>
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
//	p_leftVelocity.publish(leftWheel);

	std_msgs::Float64 right;
	right.data = actual.getRightWheel().newMove(sec - lastTime);

	std_msgs::Float64 rightWheel;
	rightWheel.data = actual.getRightWheel().getVelocity();
//	p_rightVelocity.publish(rightWheel);

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

	for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = path.poses.begin(); it != path.poses.end(); ++it)
	{
		Point p(it->pose.position.x,it->pose.position.y, it->pose.position.z);
		trajectory.push_back(p);
	}
}
void MController::autonomousControl()
{
	int nearst = findNearsTrajectoryPoint();
	std::cout << "nearst: " << nearst << std::endl;
	int L = findCircleIntersectionWithTrajectory(nearst);
	std::cout << "L " << L << std::endl;

	// uhel a vzdalenost je v poradku
	if (L != -1)
	{
		double speed = sqrt(pow(trajectory[nearst].getX() - trajectory[nearst+1].getX(),2) + pow(trajectory[nearst].getY() - trajectory[nearst+1].getY(),2))/0.02;
		std::cout << "speed " << speed << std::endl;

		double fi_odom = odom.getPosition().getAngle().getZ();
		std::cout << "fi_odom " << fi_odom << std::endl;

		double fi_wanted = atan2(trajectory[L].getY() - odom.getPosition().getPoint().getY(), trajectory[L].getX() - odom.getPosition().getPoint().getX());
		std::cout << "fi_wanted " << fi_wanted << std::endl;

		std::cout << "pathCircle " << pathTrackingCircle << std::endl;
		std::cout << "angleZ " << (fi_wanted-fi_odom)/(pathTrackingCircle/speed) << std::endl;

		Point lin(speed,0,0);
		Point ang(0,0, (fi_wanted-fi_odom)/(pathTrackingCircle/speed));
		Position vel(lin, ang);
		request.setVelocity(vel);
	}
	else{

	}

	setpointFilter();
	sendCommand();
}

void MController::setLastTime()
{
 	lastTime = ros::Time::now().toSec();
}

int MController::findNearsTrajectoryPoint()
{
	int nearst;
	double distance = 9999999999;

	for(int i = 0; i < trajectory.size(); i++)
	{
		double d = sqrt(pow(trajectory[i].getX() - odom.getPosition().getPoint().getX(),2) + pow(trajectory[i].getY() - odom.getPosition().getPoint().getY(),2));
		if (d < distance)
		{
			distance = d;
			nearst = i;
		}
	}

	return nearst;
}

void MController::setPathTrackingCircle(const double pathTrackingCircle)
{
	this->pathTrackingCircle = pathTrackingCircle;
}

int MController::findCircleIntersectionWithTrajectory(const int start)
{
	int nearst = 0;
	double distance = 999999999;

	double xs =  odom.getPosition().getPoint().getX();
	double ys =  odom.getPosition().getPoint().getY();
	double x = 0;
	double y = 0;

	for (double j = 0; j < 6.283; j = j + 0.01)
	{
		x = xs + pathTrackingCircle*cos(j);
		y = ys + pathTrackingCircle*sin(j);

		for (int i = start; i < trajectory.size() - 1; ++i)
		{
			double d = sqrt(pow(trajectory[i].getX() - trajectory[i+1].getX(),2) + pow(trajectory[i].getY() - trajectory[i+1].getY(),2));

			if ( (abs(trajectory[i].getX() - x) <= d/2) && ((abs(trajectory[i].getY() - y) <= d/2)))
			{
				return i;
			}
		}
	}
	return -1;
}
