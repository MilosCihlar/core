#ifndef __M_CONTROLLER_H__
#define __M_CONTROLLER_H__
#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "Point.h"
#include "Position.h"
#include "Robot.h"
#include "Wheel.h"

class MController
{
private:
    Robot request;
    Robot actual;
    Robot odom;

    bool manual;
    bool autonomous;

    double pathTrackingCircle;
	std::vector<Point> trajectory;

    double lastTime = ros::Time::now().toSec();
    ros::Subscriber s_velocity;     // Topic with requested velocity
    ros::Subscriber s_path;
    ros::Subscriber s_odom;

    ros::Publisher p_leftJoint;     // Topic which control left Wheel
    ros::Publisher p_rightJoint;    // Topic which control right Wheel
    ros::Publisher p_leftVelocity;
    ros::Publisher p_rightVelocity;

private:
	int findNearsTrajectoryPoint();
	int findCircleIntersectionWithTrajectory(const int start);
	double Modulo(const double p, const double m) const;
public:
    MController(ros::NodeHandle *nh);
    MController(ros::NodeHandle *nh, const Robot& actual);
    ~MController();

	void subscribeVelocity(const geometry_msgs::Twist& v);
	void subscribePath(const nav_msgs::Path &path);
	void subscribeOdometry(const nav_msgs::Odometry& odometry);
	void manualControl();
	void autonomousControl();

    void PSDF();
    void setpointFilter();
    void sendCommand();

    bool getManual() const;
    void setManual(const bool man);
	bool getAutonomous() const;
	void setAutonomous(const bool aut);
	void setLastTime();

	void setPathTrackingCircle(const double pathTrackingCircle);
};


#endif