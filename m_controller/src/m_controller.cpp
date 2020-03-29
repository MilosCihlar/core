#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "m_controller/MController.h"

#include "m_controller/Point.h"
#include "m_controller/Position.h"
#include "m_controller/Robot.h"
#include "m_controller/Wheel.h"

int main(int argc, char **argv)
{
  // Init m_controller control node
  ros::init(argc, argv, "m_controller");
  ros::NodeHandle nh;

  // Set qrequency from launch file
  double rate = 0;
  nh.getParam("m_control/freq", rate);
  double radius = 0;
  nh.getParam("m_control/radius", radius);
  double span = 0;
  nh.getParam("m_control/span", span);

  double x = 0;
  nh.getParam("m_control/x", x);
  double y = 0;
  nh.getParam("m_control/y", y);
  double z = 0;
  nh.getParam("m_control/z", z);

  double pathTrackingCircle = 0;
  nh.getParam("m_control/pathTrackingCircle", pathTrackingCircle);

  std::cout << "pathTrackingparraaaam " << pathTrackingCircle << std::endl;

  Wheel wheel(radius);
  Point point(x,y,z);
  Point angle(0,0,0);
  Position position(point, angle);
  Position velocity;

  Robot robot(wheel, position, velocity, span);

  MController controller(&nh, robot);
  controller.setPathTrackingCircle(pathTrackingCircle);

  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
	int type = 0;
	if(controller.getManual())
	{
		type = 1;
	}
	else if(controller.getAutonomous())
	{
		type = 2;
	}

  	switch(type)
	{
		case(0):
			controller.setLastTime();
			break;
		case(1):
			controller.manualControl();
			break;
		//case(2):
		//	controller.autonomousControl();
		//	break;
		default:
			controller.setLastTime();
			break;
	}

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
