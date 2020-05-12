#include <iostream>

#include "ros/ros.h"
#include <sstream>

#include "m_exploration/Map.h"
#include "m_exploration/Tree.h"
#include "m_exploration/Exploration.h"


/*BUG: Kdzy se pohnu a pridam novz node a nastavim nove seminko (seed) tak pri sendTrajectory vznika chyba memory corrupt  , nebo exit code -11, pri startu se to nedeje ... */

int main(int argc, char **argv)
{
  	// Init m_exploration node
  	ros::init(argc, argv, "m_exploration");
    ROS_INFO("/***/ Start m_exploratin node /***/");
  	ros::NodeHandle nh;

  	// Settings
	double ratio = 0;
	nh.getParam("m_exploration/ratio", ratio);
	double convergency = 0;
	nh.getParam("m_exploration/convergency", convergency);
	double tolerance = 0;
	nh.getParam("m_exploration/tolerance", tolerance);
	double amount = 0;
	nh.getParam("m_exploration/amount", amount);
	double speed = 0;
	nh.getParam("m_exploration/speed", speed);
  	double freq = 0;
  	nh.getParam("m_exploration/freq", freq);
	double lidar = 0;
  	nh.getParam("m_exploration/LidarRange", lidar);
	std::string world_frame;
	nh.getParam("m_exploration/world_frame", world_frame);

	Exploration explore(&nh, ratio, speed, world_frame);
  	ros::Rate loop_rate(freq);

    nav_msgs::OccupancyGrid m;
    geometry_msgs::Twist velocity;

	bool f = true;

  	while (ros::ok())
  	{
		if ((explore.getOdometryFlag()) && (explore.getMapFlag()))
		{
			if(f or sqrt(pow(explore.getStart().getX() - explore.getEnd().getX(),2) + pow(explore.getStart().getY() - explore.getEnd().getY(),2)) <= 3*ratio)
			{
				explore.newLocalEndPoint(amount, convergency, tolerance, (lidar - 4*ratio));
				if ((explore.getStart().getX() == explore.getEnd().getX()) && (explore.getStart().getY() == explore.getEnd().getY()))
					explore.newEndPoint(amount, convergency, tolerance);
			
				f = false;
			}

			for (int i = 0; i < 1000; i++)	
			{
    			Point random = explore.randomPoint();
    			explore.addNode(random, tolerance, amount);
				explore.VisualizeTree();
				explore.VisualizeStartEndPoint();
			}

			explore.VisualizeStartEndPoint();

			if(explore.endPath())
			{
				explore.sendTrajectory();
				explore.visualizeTrajectory();
			}
        }
        else
        {
            ROS_INFO("--Wait on data--");
        }
    	
	    ros::spinOnce();
    	loop_rate.sleep();
  	}

	return 0;
}