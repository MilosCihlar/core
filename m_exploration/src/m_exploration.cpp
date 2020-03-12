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
	double smooth = 0;
	nh.getParam("m_exploration/smooth", smooth);
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
	std::string world_frame;
	nh.getParam("m_exploration/world_frame", world_frame);

	Exploration explore(&nh, ratio, speed, smooth, world_frame);
  	ros::Rate loop_rate(freq);

    nav_msgs::OccupancyGrid m;
    geometry_msgs::Twist velocity;
	
  	while (ros::ok())
  	{
		if ((explore.getOdometryFlag()) && (explore.getMapFlag()))
        {        
    		explore.newEndPoint(amount, convergency, tolerance);
            Point end = explore.getEnd();

    		while (!explore.endPath())
    		{
    			Point random = explore.randomPoint();
    			explore.addNode(random, tolerance, amount);
				explore.Visualize();
			}

        	explore.sendTrajectory();
			explore.visualizeTrajectory();
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