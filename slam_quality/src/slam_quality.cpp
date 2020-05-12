#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <ctime>

#include "slam_quality/Map.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include <rosbag/bag.h>

nav_msgs::Path real;
nav_msgs::Path slam;
Map maps;

bool flagReal = false;
bool flagSlam = false;
bool flagMap = false;

void callbackRealPath(const nav_msgs::Path& path)
{
	real = path;
	flagReal = true;
	ROS_INFO("-----------RealPath----------");
}

void callbackSlamPath(const nav_msgs::Path& path)
{
	slam = path;
	flagSlam = true;
	ROS_INFO("-----------SLAMPath----------");
}

void callbackMap(const nav_msgs::OccupancyGrid& msg)
{
	maps.callbackNewMap(msg);
	flagMap = true;
	ROS_INFO("----------Data from map----------");
}


int main(int argc, char **argv)
{
	// check input parameters
	ros::init(argc, argv, "slam_quality");
	ros::NodeHandle n;

	ros::Subscriber s_map = n.subscribe("s_map", 1, callbackMap);
	ros::Subscriber s_real = n.subscribe("s_realPath", 1, callbackRealPath);
	ros::Subscriber s_slam = n.subscribe("s_slamPath", 1, callbackSlamPath);

	int* histogram = new int[102];
	int all = 0;
	ROS_INFO("----------Wait on data----------");
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		if (maps.getMapFlag())
		{
			for (int y = 0; y < maps.getHeight(); ++y)
			{
				for (int x = 0; x < maps.getWidth(); ++x)
				{
					int value = maps.getValue(x,y);

					if(value > 100 or value < 0)
					{
						value = 101;
					}

					histogram[value] += 1;
					all = all + 1;
				}
			}

			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("----------Save data----------");

	
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,sizeof(buffer),"%d_%m_%Y_%H_%M_%S",timeinfo);
	std::string str(buffer);

	/*Histogram*/
	std::string filename = "/home/mildoor/Desktop/";
	filename.append(str);
	std::string txt = ".txt";
	filename.append(txt);
	std::cout << "File will be save to> " << filename << std::endl;
	std::ofstream histFile(filename);
	
	for (int i = 0; i < 102 ; ++i)
	{
		if (i == 101)
			histFile << -1 << ": " << histogram[101] << std::endl;
		else
			histFile << i << ": " << histogram[i] << std::endl;
 	}
	histFile.close();

	ROS_INFO("----------Save rosbag----------");
	/*Rosbag*/
	rosbag::Bag bag;
	std::string bagname = "/home/mildoor/Desktop/";
	bagname.append(str);
	std::string b = ".bag";
	bagname.append(b);
	bag.open(bagname, rosbag::bagmode::Write);
	
	while (true)
	{
		if(flagSlam && flagReal)
		{	
			for (size_t i = 0; i < 10; i++)
			{	
				bag.write("RealTrajectory", ros::Time::now(), real);
				bag.write("SlamTrajectory", ros::Time::now(), slam);
			}
			break;
		}
	}

	bag.close();
	ROS_INFO("----------END----------");

	return 0;
};


