#ifndef __EXPLORATION_H__
#define __EXPLORATION_H__

#include <stdlib.h>
#include <sstream>

#include "ros/ros.h"
#include "m_exploration/Map.h"
#include "m_exploration/Point.h"
#include "m_exploration/Tree.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

/*TIP: Pridat rozmery robota aby algoritmus mohl pocitat i s velikosti*/
/*BUG: Add node pocita s toleranci */
class Exploration
{
private:
	Map maps;
	Tree tree;

	Point start;
	Point end;
	Point* nodes;
	int* finalBranch;
	Point* trajectory;

	double param;

	ros::Subscriber s_odom;
	bool odometryFlag;
	ros::Subscriber s_map;
	ros::Publisher p_trajectory;

	ros::Publisher p_marker; 
private:
	int findNearstNode(const Point& point);
	int* addNumberVisualization(int* array, const int length, const int i) const; 
public:
	/*C'tors adn D'tors*/
	Exploration(ros::NodeHandle *nh, double param);
	Exploration(double param);
	~Exploration();

	/*Methods*/
	void callbackNewStartPoint(const nav_msgs::Odometry& msg);
	void newEndPoint(const int amount,const double convergecy, const int tolerance);
	Point randomPoint() const;
	void newMap(const int* array, const int width, const int height, const double resolution, const double offsetx, const double offsety);
	bool endPath();
	void addNode(const Point& point, const int tolerance = 0, const int amount = 0);
	void sendTrajectory();

	void Visualize(const std::string) const;
	void visualizeTrajectory(const std::string world_frame) const;

	/*Setter and Getter*/
	void setParam(const double p);
	void setStartPoint(const Point& point);
	void setEndPoint(const Point& point);
	bool getOdometryFlag() const;
	bool getMapFlag() const;
	Point getEnd() const;
	Point getStart() const;

	Map getMap();

};

#endif // !__EXPLORATION_H__

