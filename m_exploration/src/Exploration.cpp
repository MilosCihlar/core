#include "m_exploration/Exploration.h"
#include "m_exploration/Map.h"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"


Exploration::Exploration(ros::NodeHandle *nh, double param):tree(), param(param), start(), end(), trajectory(nullptr), finalBranch(nullptr), odometryFlag(false)
{
	s_odom = nh->subscribe("s_odometry", 1, &Exploration::callbackNewStartPoint, this);
	s_map = nh->subscribe("s_map", 1, &Map::callbackNewMap, &maps);
	p_trajectory = nh->advertise<nav_msgs::Path>("p_trajectory", 1);
	p_marker = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);

	nodes = new Point[1];
}

Exploration::Exploration(double param) :tree(), param(param), start(), end(), trajectory(nullptr), finalBranch(nullptr)
{
	nodes = new Point[1];
}

Exploration::~Exploration()
{
	delete[] nodes;
	delete[] trajectory;
	delete[] finalBranch;
}


void Exploration::callbackNewStartPoint(const nav_msgs::Odometry& msg)
{
	Point p;
	p.setX(msg.pose.pose.position.x);
	p.setY(msg.pose.pose.position.y);
	p.setZ(0);
	start = p;

	if (!odometryFlag)
	{
		nodes[0] = p;
		maps.Calibry(p.getX(), p.getY());
	}

	int near = findNearstNode(p);
	tree.setSeed(near);
	odometryFlag = true;
}

void Exploration::newMap(const int* array, const int width, const int height, const double resolution, const double offsetx, const double offsety)
{
	maps.newMap(array, width, height, resolution, offsetx, offsety);
}

bool Exploration::endPath()
{
	int nearst = findNearstNode(end);
	double d = maps.distance(nodes[nearst], end); 
	if (d >= 2*param)
	{
		return false;
	}
	else
	{
		addNode(end);
		return true;
	}
}

int Exploration::findNearstNode(const Point& point)
{
	Point p(100000, 100000, 0);
	double distance = 10000000;
	int nearst = 0;
	for (int i = 0; i <= tree.getNumberPoint(); ++i)
	{
		double d = maps.distance(point, nodes[i]);
		if (distance > d)
		{
			p = nodes[i];
			distance = d;
			nearst = i;
		}
	}

	return nearst;

}

void Exploration::addNode(const Point& point,const int tolerance, const int amount )
{
	int nearst = findNearstNode(point);
	double vx = point.getX() - nodes[nearst].getX();
	double vy = point.getY() - nodes[nearst].getY();
	double D = maps.distance(point, nodes[nearst]);
	if (D >= param)
	{
		double t = 1/(D/param);
		Point newNode((nodes[nearst].getX() + vx*t), (nodes[nearst].getY() + vy * t), 0);
		if (maps.checkEnvLine(newNode, nodes[nearst], tolerance, amount, -1) && maps.freee(newNode.getX(), newNode.getY(), tolerance))
		{
			int length = tree.getNumberPoint();
			Point* copy = new Point[length + 1];
			for (int i = 0; i <= length; i++)
			{
				copy[i] = nodes[i];
			}
			delete[] nodes;

			nodes = new Point[length + 2];
			for (int i = 0; i <= length; i++)
			{
				nodes[i] = copy[i];
			}
			delete[] copy;
			nodes[length + 1] = newNode;
			//std::cout << "Novy node: " << newNode.getX() << " " << newNode.getY() << std::endl;
			tree.addPoint(length + 1, nearst);		
			//std::cout << "Node: " << length + 1 << " Soused: " << nearst << std::endl << std::endl;
			tree.print();
			//std::cout <<  std::endl;
		}
	}
}

int* Exploration::addNumberVisualization(int* array, const int length, const int i) const
{
	int* copy = new int[length-1];
	for (int i = 0; i < (length-1); i++)
	{
		copy[i] = array[i];
	}
	delete[] array;
	array = new int[length];

	for (int i = 0; i < (length-1); i++)
	{
		array[i] = copy[i];
	}

	array[length-1] = i;
	
	delete[] copy;
	
	return array;
}

void Exploration::Visualize(const std::string world_frame) const
{
	visualization_msgs::Marker n, points, line_strip;
    line_strip.header.frame_id = points.header.frame_id = n.header.frame_id = world_frame;
    line_strip.header.stamp = points.header.stamp = n.header.stamp = ros::Time::now();
    line_strip.ns = points.ns = n.ns = "Tree";
    line_strip.action = points.action = n.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = points.pose.orientation.w = n.pose.orientation.w = 1.0;

    line_strip.id = 0;
	points.id = 1; 
	n.id = 2;

	n.type = visualization_msgs::Marker::POINTS;

	n.scale.x = 0.03;
    n.scale.y = 0.03;

	// Nodes are red
	n.color.r = 1.0;
    n.color.a = 1.0;

	for (int i = 0; i < tree.getNumberPoint(); i++)
	{
	  	geometry_msgs::Point n1;
      	n1.x = nodes[i].getX();
      	n1.y = nodes[i].getY();
      	n1.z = 0;
	  	n.points.push_back(n1);
	}

	p_marker.publish(n);


}

void Exploration::setParam(const double p)
{
	param = p;
}



void Exploration::sendTrajectory()
{

	int l = tree.getNumberOfNode();
	finalBranch = new int[l];
	tree.getTrajectory(finalBranch);

	for (int i = 0; i < l; i++)
	{
		std::cout << finalBranch[i] << " ";
	}
	std::cout << std::endl;
	
}

void Exploration::visualizeTrajectory(const std::string world_frame) const
{
	visualization_msgs::Marker line_strip, points;
    line_strip.header.frame_id = points.header.frame_id = world_frame;
    line_strip.header.stamp = points.header.stamp = ros::Time::now();
    line_strip.ns  = points.ns  = "Tree";
    line_strip.action = points.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = points.pose.orientation.w = 1.0;

    line_strip.id = 0;
	points.id = 1;

	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	points.type = visualization_msgs::Marker::POINTS;

    line_strip.scale.x = 0.025;
	points.scale.x = 0.1;
	points.scale.y = 0.1;

    // Line list is red
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

	points.color.g = 1.0f;
	points.color.a = 1.0;

	int len = tree.getNumberOfNode();
	for (int i = 0; i < len; i++)
	{
	  	geometry_msgs::Point n1;
      	n1.x = nodes[finalBranch[i]].getX();
      	n1.y = nodes[finalBranch[i]].getY();
      	n1.z = 0;
	  	line_strip.points.push_back(n1);
	}

	geometry_msgs::Point p;
	p.x = start.getX();
	p.y = start.getY();
	p.z = 0;
	points.points.push_back(p);

	p.x = end.getX();
	p.y = end.getY();
	p.z = 0;
	points.points.push_back(p);

	p_marker.publish(points);
    p_marker.publish(line_strip);

}

void Exploration::setEndPoint(const Point& point)
{
	end = point;
}

void Exploration::setStartPoint(const Point& point)
{
	start = point;
	nodes[0] = start;
}

void Exploration::newEndPoint(const int amount,const double convergecy, const int tolerance)
{
	end = maps.getEndPoint(start, amount, convergecy, tolerance);
}

Point Exploration::randomPoint() const
{
	Point p = maps.randomPoint();
	return p;
}

bool Exploration::getOdometryFlag() const
{
	return odometryFlag;
}

bool Exploration::getMapFlag() const
{
	return maps.getMapFlag();
}

Point Exploration::getEnd() const
{
	return end;
}

Point Exploration::getStart() const
{
	return start;
}

Map Exploration::getMap()
{
	return maps;
}





