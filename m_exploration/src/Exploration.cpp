#include "m_exploration/Exploration.h"
#include "m_exploration/Map.h"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


Exploration::Exploration(ros::NodeHandle *nh, double param, double speed, double smooth, std::string world_frame)
:nh(nh), tree(), param(param), start(), end(), trajectory(nullptr), finalBranch(nullptr), odometryFlag(false), speed(speed), worldFrame(world_frame)
{
	s_odom = nh->subscribe("s_odometry", 1, &Exploration::callbackNewStartPoint, this);
	s_map = nh->subscribe("s_map", 1, &Map::callbackNewMap, &maps);
	p_trajectory = nh->advertise<nav_msgs::Path>("p_trajectory", 1, false);
	p_marker = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
	p_marker_array = nh->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	nodes = new Point[1];

	if(smooth > 1)
	{
		smooth = 1;
	}
	else if(smooth < 0)
	{
		smooth = 0;
	} else
		this->smooth = smooth;
}

Exploration::Exploration(double param, double speed, std::string world_frame)
:tree(), param(param), start(), end(), trajectory(nullptr), finalBranch(nullptr), speed(speed), worldFrame(world_frame)
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
	//std::cout << "odometrie" << std::endl;

	if (!odometryFlag)
	{
		first.setX(msg.pose.pose.position.x);
		first.setY(msg.pose.pose.position.y);
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
	if (d > param)
	{
		return false;
	}
	else
	{
		addNode(end);
		return true;
	}
}

int Exploration::findNearstNode(const Point& point) const
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
			//tree.print();
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
void Exploration::VisualizeStartEndPoint() const
{
	visualization_msgs::Marker n, points;
	points.header.frame_id = n.header.frame_id = worldFrame;
	points.header.stamp = n.header.stamp = ros::Time::now();
	points.ns = n.ns = "StartEndPoint";
	points.action = n.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = n.pose.orientation.w = 1.0;

	points.id = 1;
	n.id = 2;
	n.type = visualization_msgs::Marker::POINTS;
	points.type = visualization_msgs::Marker::POINTS;

	n.scale.x = 0.04;
	n.scale.y = 0.04;
	points.scale.x = 0.1;
	points.scale.y = 0.1;

	n.color.r = 1.0;
	n.color.a = 1.0;
	points.color.g = 1.0f;
	points.color.a = 1.0;

	for (int i = 0; i < tree.getNumberPoint(); i++)
	{
		geometry_msgs::Point n1;
		n1.x = nodes[i].getX();
		n1.y = nodes[i].getY();
		n1.z = 0;
		n.points.push_back(n1);
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
	p_marker.publish(n);


}

void Exploration::VisualizeTree() const
{

	Tree copyTree = tree;
	int* nextStart = new int[2];
	nextStart[0] = 1;
	nextStart[1] = 0;
	int node = 0;
	int next = 0;
	int l = 0;

	while (true)
	{
		while (next != -1)
		{
			next = copyTree.getFirstNeighbor(node);
			if (next != -1)
			{
				if (copyTree.getNumberNeighbor(node) >= 2)
				{
					int* copy = new int[nextStart[0]+1];
					for (int i = 0; i < nextStart[0] + 1; i++)
					{
						copy[i] = nextStart[i];
					}
					delete[] nextStart;
					nextStart = new int[copy[0] + 2];
					for (int i = 0; i < copy[0] + 1; i++)
					{
						nextStart[i] = copy[i];
					}
					nextStart[0] = copy[0] + 1;
					nextStart[copy[0] + 1] = node;

					delete[] copy;
				}

				copyTree.removeNeighbor(node, next);
				copyTree.removeNeighbor(next, node);

				node = next;
			}
		}
		next = 0;

		node = nextStart[nextStart[0]];

		int* copy = new int[nextStart[0]];
		for (int i = 0; i < nextStart[0]; i++)
		{
			copy[i] = nextStart[i];
		}
		delete[] nextStart;
		nextStart = new int[copy[0]];
		for (int i = 0; i < copy[0]; i++)
		{
			nextStart[i] = copy[i];
		}
		nextStart[0] = copy[0] - 1;
		delete[] copy;

		if (nextStart[0] == 0)
		{
			break;
		}

		l += 1;
	}
	delete[] nextStart;

	visualization_msgs::MarkerArray tre;
	tre.markers.resize(l + 1);
	for (int i = 0; i <= l; ++i)
	{

		tre.markers[i].header.frame_id = worldFrame;
		tre.markers[i].header.stamp = ros::Time::now();
		tre.markers[i].ns = "Tree";
		tre.markers[i].action = visualization_msgs::Marker::ADD;
		tre.markers[i].pose.orientation.w = 1.0;
		tre.markers[i].id = i;
		tre.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
		tre.markers[i].scale.x = 0.03;
		tre.markers[i].color.r = 1.0;
		tre.markers[i].color.a = 1.0;

	}

	Tree copyT = tree;
	nextStart = new int[2];
	nextStart[0] = 1;
	nextStart[1] = 0;
	node = 0;
	next = 0;
	l = 0;

	while (true)
	{
		while (next != -1)
		{
			geometry_msgs::Point p;
			p.x = nodes[node].getX();
			p.y = nodes[node].getY();
			p.z = 0;
			tre.markers[l].points.push_back(p);
			next = copyT.getFirstNeighbor(node);
			if (next != -1)
			{
				if (copyT.getNumberNeighbor(node) >= 2)
				{
					int* copy = new int[nextStart[0]+1];
					for (int i = 0; i < nextStart[0] + 1; i++)
					{
						copy[i] = nextStart[i];
					}
					delete[] nextStart;
					nextStart = new int[copy[0] + 2];
					for (int i = 0; i < copy[0] + 1; i++)
					{
						nextStart[i] = copy[i];
					}
					nextStart[0] = copy[0] + 1;
					nextStart[copy[0] + 1] = node;

					delete[] copy;
				}

				copyT.removeNeighbor(node, next);
				copyT.removeNeighbor(next, node);
				node = next;
			}
		}

		l += 1;
		next = 0;

		node = nextStart[nextStart[0]];

		int* copy = new int[nextStart[0]];
		for (int i = 0; i < nextStart[0]; i++)
		{
			copy[i] = nextStart[i];
		}
		delete[] nextStart;
		nextStart = new int[copy[0]];
		for (int i = 0; i < copy[0]; i++)
		{
			nextStart[i] = copy[i];
		}
		nextStart[0] = copy[0] - 1;
		delete[] copy;

		if (nextStart[0] == 0)
		{
			break;
		}

	}

	delete[] nextStart;

	p_marker_array.publish(tre);
}

void Exploration::setParam(const double p)
{
	param = p;
}



void Exploration::sendTrajectory()
{
	int e = findNearstNode(end);
	int l = tree.getNumberOfNode(e);
	finalBranch = new int[l];
	tree.getTrajectory(finalBranch, e);
	smoothTrajectory(l);
}

void Exploration::visualizeTrajectory() const
{
	visualization_msgs::Marker line_strip, points;
    line_strip.header.frame_id = points.header.frame_id = worldFrame;
    line_strip.header.stamp = points.header.stamp = ros::Time::now();
    line_strip.ns  = points.ns  = "Trajectory";
    line_strip.action = points.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = points.pose.orientation.w = 1.0;

    line_strip.id = 0;

	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
    line_strip.scale.x = 0.06;

	line_strip.color.r = 1.0;
	line_strip.color.a = 1.0;

	int e = findNearstNode(end);
	int len = tree.getNumberOfNode(e);

	for (int i = 0; i < len; i++)
	{
	  	geometry_msgs::Point n1;
      	n1.x = nodes[finalBranch[i]].getX();
      	n1.y = nodes[finalBranch[i]].getY();
      	n1.z = 0;
	  	line_strip.points.push_back(n1);
	}

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
	end = maps.getEndPoint(start, first, amount, convergecy, tolerance);
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

void Exploration::smoothTrajectory(const int lenTraj)
{
	static int id = 0;
	//p_trajectory.shutdown();

	trajectory = new Point[lenTraj];
	Point p;
	double px, py;
	Point d;
	double dx, dy;
	Point t;
	double tx, ty;
	Point c;
	double cx, cy;

	Point newPoint;
	nav_msgs::Path path;

	path.poses.clear();
	int res = 0;
	for (int i = 0; i < (lenTraj-3); i = i + 3)
	{
		p = nodes[finalBranch[i]];
		px = p.getX(); py = p.getY();
		d = nodes[finalBranch[i+1]];
		dx = d.getX(); dy = d.getY();
		t = nodes[finalBranch[i+2]];
		tx = t.getX(); ty = t.getY();
		c = nodes[finalBranch[i+3]];
		cx = c.getX(); cy = c.getY();

		res = i;

		double d1 = maps.distance(p, d);
		double d2 = maps.distance(d, t);
		double d3 = maps.distance(t, c);
		double D = d1 + d2 + d3;
		double Dn = 1.2*D;

		double time = Dn/speed;
		double param = 0.02/time;

		for (double t = 0; t < 1; t = t + param)
		{
			id += 1;
			double x = (-px + 3*dx -3*tx + cx)*t*t*t + (3*px -6*dx + 3*tx)*t*t + (-3*px + 3*dx)*t + px;
			double y = (-py + 3*dy -3*ty + cy)*t*t*t + (3*py -6*dy + 3*ty)*t*t + (-3*py + 3*dy)*t + py;

			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = x;
			pose.pose.position.y = y;
			pose.pose.position.z = 0;
			path.poses.push_back(pose);
			path.header.seq = id;
		}

	}


	double Dn = 1.2*maps.distance(nodes[finalBranch[res+3]], nodes[finalBranch[lenTraj-1]]);
	double time = Dn/speed;
	double param = 0.02/time;

	double vx = nodes[finalBranch[lenTraj-1]].getX() - nodes[finalBranch[res+3]].getX();
	double vy = nodes[finalBranch[lenTraj-1]].getY() - nodes[finalBranch[res+3]].getY();

	for (double t = 0; t < 1; t = t + param)
	{
		id += 1;
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = nodes[finalBranch[res+3]].getX() + vx*t;
		pose.pose.position.y = nodes[finalBranch[res+3]].getY() + vy*t;
		pose.pose.position.z = 0;
		path.poses.push_back(pose);
		path.header.seq = id;
	}

	id += 1;
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = nodes[finalBranch[lenTraj-1]].getX();
	pose.pose.position.y = nodes[finalBranch[lenTraj-1]].getY();
	pose.pose.position.z = 0;
	path.poses.push_back(pose);
	path.header.seq = id;
	path.header.frame_id = worldFrame;

	p_trajectory.publish(path);
}





