#ifndef __MAP_H__
#define __MAP_H__

#include <iostream>
#include "ros/ros.h"
#include "m_exploration/Point.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"



/*BUG: Nepocita s tim, ze by mapa mohla byt natocena */
/*BUG: AdjUnkow pocita s toleraci ktera je zadana primo ve funkci */
/*TIP: do checkEnvLine pridat moznost kontrolovat nejen po primce ale i prujezdnost v zavislosti na velikosti robota*/
class Map
{
private:
	int Height;
	int Width;
	double Resolution;

	double offsetX;
	double offsetY;

	int** maps;

	bool mapFlag;

public:
	/*C'tors and D'tors*/
	Map();
	Map(const int height, const int width, const double offsetX, const double offsetY, const double resolution, const int* array);
	Map(const int height, const int width, const double offsetX, const double offsetY, const double resolution);
	Map(const Map& map);
	~Map();

	/*Operators*/
	friend std::ostream& operator<<(std::ostream& aOStream, const Map& map);

	/*Methods*/
	bool checkEnvRect(const int x, const int y, const int amount, const int tolerance = 0) const;
	bool checkEnvRect(const Point& point, const int amount, const int tolerance, const int unknown = 0) const;
	bool checkEnvLine(const Point& start, const Point& end, const int tolerance = 0, const int rectAmount = 0, const int unknown = 0) const;
	bool adjUnknown(const Point& p, int tolerance) const;
	bool freee(const int x, const int y, const int tolerance = 0, const int unknown = 0) const;
	bool freee(const double x, const double y, const int tolerance = 0, const int unknown = 0) const;
	double distance(const Point& start, const Point& end) const;
	void Calibry(const double x, const double y);	
	Point randomPoint() const;
	Point getEndPoint(const Point& start, const int amount, const double convergecy, const int tolerance) const;
	void newSameMap(const int* array);
	void newMap(const int* array, const int width, const int height, const double resolution, const double offsetx, const double offsety);
	void callbackNewMap(const nav_msgs::OccupancyGrid& msg);

	/*Getter and Setter*/
	void setValue(const int x, const int y, const int value);
	void setValue(const double x, const double y, const int value);

	int getValue(const int x, const int y) const;
	int getValue(const double x, const double y) const;
	Point getPoint(const int x, const int y) const;

	int getHeight() const;
	int getWidth() const;
	double getResolution() const;
	double getOffsetX() const;
	double getOffsetY() const;
	bool getMapFlag() const;
};

#endif // !__MAP_H__