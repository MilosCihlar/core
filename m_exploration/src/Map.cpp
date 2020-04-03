//#include "m_exploration/Map.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include "m_exploration/Map.h"

/*C'tors and D'tors*/
Map::Map():Height(0), Width(0), offsetX(0), offsetY(0), Resolution(0), maps(nullptr) {}

Map::Map(const int height, const int width, const double offsetX, const double offsetY, const double resolution, const int* array)
	: Height(height), Width(width), offsetX(offsetX), offsetY(offsetY),Resolution(resolution)
{
	maps = new int* [height];
	int index = 0;
	for (int y = 0; y < height; ++y)
	{
		maps[y] = new int[width];
		for (int x = 0; x < width; ++x)
		{
			maps[y][x] = array[index];
			index++;
		}
	}
}

Map::Map(const int height, const int width, const double offsetX, const double offsetY, const double resolution)
	: Height(height), Width(width),offsetX(offsetX), offsetY(offsetY), Resolution(resolution)
{
	maps = new int* [height];
	int index = 0;
	for (int y = 0; y < height; ++y)
	{
		maps[y] = new int[width];
		for (int x = 0; x < width; ++x)
		{
			maps[y][x] = -1;
			index++;
		}
	}
}

Map::Map(const Map& map)
{
	Height = map.Height;
	Width = map.Width;
	Resolution = map.Resolution;
	offsetX = map.offsetX;
	offsetY = map.offsetY;
	maps = new int* [Height];
	for (int y = 0; y < Height; ++y)
	{
		maps[y] = new int[Width];
		for (int x = 0; x < Width; ++x)
		{
			maps[y][x] = map.maps[y][x];
		}
	}
}

Map::~Map()
{
	for (int i = 0; i < Height; i++)
	{
		delete[] maps[i];
	}
	delete[] maps;
}

	//return maps[(int)(y / Resolution - offsetY/Resolution)][(int)(x / Resolution - offsetX/Resolution)];
/*Methods*/
bool Map::checkEnvRect(const int x, const int y, const int amount, const int tolerance) const
{
	for (int yy = y-amount; yy <= y + amount; ++yy)
	{
		for (int xx = x - amount; xx <= x + amount; ++xx)
		{
			if (!(yy < 0 || xx < 0 || yy > Height || xx > Width))
			{
				if (!freee(xx, yy, tolerance))
					return false;
			}
		}
	}
	return true;
}

bool Map::checkEnvRect(const Point& point, const int amount, const int tolerance, const int unknown) const
{
	int y = (int)(point.getY() / Resolution - offsetY/Resolution);
	int x = (int)(point.getX() / Resolution - offsetX/Resolution);

	for (int yy = y-amount; yy <= y + amount; ++yy)
	{
		for (int xx = x - amount; xx <= x + amount; ++xx)
		{
			if (!(yy < 0 || xx < 0 || yy > Height || xx > Width))
			{
				if (!freee(xx, yy, tolerance, -1))
					return false;
			}
		}
	}
	return true;
}

bool Map::checkEnvLine(const Point& start, const Point& end, const int tolerance, const int rectAmount, const int unknown) const
{
	double vx = end.getX() - start.getX();
	double vy = end.getY() - start.getY();
	double D = distance(start, end);
	double param = 1 / (D / Resolution);
	
	for (double t = 0; t <= 1; t = t + param)
	{
		Point p(start.getX() + vx*t,  start.getY() + vy * t, 0);
		//if (!freee(start.getX() + vx*t, start.getY() + vy * t, tolerance))
		if (!checkEnvRect(p, rectAmount, tolerance, unknown))
		{
			return false;
		}
	}

	return true;
}

bool Map::adjUnknown(const Point& p, int tolerance) const
{

	if (freee(p.getX(), p.getY(), tolerance))
	{
		if (((int)((p.getY() - offsetY) / Resolution) + 1 < Height) && (((int)((p.getY() - offsetY) / Resolution) - 1) >= 0) && (((int)((p.getX() - offsetX) / Resolution) + 1) < Width) && ((int)((p.getX() - offsetX) / Resolution) - 1) >= 0)
		{
			bool p2 = (100 < maps[(int)((p.getY() - offsetY) / Resolution) + 1][(int)((p.getX() - offsetX) / Resolution)    ]) || (maps[(int)((p.getY() - offsetY) / Resolution) + 1][(int)((p.getX()) / Resolution)    ] < 0);
			bool p4 = (100 < maps[(int)((p.getY() - offsetY) / Resolution)    ][(int)((p.getX() - offsetX) / Resolution) - 1]) || (maps[(int)((p.getY() - offsetY) / Resolution)    ][(int)((p.getX()) / Resolution) - 1] < 0);
			bool p5 = (100 < maps[(int)((p.getY() - offsetY) / Resolution)    ][(int)((p.getX() - offsetX) / Resolution) + 1]) || (maps[(int)((p.getY() - offsetY) / Resolution)    ][(int)((p.getX()) / Resolution) + 1] < 0);
			bool p7 = (100 < maps[(int)((p.getY() - offsetY) / Resolution) - 1][(int)((p.getX() - offsetX) / Resolution)    ]) || (maps[(int)((p.getY() - offsetY) / Resolution) - 1][(int)((p.getX()) / Resolution)    ] < 0);

			if (p2 || p4 || p5 || p7)
				return true;
			else
				return false;
		}
	}
	return false;
}

bool Map::freee(const int x, const int y, const int tolerance, const int unknown) const
{
	if (maps[y][x] >= unknown && maps[y][x] <= tolerance)
		return true;
	else
		return false;
}

bool Map::freee(const double x, const double y, const int tolerance, const int unknown) const
{
	if (maps[(int)((y)/ Resolution - offsetY/Resolution)][(int)((x)/ Resolution - offsetX/Resolution)] >= unknown && maps[(int)((y)/ Resolution - offsetY/Resolution)][(int)((x)/ Resolution - offsetX/Resolution)] <= tolerance)
		return true;

	return false;
}

double Map::distance(const Point& start, const Point& end) const
{
	return sqrt(pow( (start.getX() - end.getX()) ,2) + pow((start.getY() - end.getY()),2));
}


Point Map::randomPoint() const
{
	int x = rand() % (Width);
	int y = rand() % (Height);

	Point point = getPoint(x, y);
	return point;
}

Point Map::getEndPoint(const Point& start, const Point& first, const int amount, const double convergency, const int tolerance) const
{
	Point max = start;
	double maxLength = 0;

	bool flag = false;

	for (int y = 0; y < Height; ++y)
	{
		for (int x = 0; x < Width; ++x)
		{
			Point p = getPoint(x, y);
			if ((getValue(x,y) >= 0) && (getValue(x,y) <= tolerance))
			{
				double unknown = 0;
				double known = 0;
				double barrier = 0;

				for (int yy = y - amount; yy <= y + amount; ++yy)
				{
					for (int xx = x - amount; xx <= x + amount; ++xx)
					{
						if (!(yy < 0 || xx < 0 || yy > Height || xx > Width))
						{
							int value = getValue(xx, yy);
							if(value >= 0 && value <= tolerance)
								known = known + 1;
							else if(value == -1)
								unknown = unknown + 1;
							else
								barrier = barrier + 1;
						}
					}
				}
				
				if ((barrier == 0) && ((known/(known + unknown))*100 >= convergency) && (unknown >= 1))
				{
					double d = distance(start, p);
					if (d > maxLength)
					{
						maxLength = d;
						max = p;
						flag = true;
					}
				}
				
			}
		}
	}

	if(flag)
		return max;
	else
		return first;
}

void Map::newSameMap(const int* array)
{
	int index = 0;
	for (int y = 0; y < Height; ++y)
	{
		for (int x = 0; x < Width; ++x)
		{
			maps[y][x] = array[index];
			index++;
		}
	}
}

void Map::newMap(const int* array, const int width, const int height, const double resolution, const double offsetx, const double offsety)
{
	Resolution = resolution;
	offsetX = offsetx;
	offsetY = offsety;
	Width = width;
	Height = height;

	maps = new int* [height];
	int index = 0;
	for (int y = 0; y < height; ++y)
	{
		maps[y] = new int[width];
		for (int x = 0; x < width; ++x)
		{
			maps[y][x] = array[index];
			index++;
		}
	}

	mapFlag = true;
}

void Map::callbackNewMap(const nav_msgs::OccupancyGrid& msg)
{
	Height = msg.info.height;
	Width = msg.info.width;
	Resolution = msg.info.resolution;

	if (!mapFlag)
	{	
		offsetX = offsetX + msg.info.origin.position.x;
		offsetY = offsetY + msg.info.origin.position.y;
		
		maps = new int* [Height];
		int index = 0;
		for (int y = 0; y < Height; ++y)
		{
			maps[y] = new int[Width];
			for (int x = 0; x < Width; ++x)
			{
				maps[y][x] = msg.data[index];
				index++;
			}
		}
		mapFlag = true;
	}
	else
	{
		int index = 0;
		for (int y = 0; y < Height; ++y)
		{
			for (int x = 0; x < Width; ++x)
			{
				maps[y][x] = msg.data[index];
				index++;
			}
		}
	}
}



/*Getter and Setter*/
void Map::setValue(const int x, const int y, const int value)
{
	maps[y][x] = value;
}

void Map::setValue(const double x, const double y, const int value)
{
	maps[(int)(y / Resolution - offsetY/Resolution)][(int)(x / Resolution - offsetX/Resolution)] = value;
}

int Map::getValue(const int x, const int y) const
{
	return maps[y][x];
}

int Map::getValue(const double x, const double y) const
{
	return maps[(int)(y / Resolution - offsetY/Resolution)][(int)(x / Resolution - offsetX/Resolution)];
}

Point Map::getPoint(const int x, const int y) const
{
	Point point(x * Resolution + Resolution / 2 + offsetX, y * Resolution + Resolution / 2 + offsetY, 0);
	return point;
}

int Map::getHeight() const
{
	return Height;
}

int Map::getWidth() const
{
	return Width;
}

double Map::getResolution() const
{
	return Resolution;
}

double Map::getOffsetX() const
{
	return offsetX;
}

double Map::getOffsetY() const
{
	return offsetY;
}

std::ostream& operator<<(std::ostream& aOStream, const Map& map)
{
	for (int y = map.getHeight()-1; y >= 0; --y)
	{
		for (int x = 0; x < map.getWidth(); ++x)
		{
			aOStream << map.getValue(x,y) << " ";
		}
		aOStream << std::endl;
	}

	return aOStream;
}

bool Map::getMapFlag() const
{
	return mapFlag;
}

void Map::Calibry(const double x, const double y)
{
	offsetX = offsetX + x;
	offsetY = offsetY + y;
}

Point Map::getLocalEndPoint(const Point& start, const Point& first, const int amount, const double convergency, const int tolerance, const double len) const
{
	Point max = start;
	double maxLength = 0;

	bool flag = false;

	for (int y = 0; y < Height; ++y)
	{
		for (int x = 0; x < Width; ++x)
		{
			Point p = getPoint(x, y);
			if ((getValue(x,y) >= 0) && (getValue(x,y) <= tolerance))
			{
				double unknown = 0;
				double known = 0;
				double barrier = 0;

				for (int yy = y - amount; yy <= y + amount; ++yy)
				{
					for (int xx = x - amount; xx <= x + amount; ++xx)
					{
						if (!(yy < 0 || xx < 0 || yy > Height || xx > Width))
						{
							int value = getValue(xx, yy);
							if(value >= 0 && value <= tolerance)
								known = known + 1;
							else if(value == -1)
								unknown = unknown + 1;
							else
								barrier = barrier + 1;
						}
					}
				}

				if ((barrier == 0) && ((known/(known + unknown))*100 >= convergency) && (unknown >= 1))
				{
					double d = distance(start, p);
					if (d > maxLength && d <= len)
					{
						maxLength = d;
						max = p;
						flag = true;
					}
				}

			}
		}
	}

	if(flag)
		return max;
	else
		return start;
}
