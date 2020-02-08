#ifndef __POINT_H__
#define __POINT_H__

class Point
{
private:
    double x;
    double y;
    double z;

public:
    // c'tors and d'tors
    Point();
    Point(const double& x, const double& y, const double& z);
    Point(const double& x);
    Point(const Point& point);
    ~Point();

    Point& operator=(const Point& point);

    // Setter and Getter
    double getX() const;
    double getY() const;
    double getZ() const;
    void setX(const double& value);
    void setY(const double& value);
    void setZ(const double& value);
};


#endif