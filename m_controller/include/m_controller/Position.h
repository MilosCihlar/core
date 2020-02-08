#ifndef __POSITION_H__
#define __POSITION_H__

#include "Point.h"
#include "Quaternions.h"

class Position
{
private:
    Point point;
    Point angle;    
public:
    // c'tors and d"tors
    Position();
    Position(const Point& pos, const Point& angle);
    Position(const Position& euler);
    ~Position();

    Position& operator=(const Position& euler);

    Quaternions eulerToQuaternion() const;

    // Getter and Setter
    void setPoint(const Point& point);
    void setAngle(const Point& point);
    Point getPoint() const;
    Point getAngle() const;

};


#endif