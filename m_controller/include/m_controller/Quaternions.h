#ifndef __QUATERNIONS_H__
#define __QUATERNIONS_H__

class Quaternions
{
private:
    double x;
    double y;
    double z;
    double w;

public:
    // c'tors and d'tors
    Quaternions();
    Quaternions(const double& x, const double& y, const double& z, const double& w);
    Quaternions(const double& x);
    Quaternions(const Quaternions& quat);
    ~Quaternions();

    Quaternions& operator=(const Quaternions& point);

    // Setter and Getter
    double getX() const;
    double getY() const;
    double getZ() const;
    double getW() const;
    void setX(const double& value);
    void setY(const double& value);
    void setZ(const double& value);
    void setW(const double& value);
};


#endif