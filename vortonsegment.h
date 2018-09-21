#ifndef VORTONSEGMENT_H
#define VORTONSEGMENT_H
#include "vector3d.h"

struct VelBsym
{
    Vector3D Vel;
    double B[3][3];
    VelBsym ();
    VelBsym (Vector3D Velocity, double B00, double B01, double B02,double B10, double B11, double B12,double B20, double B21, double B22);
    VelBsym operator = (VelBsym C);
    VelBsym operator + (VelBsym C);
};




class VortonSegment
{
public:
    VortonSegment();
    VortonSegment(const Vector3D center,const  Vector3D tail,const double vorticity,const double radius);

    Vector3D velocityAtPoint(const Vector3D point) const;
    Vector3D singleIntensityVelocity(const Vector3D point) const;
    Vector3D intermediateSingleIntensityVelocity(const Vector3D point);
    double deformationTensor(const int firstComponent,const int thirdComponent,const Vector3D point) const;
    double symmetricDeformationTensor(const int firstComponent,const int thirdComponent,const Vector3D point) const;

    bool operator ==(const VortonSegment& vorton) const;
    bool operator !=(const VortonSegment& vorton) const;
    bool operator < (const VortonSegment& vorton) const;
    bool operator > (const VortonSegment& vorton) const;
    VortonSegment& operator = (const VortonSegment& vorton);

    void setCenter(const Vector3D center);
    void setTail(const Vector3D tail);
    void setVorticity(const double vorticity);
    void setRadius(const double radius);

    Vector3D getCenter() const;
    Vector3D getTail() const;
    double getVorticity() const;
    double getRadius() const;

private:
    Vector3D center;
    Vector3D tail;
    double vorticity;
    double radius;
};

#endif // VORTONSEGMENT_H
