#ifndef VORTON_H
#define VORTON_H
#include "vector3d.h"

struct VelBsym
{
    Vector3D Vel;
    double B[3][3];
    VelBsym ();
    VelBsym (Vector3D velocity);
    VelBsym (Vector3D velocity, double B00, double B01, double B02,double B10, double B11, double B12,double B20, double B21, double B22);
    VelBsym operator = (VelBsym C);
    VelBsym operator + (VelBsym C);
    VelBsym operator +=(VelBsym C);
};

class Vorton
{
private:
    Vector3D mid;
    Vector3D tail;
    double vorticity;
    double radius;
    Vector3D move;
    Vector3D elongation;
public:
    Vorton();
    Vorton(const Vector3D _mid, const Vector3D _tail, const double _vorticity, const double _radius);
    //Vorton(const Vorton& a);
    Vector3D q(const Vector3D& point) const;
    Vector3D qHelp(const Vector3D& point) const;
    Vector3D velocity(const Vector3D& point) const;
    VelBsym velAndBsym(const Vector3D& point) const;
    void turn();
    void translate(const Vector3D& translation);
    Vorton operator =(const Vorton &vort2);
    static double levi(int firstComponent, int secondComponent, int thirdComponent);
    static Vector3D rotated(const Vector3D& vec1, const Vector3D& axis, const double theta);
    void rotateAroundNormal(const Vector3D& normal);
    void setMid(const Vector3D& _mid);
    void setTail(const Vector3D &_tail);
    void setVorticity(const double _vorticity);
    void setRadius(const double _radius);
    void setElongation(const Vector3D& _elongation);
    void setMove(const Vector3D& _move);
    Vector3D getMid() const;
    Vector3D getTail() const;
    double getVorticity() const;
    double getRadius() const;
    Vector3D getElongation() const;
    Vector3D getMove() const;

};

bool operator ==(const Vorton& vort1, const Vorton& vort2);

#endif // VORTON_H
