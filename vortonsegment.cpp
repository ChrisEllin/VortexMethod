#include "vortonsegment.h"

VelBsym:: VelBsym ()
{

}

VelBsym::VelBsym (Vector3D Velocity, double B00, double B01, double B02,double B10, double B11, double B12,double B20, double B21, double B22)
{
    Vel = Velocity;
    B[0][0]=B00;
    B[0][1]=B01;
    B[0][2]=B02;
    B[1][0]=B10;
    B[1][1]=B11;
    B[1][2]=B12;
    B[2][0]=B20;
    B[2][1]=B21;
    B[2][2]=B22;
}

VelBsym VelBsym::operator = (VelBsym C)
{
    Vel = C.Vel;
    B[0][0]=C.B[0][0];
    B[0][1]=C.B[0][1];
    B[0][2]=C.B[0][2];
    B[1][0]=C.B[1][0];
    B[1][1]=C.B[1][1];
    B[1][2]=C.B[1][2];
    B[2][0]=C.B[2][0];
    B[2][1]=C.B[2][1];
    B[2][2]=C.B[2][2];
    return *this;
}

VelBsym VelBsym:: operator + (VelBsym C)
{
    VelBsym ans = VelBsym();
    ans.Vel = Vel+C.Vel;
    ans.B[0][0]=B[0][0]+C.B[0][0];
    ans.B[0][1]=B[0][1]+C.B[0][1];
    ans.B[0][2]=B[0][2]+C.B[0][2];
    ans.B[1][0]=B[1][0]+C.B[1][0];
    ans.B[1][1]=B[1][1]+C.B[1][1];
    ans.B[1][2]=B[1][2]+C.B[1][2];
    ans.B[2][0]=B[2][0]+C.B[2][0];
    ans.B[2][1]=B[2][1]+C.B[2][1];
    ans.B[2][2]=B[2][2]+C.B[2][2];
    return ans;
}

VortonSegment::VortonSegment() : center(), tail(), vorticity(0.0), radius(0.0)
{

}

VortonSegment::VortonSegment(const Vector3D center,const Vector3D tail,const double vorticity,const double radius)
{
    this->center=center;
    this->tail=tail;
    this->vorticity=vorticity;
    this->radius=radius;
}



bool VortonSegment::operator ==(const VortonSegment &vorton) const
{
    if ((center==vorton.getCenter())&&(tail==vorton.getTail())&&
            (vorticity==vorton.getVorticity())&&(radius==vorton.getRadius())) {
        return true;
    }
    return false;
}

bool VortonSegment::operator !=(const VortonSegment &vorton) const
{
    return !(*this==vorton);
}

bool VortonSegment::operator <(const VortonSegment &vorton) const
{
    if ((tail-center).length()<(vorton.getTail()-vorton.getCenter()).length())
        return true;
    else
        return false;
}

bool VortonSegment::operator >(const VortonSegment &vorton) const
{
    return !(*this<vorton);
}

VortonSegment& VortonSegment::operator =(const VortonSegment &vorton)
{
    center=vorton.getCenter();
    tail=vorton.getTail();
    vorticity=vorton.getVorticity();
    radius=vorton.getRadius();
    return *this;
}

void VortonSegment::setCenter(const Vector3D center)
{
    this->center=center;
}

void VortonSegment::setTail(const Vector3D tail)
{
    this->tail=tail;
}

void VortonSegment::setVorticity(const double vorticity)
{
    this->vorticity=vorticity;
}

void VortonSegment::setRadius(const double radius)
{
    this->radius=radius;
}

Vector3D VortonSegment::getCenter() const
{
    return center;
}

Vector3D VortonSegment::getTail() const
{
    return tail;
}

double VortonSegment::getVorticity() const
{
    return vorticity;
}

double VortonSegment::getRadius() const
{
    return radius;
}


