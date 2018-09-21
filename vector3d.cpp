#include "vector3d.h"

Vector3D::Vector3D()
{
    X=Y=Z=0.0;
}

Vector3D::Vector3D(const double _x, const double _y, const double _z)
{
    X=_x;
    Y=_y;
    Z=_z;
}

//Vector3D::Vector3D(const Vector3D &vec)
//{
//    X=vec.X;
//    Y=vec.Y;
//    Z=vec.Z;
//}

double Vector3D::length() const
{
    return sqrt(X*X+Y*Y+Z*Z);
}

double Vector3D::lengthSquared() const
{
    return X*X+Y*Y+Z*Z;
}

Vector3D Vector3D::normalized()
{
    return  Vector3D(*this/(this->length()));
}

void Vector3D::normalize(Vector3D &vec)
{
    vec=vec/vec.length();
}

void Vector3D::x(const double _x)
{
    this->X=_x;
}

void Vector3D::y(const double _y)
{
    this->Y=_y;
}

void Vector3D::z(const double _z)
{
    this->Z=_z;
}

double Vector3D::x() const
{
    return X;
}

double Vector3D::y() const
{
    return Y;
}

double Vector3D::z() const
{
    return Z;
}

double Vector3D::dotProduct(const Vector3D &vec1, const Vector3D &vec2)
{
    return vec1.x()*vec2.x()+vec1.y()*vec2.y()+vec1.z()*vec2.z();
}

Vector3D Vector3D::crossProduct(const Vector3D &vec1, const Vector3D &vec2)
{
    return Vector3D(vec1.y()*vec2.z()-vec1.z()*vec2.y(),
                    vec1.z()*vec2.x()-vec1.x()*vec2.z(),
                    vec1.x()*vec2.y()-vec1.y()*vec2.x());
}

double Vector3D::mixedProduct(const Vector3D &vec1, const Vector3D &vec2, const Vector3D &vec3)
{
    return Vector3D::dotProduct(vec1,Vector3D::crossProduct(vec2,vec3));
}

QVector3D Vector3D::toQVector3D(const Vector3D &vec)
{
    return QVector3D(vec.x(), vec.y(), vec.z());
}

void Vector3D::operator =(const Vector3D &vec1)
{
    this->X=vec1.x();
    this->Y=vec1.y();
    this->Z=vec1.z();
}

Vector3D Vector3D::operator -()
{
    this->X*=-1;
    this->Y*=-1;
    this->Z*=-1;
    return *this;
}

double& Vector3D::operator[](const double i)
{
    if (i==0)
        return X;
    if (i==1)
        return Y;
    if (i==2)
        return Z;
    QMessageBox::critical(new QWidget(), "Ошибка", "Указан неверный индекс");
    exit(1);
}

Vector3D operator + (const Vector3D& vec1, const Vector3D &vec2)
{
    return Vector3D(vec1.x()+vec2.x(), vec1.y()+vec2.y(), vec1.z()+vec2.z());
}

Vector3D operator - (const Vector3D &vec1, const Vector3D &vec2)
{
    return Vector3D(vec1.x()-vec2.x(),vec1.y()-vec2.y(),vec1.z()-vec2.z());
}

Vector3D operator * (const Vector3D &vec1, const double value)
{
    return Vector3D(vec1.x()*value, vec1.y()*value, vec1.z()*value);
}

Vector3D operator * (const double value, const Vector3D &vec1)
{
    return Vector3D(vec1.x()*value, vec1.y()*value, vec1.z()*value);
}

Vector3D operator / (const Vector3D &vec1, const double value)
{
    return Vector3D(vec1.x()/value, vec1.y()/value, vec1.z()/value);
}

void operator +=(Vector3D &vec1, const Vector3D &vec2)
{
    vec1.x(vec1.x()+vec2.x());
    vec1.y(vec1.y()+vec2.y());
    vec1.z(vec1.z()+vec2.z());
}

void operator -=(Vector3D &vec1, const Vector3D &vec2)
{
    vec1.x(vec1.x()-vec2.x());
    vec1.y(vec1.y()-vec2.y());
    vec1.z(vec1.z()-vec2.z());
}

void operator *=(Vector3D &vec1, const double value)
{
    vec1.x(vec1.x()*value);
    vec1.y(vec1.y()*value);
    vec1.z(vec1.z()*value);
}

void operator /=(Vector3D &vec1, const double value)
{
    vec1.x(vec1.x()/value);
    vec1.y(vec1.y()/value);
    vec1.z(vec1.z()/value);
}

bool operator ==(const Vector3D &vec1, const Vector3D &vec2)
{
    if (vec1.x()==vec2.x() &&
        vec1.y()==vec2.y() &&
        vec1.z()==vec2.z())
        return true;
    return false;
}

bool operator !=(const Vector3D &vec1, const Vector3D &vec2)
{
    return !(vec1==vec2);
}
