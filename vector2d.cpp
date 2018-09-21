#include "vector2d.h"

Vector2D::Vector2D()
{
    X=Y=0.0;
}

Vector2D::Vector2D(const double _x, const double _y)
{
    X=_x;
    Y=_y;
}

double Vector2D::length()
{
    return sqrt(X*X+Y*Y);
}

double Vector2D::lengthSquared()
{
    return X*X+Y*Y;
}

Vector2D Vector2D::normalized()
{
    return (*this)/this->length();
}

void Vector2D::x(const double _x)
{
    this->X=_x;
}

void Vector2D::y(const double _y)
{
    this->Y=_y;
}

double Vector2D::x() const
{
    return X;
}

double Vector2D::y() const
{
    return Y;
}

void Vector2D::operator =(const Vector2D &vec1)
{
    this->X=vec1.x();
    this->Y=vec1.y();
}

Vector2D Vector2D::operator -()
{
    this->X*=-1;
    this->Y*=-1;
    return *this;
}

double &Vector2D::operator[](const double i)
{
    if (i==0)
        return X;
    if (i==1)
        return Y;
    QMessageBox::critical(new QWidget(), "Ошибка", "Указан неверный индекс");
    exit(1);
}

Vector2D operator +(const Vector2D &vec1, const Vector2D &vec2)
{
    return Vector2D(vec1.x()+vec2.x(), vec1.y()+vec2.y());
}

Vector2D operator -(const Vector2D &vec1, const Vector2D &vec2)
{
    return Vector2D(vec1.x()-vec2.x(),vec1.y()-vec2.y());
}

Vector2D operator *(const Vector2D &vec1, const double value)
{
    return Vector2D(vec1.x()*value, vec1.y()*value);
}

Vector2D operator *(const double value, const Vector2D &vec1)
{
    return Vector2D(vec1.x()*value, vec1.y()*value);
}

Vector2D operator /(const Vector2D &vec1, const double value)
{
    return Vector2D(vec1.x()/value, vec1.y()/value);
}

void operator +=(Vector2D &vec1, const Vector2D &vec2)
{
    vec1.x(vec1.x()+vec2.x());
    vec1.y(vec1.y()+vec2.y());
}

void operator -=(Vector2D &vec1, const Vector2D &vec2)
{
    vec1.x(vec1.x()-vec2.x());
    vec1.y(vec1.y()-vec2.y());
}

void operator *=(Vector2D &vec1, const double value)
{
    vec1.x(vec1.x()*value);
    vec1.y(vec1.y()*value);
}

void operator /=(Vector2D &vec1, const double value)
{
    vec1.x(vec1.x()/value);
    vec1.y(vec1.y()/value);
}

bool operator ==(const Vector2D &vec1, const Vector2D &vec2)
{
    if (vec1.x()==vec2.x() && vec1.y()==vec2.y())
        return true;
    return false;
}

bool operator !=(const Vector2D &vec1, const Vector2D &vec2)
{
    return !(vec1==vec2);
}
