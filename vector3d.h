#ifndef VECTOR3D_H
#define VECTOR3D_H
#include <cmath>
#include <QMessageBox>
#include <QVector3D>
class Vector3D
{
private:
    double X;
    double Y;
    double Z;
public:
    Vector3D();
    Vector3D(const double _x,const double _y,const double _z);
    //Vector3D(const Vector3D& vec);

    double length() const;
    double lengthSquared() const;

    Vector3D normalized();
    void normalize(Vector3D& vec);

    void x(const double _x);
    void y(const double _y);
    void z(const double _z);

    double x() const;
    double y() const;
    double z() const;

    void translate(const Vector3D &translation);
    static double dotProduct(const Vector3D& vec1,const Vector3D& vec2);
    static Vector3D crossProduct(const Vector3D& vec1,const Vector3D& vec2);
    static double mixedProduct(const Vector3D& vec1, const Vector3D& vec2, const Vector3D& vec3);
    static QVector3D toQVector3D(const Vector3D& vec);
    void operator = (const Vector3D &vec1);
    Vector3D operator -();
    double& operator[](const double i);

    template <typename  T> static int sign(T val)
    {
       return (val > T(0)) ? 1 : ((val < T(0)) ? -1 : 0);
    }
};


Vector3D operator + (const Vector3D &vec1, const Vector3D &vec2);
Vector3D operator - (const Vector3D &vec1, const Vector3D &vec2);
Vector3D operator * (const Vector3D &vec1, const double value);
Vector3D operator * (const double value, const Vector3D &vec1);
Vector3D operator / (const Vector3D &vec1, const double value);
void operator += (Vector3D &vec1, const Vector3D &vec2);
void operator -= (Vector3D &vec1, const Vector3D &vec2);
void operator *= (Vector3D &vec1, const double value);
void operator /= (Vector3D &vec1, const double value);
bool operator == (const Vector3D& vec1, const Vector3D& vec2);
bool operator != (const Vector3D& vec1, const Vector3D& vec2);

#endif // VECTOR3D_H
