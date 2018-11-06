#ifndef VECTOR3D_H
#define VECTOR3D_H
#include <cmath>
#include <QMessageBox>
#include <QVector3D>

/*!
    \brief Класс, реализующий трехмерный вектор с double-компонентами
*/
class Vector3D
{
private:
    double X; ///<X компонента вектора
    double Y; ///<Y компонента вектора
    double Z; ///<Z компонента вектора
public:
    Vector3D();
    Vector3D(const double _x,const double _y,const double _z);
    //Vector3D(const Vector3D& vec);

    double length() const;
    double lengthSquared() const;

    Vector3D normalized();
    void normalize(Vector3D& vec);

    void rotated (const Vector3D& axis, const double theta);
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
    static bool fabsCompare(double a, double b);
    static int sign(int val);
    static int sign(double val);
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
