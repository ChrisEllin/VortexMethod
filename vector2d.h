#ifndef VECTOR2D_H
#define VECTOR2D_H
#include <QMessageBox>
#include <cmath>

/*!
    \brief Класс, реализующий двумерный вектор с double-компонентами 
*/
class Vector2D
{
private:
    double X; ///<X компонента вектора
    double Y; ///<Y компонента вектора
public:
    Vector2D();
    Vector2D(const double _x, const double _y);

    double length();
    double lengthSquared();
    Vector2D normalized();
    void x(const double _x);
    void y(const double _y);

    double x() const;
    double y() const;

    void operator = (const Vector2D &vec1);
    Vector2D operator -();
    double& operator[](const double i);
};

Vector2D operator + (const Vector2D &vec1, const Vector2D &vec2);
Vector2D operator - (const Vector2D &vec1, const Vector2D &vec2);
Vector2D operator * (const Vector2D &vec1, const double value);
Vector2D operator * (const double value, const Vector2D &vec1);
Vector2D operator / (const Vector2D &vec1, const double value);
void operator += (Vector2D &vec1, const Vector2D &vec2);
void operator -= (Vector2D &vec1, const Vector2D &vec2);
void operator *= (Vector2D &vec1, const double value);
void operator /= (Vector2D &vec1, const double value);
bool operator == (const Vector2D& vec1, const Vector2D& vec2);
bool operator != (const Vector2D& vec1, const Vector2D& vec2);

#endif // VECTOR2D_H
