#include "vector2d.h"

/*!
Создает нулевой вектор
*/
Vector2D::Vector2D()
{
    X=Y=0.0;
}

/*!
Создает вектор с заданными значениями
\param _x заданная компонента X
\param _y заданная компонента Y
*/
Vector2D::Vector2D(const double _x, const double _y)
{
    X=_x;
    Y=_y;
}

/*!
Возвращает значение длины вектора
\return Длина вектора
*/
double Vector2D::length()
{
    return sqrt(X*X+Y*Y);
}

/*!
Возвращает квадрат значения длины вектора
\return Квадрат длины вектора
*/
double Vector2D::lengthSquared()
{
    return X*X+Y*Y;
}

/*!
Возвращает нормированный вектор
\return Нормированный вектор
*/
Vector2D Vector2D::normalized()
{
    return (*this)/this->length();
}

/*!
Устанавливает значение координаты X
\param _x Задаваемое значение X
*/
void Vector2D::x(const double _x)
{
    this->X=_x;
}

/*!
Устанавливает значение координаты Y
\param _y Задаваемое значение Y
*/
void Vector2D::y(const double _y)
{
    this->Y=_y;
}

/*!
Возвращает значение координаты X
*/
double Vector2D::x() const
{
    return X;
}

/*!
Возвращает значение координаты Y
*/
double Vector2D::y() const
{
    return Y;
}

/*!
Оператор копирования
\param vec1 Новое значение вектора
*/
void Vector2D::operator =(const Vector2D &vec1)
{
    this->X=vec1.x();
    this->Y=vec1.y();
}

/*!
 Оператор смены знака
*/
Vector2D Vector2D::operator -()
{
    this->X*=-1;
    this->Y*=-1;
    return *this;
}

/*!
 Доступ к элементу двумерного вектора по индексу
 \param i Номер индекса
*/
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
