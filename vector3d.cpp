#include "vector3d.h"

/*!
Создает нулевой вектор
*/
Vector3D::Vector3D()
{
    X=Y=Z=0.0;
}

/*!
Создает вектор с заданными значениями 
\param _x заданная компонента X
\param _y заданная компонента Y
\param _z заданная компонента Z
*/
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

/*!
Возвращает значение длины вектора
\return Длина вектора
*/
double Vector3D::length() const
{
    return sqrt(X*X+Y*Y+Z*Z);
}

/*!
Возвращает квадрат значения длины вектора
\return Квадрат длины вектора
*/
double Vector3D::lengthSquared() const
{
    return X*X+Y*Y+Z*Z;
}

/*!
Возвращает нормированный вектор
\return Нормированный вектор
*/
Vector3D Vector3D::normalized()
{
    return  Vector3D(*this/(this->length()));
}

/*!
Нормирует вектор
*/
void Vector3D::normalize(Vector3D &vec)
{
    vec=vec/vec.length();
}

/*!
Поворачивает вектор
\param axis Ось поворота
\param theta Угол поворота
*/
void Vector3D::rotated(const Vector3D &axis, const double theta)
{
    Vector3D newaxis = axis/axis.length();
    double a = cos(theta/2.0);
    Vector3D secondary = -newaxis*sin(theta/2.0);
    double b = secondary.x();
    double c = secondary.y();
    double d = secondary.z();
    double aa = a*a;
    double bb = b*b;
    double cc = c*c;
    double dd = d*d;
    double bc = b*c;
    double ad = a*d;
    double ac = a*c;
    double ab = a*b;
    double bd = b*d;
    double cd = c*d;
    *this=Vector3D((aa+bb-cc-dd)*X+2.0*(bc+ad)*Y+2.0*(bd-ac)*Z,2.0*(bc-ad)*X+(aa+cc-bb-dd)*Y+2.0*(cd+ab)*Z, 2.0*(bd+ac)*X+2.0*(cd-ab)*Y+(aa+dd-bb-cc)*Z);
}

/*!
Устанавливает значение координаты X
\param _x Задаваемое значение X
*/
void Vector3D::x(const double _x)
{
    this->X=_x;
}

/*!
Устанавливает значение координаты Y
\param _y Задаваемое значение Y
*/
void Vector3D::y(const double _y)
{
    this->Y=_y;
}

/*!
Устанавливает значение координаты Z
\param _z Задаваемое значение Z
*/
void Vector3D::z(const double _z)
{
    this->Z=_z;
}

/*!
Возвращает значение координаты X
*/
double Vector3D::x() const
{
    return X;
}

/*!
Возвращает значение координаты Y
*/
double Vector3D::y() const
{
    return Y;
}

/*!
Возвращает значение координаты Z
*/
double Vector3D::z() const
{
    return Z;
}

/*!
Переносит вектор на величину вектора
\param translation Вектор переноса
*/
void Vector3D::translate(const Vector3D &translation)
{
    X+=translation.x();
    Y+=translation.y();
    Z+=translation.z();
}

/*!
Возвращает скалярное произведение двух векторов
\param vec1, vec2 Векторы, участвующие в произведении
\return Значение скалярного произведения
*/
double Vector3D::dotProduct(const Vector3D &vec1, const Vector3D &vec2)
{
    return vec1.x()*vec2.x()+vec1.y()*vec2.y()+vec1.z()*vec2.z();
}

/*!
Возвращает векторное произведение двух векторов
\param vec1, vec2 Векторы, участвующие в произведении
\return Значение векторное произведения
*/
Vector3D Vector3D::crossProduct(const Vector3D &vec1, const Vector3D &vec2)
{
    return Vector3D(vec1.y()*vec2.z()-vec1.z()*vec2.y(),
                    vec1.z()*vec2.x()-vec1.x()*vec2.z(),
                    vec1.x()*vec2.y()-vec1.y()*vec2.x());
}

/*!
Возвращает смешанное произведение двух векторов
\param vec1, vec2, vec3 Векторы, участвующие в произведении
\return Значение смешанного произведения
*/
double Vector3D::mixedProduct(const Vector3D &vec1, const Vector3D &vec2, const Vector3D &vec3)
{
    return Vector3D::dotProduct(vec1,Vector3D::crossProduct(vec2,vec3));
}

/*!
Переводит вектор в QVector3D
\param vec Вектор для перевода
\return Полученный вектор
*/
QVector3D Vector3D::toQVector3D(const Vector3D &vec)
{
    return QVector3D(vec.x(), vec.y(), vec.z());
}

/*!
Оператор копирования
\param vec1 Новое значение вектора
*/
void Vector3D::operator =(const Vector3D &vec1)
{
    this->X=vec1.x();
    this->Y=vec1.y();
    this->Z=vec1.z();
}

/*!
Оператор смены знака трехмерного вектора
*/
Vector3D Vector3D::operator -()
{
    return Vector3D(this->X*-1,this->Y*-1,this->Z*-1);

}

/*!
Возвращает указанную компоненту вектора
\param i Номер компоненты вектора
\return Значение компоненты вектора
*/
double& Vector3D::operator[](const int i)
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

bool Vector3D::fabsCompare(double a, double b)
{
    return (fabs(a) < fabs(b));
}

/*!
Рассчитывает сигнум-функцию от переменной
\param val Значение переменной
\return Значение сигнум-функции
*/
int Vector3D::sign(int val)
{
    return (val > 0) ? 1 : ((val < 0) ? -1 : 0);
}

/*!
Рассчитывает сигнум-функцию от переменной
\param val Значение переменной
\return Значение сигнум-функции
*/
int Vector3D::sign(double val)
{
    return (val > 0.0) ? 1 : ((val < 0.0) ? -1 : 0);
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
