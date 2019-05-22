#include "multiframe.h"

/*!
Cоздает пустую рамку
*/
MultiFrame::MultiFrame()
{

}

/*!
Записывает рамку из заданных значений
\param anglesNumber количество углов
\param r0 координата центра рамки
\param r01 координата первого угла рамки 
\param r11 координата второго угла рамки
\param eps значение радиуса вортон-отрезка
*/

MultiFrame::MultiFrame(const int anglesNumber, const Vector3D& r0, const Vector3D& r01, const Vector3D& r11, const double eps)
{
    vortons.resize(anglesNumber);
    center=r0;
    anglesNum=anglesNumber;
    double fi = 2*M_PI/anglesNumber;
    vortons[0] = Vorton((r01+r11)*0.5,r11,0,eps);
    Vector3D a = r11-r0;
    Vector3D k = -Vector3D::crossProduct(r11-r01,r01-r0);
    for (int i=1; i<anglesNumber; i++)
    {
        a.rotated(k,fi*0.5);
        a*=cos(fi*0.5);
        Vector3D Vort_r0 = r0+a;
        a.rotated(k,fi*0.5);
        a/=cos(fi*0.5);
        vortons[i] = Vorton(Vort_r0,r0+a,0,eps);
    }
}

/*!
Возращает вортон-отрезок с заданным номером
\param i номер искомого вортон-отрезка
\return искомый вортон-отрезок
*/
Vorton &MultiFrame::operator []( std::size_t i)
{
    return vortons[i];
}

/*!
Возращает вортон-отрезок с заданным номером
\param i номер искомого вортон-отрезка
\return искомый вортон-отрезок
*/
Vorton &MultiFrame::at(int i)
{
    return vortons[i];
}

/*!
Возращает значение интенсивности от рамки в заданной точке
\param point координата точки для расчета
\return значение интенсивности
*/
Vector3D MultiFrame::q(const Vector3D& point) const
{
    Vector3D q= Vector3D();
    for (int i=0; i<anglesNum; i++)
        q+=vortons[i].q(point);
    return q;
}

/*!
Возращает значение единичной интенсивности от рамки в заданной точке
\param point координата точки для расчета
\return значение единичной интенсивности
*/
Vector3D MultiFrame::qHelp(const Vector3D& point) const
{
    Vector3D  qHelp=Vector3D ();
    for (int i=0; i<anglesNum; i++)
        qHelp=qHelp+vortons[i].qHelp(point);
    return qHelp;
}

/*!
Возращает значение скорости от рамки в заданной точке
\param point координата точки для расчета
\return значение скорости
*/
Vector3D MultiFrame::velocity(const Vector3D& point) const
{
    Vector3D  vel=Vector3D ();
    for (int i=0; i<anglesNum; i++)
        vel=vel+vortons[i].velocity(point);
    return vel;
}

/*!
Задает значение завихренности для рамки
\param _vorticity новое значение завихренности
*/
void MultiFrame::setVorticity(const double _vorticity)
{
    vorticity=_vorticity;
    for (int i=0; i<anglesNum; i++)
        vortons[i].setVorticity(_vorticity);
}

/*!
Возращает значение скорости и тензора деформации от рамки в заданной точке
\param _vorticity новое значение завихренности
\return значение скорости и тензора деформации
*/
VelBsym MultiFrame::VelAndBsym(const Vector3D& point) const
{
    VelBsym result=VelBsym(Vector3D(0,0,0),0,0,0,0,0,0,0,0,0);
    for (int i=0; i<vortons.size(); i++)
        result=result+vortons[i].velAndBsym(point);
    return result;
}

double MultiFrame::length() const
{
    double sum=0.0;
    for (int i=0; i<vortons.size(); i++)
    {
        sum+=(vortons[i].getMid()-center).length();
    }
    return sum/vortons.size();
}

/*!
Возращает три грани рамки
\return вектор, состоящий из трех концов вортон-отрезков
*/
QVector<Vector3D> MultiFrame::getThreeEdges() const
{
    QVector<Vector3D> edges;
    edges.append(vortons[0].getTail());
    edges.append(vortons[1].getTail());
    edges.append(vortons[2].getTail());
    return edges;
}

/*!
Возращает значение телесного угла рамки в заданной точке
\param point заданная точка
\return значение телесного угла
*/
double MultiFrame::solidAngleFrame(const Vector3D& point) const
{
    double teta = MultiFrame::solidAngle(vortons[anglesNum-1].getTail(),vortons[0].getTail(),center,point);
    for (int i=0; i<anglesNum-1; i++)
        teta = teta+MultiFrame::solidAngle(vortons[i].getTail(),vortons[i+1].getTail(),center,point);
    return teta;
}

/*!
Возращает значение телесного угла треугольной рамки в заданной точке
\param v1 координаты первой грани рамки
\param v2 координаты второй грани рамки
\param v3 координаты третьей грани рамки
\param point заданная точка
\return значение телесного угла
*/
double MultiFrame::solidAngle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3, const Vector3D& point)
{
    Vector3D a = v1-point;
    Vector3D b = v2-point;
    Vector3D c = v3-point;
    if (Vector3D::mixedProduct(a,b,c)==0)
        return 0;
    else
    {
        double a_m = a.length();
        double b_m = b.length();
        double c_m = c.length();
        return (fabs(2*atan(Vector3D::mixedProduct(a,b,c)/(a_m*b_m*c_m+Vector3D::dotProduct(a,c)*b_m+Vector3D::dotProduct(a,b)*c_m+Vector3D::dotProduct(b,c)*a_m))));
    }
}

/*!
Возращает значение угла fi от рамки в заданной точке
\param point заданная точка
\return значение угла fi
*/
double MultiFrame::fi(const Vector3D& point) const
{
    double res = (Vector3D::sign(Vector3D::mixedProduct(center-point, vortons[0].getTail()-point, vortons[1].getTail()-point))*
            solidAngleFrame(point)/(4*M_PI));
    return res;
}

/*!
Переносит рамку по заданному направлению
\param translation направление переноса
*/
void MultiFrame::translate(const Vector3D& translation)
{
    for (int i=0; i<vortons.size(); i++)
        vortons[i].translate(translation);
    center+=translation;
}

/*!
Возвращает вортон-отрезки полученные от рамки
\return вектор вортон-отрезков
*/
QVector<Vorton> MultiFrame::getVortons() const
{
    return vortons;
}

/*!
Возвращает поднятые вортон-отрезки полученные от рамки
\param translation направление подъема
\return вектор вортон-отрезков
*/
QVector<Vorton> MultiFrame::getLiftedVortons(const Vector3D &translation) const
{
    QVector<Vorton> vort=vortons;
    for (int i=0; i<vort.size(); i++)
        vort[i].translate(translation);
    return vort;
}

/*!
Возвращает значение завихренности рамки
\return значение завихренности
*/
double MultiFrame::getVorticity() const
{
    return vorticity;
}

/*!
Возвращает количество углов рамки
\return количество углов
*/
int MultiFrame::getAnglesNum() const
{
    return anglesNum;
}

void MultiFrame::setRadius(double radius)
{
    for (int i=0;i<vortons.size();i++)
        vortons[i].setRadius(radius);
}

double MultiFrame::getRadius() const
{
    return vortons[0].getRadius();
}

/*!
Устанавливает значение центра рамки
\param _center новая координата центра
*/
void MultiFrame::setCenter(const Vector3D &_center)
{
    center=_center;
}

/*!
Возвращает значение центра рамки
\return Координата центра рамки
*/
Vector3D MultiFrame::getCenter() const
{
    return center;
}


