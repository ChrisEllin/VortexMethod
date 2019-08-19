#include "fourframe.h"

/*!
Создает пустую четырехугольную рамку
*/
void FourFrame::makeTriangles()
{
    triangles.push_back(MultiFrame::createTriangle(vortons[0].getTail(),vortons[1].getTail(),vortons[2].getTail(),vortons[0].getVorticity(),vortons[0].getRadius()));
    triangles.push_back(MultiFrame::createTriangle(vortons[2].getTail(),vortons[3].getTail(),vortons[0].getTail(),vortons[0].getVorticity(),vortons[0].getRadius()));
}

FourFrame::FourFrame()
{

}

/*!
Записывает рамку из заданных значений
\param r01 координата первой грани рамки
\param r11 координата второй грани рамки 
\param r21 координата третьей грани рамки
\param r31 координата четвертой грани рамки
\param eps значение радиуса вортон-отрезка
*/
FourFrame::FourFrame(Vector3D r01, Vector3D r11, Vector3D r21, Vector3D r31, double eps)
{
    vortons.resize(4);
    anglesNum=4;
    vortons[0]=Vorton((r01+r31)*0.5,r01,0,eps);
    vortons[1]=Vorton((r01+r11)*0.5,r11,0,eps);
    vortons[2]=Vorton((r11+r21)*0.5,r21,0,eps);
    vortons[3]=Vorton((r21+r31)*0.5,r31,0,eps);
    center=0.5*(vortons[0].getTail()+vortons[2].getTail());
    makeTriangles();
}


/*!
Возращает значение интенсивности от рамки в заданной точке
\param point координата точки для расчета
\return значение интенсивности
*/
Vector3D FourFrame::q(const Vector3D& point) const
{
    return (vortons[0].q(point)+vortons[1].q(point)+vortons[2].q(point)+vortons[3].q(point));
}

/*!
Возращает значение единичной интенсивности от рамки в заданной точке
\param point координата точки для расчета
\return значение единичной интенсивности
*/
Vector3D FourFrame::qHelp(const Vector3D& point) const
{
    return (vortons[0].qHelp(point)+vortons[1].qHelp(point)+vortons[2].qHelp(point)+vortons[3].qHelp(point));
}

/*!
Возращает значение скорости от рамки в заданной точке
\param point координата точки для расчета
\return значение скорости
*/
Vector3D FourFrame::velocity(const Vector3D& point) const
{
    return (vortons[0].velocity(point)+vortons[1].velocity(point)+vortons[2].velocity(point)+vortons[3].velocity(point));
}

/*!
Задает значение завихренности для рамки
\param _vorticity новое значение завихренности
*/
void FourFrame::setVorticity(const double _vorticity)
{
    vorticity=_vorticity;
    for (int i=0; i<4; i++)
        vortons[i].setVorticity(_vorticity);
}

/*!
Возращает значение скорости и тензора деформации от рамки в заданной точке
\param point заданная точка
\return значение скорости и тензора деформации
*/
VelBsym FourFrame::VelAndBsym(const Vector3D& point) const
{
    VelBsym res (Vector3D(0,0,0),0,0,0,0,0,0,0,0,0);
    for (int i=0; i<4; i++)
        res=res+vortons[i].velAndBsym(point);
    return res;
}

/*!
Возращает значение телесного угла рамки в заданной точке
\param point заданная точка
\return значение телесного угла
*/
double FourFrame::solidAngleFrame(const Vector3D& point) const
{
    return (solidAngle(vortons[0].getTail(),vortons[1].getTail(),vortons[2].getTail(),point)
            +solidAngle(vortons[0].getTail(),vortons[2].getTail(), vortons[3].getTail(),point));
}

/*!
Возвращает вортон-отрезки полученные от рамки
\return вектор вортон-отрезков
*/
QVector<Vorton> FourFrame::getVortons() const
{
    return vortons;
}

int FourFrame::getTrianglesNum()
{
    return 2;
}





