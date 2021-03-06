﻿#ifndef FOURFRAME_H
#define FOURFRAME_H
#include "multiframe.h"
#include <memory>

/*!
    \brief Класс, представляющий собой четырехугольную рамку 

    Создает четырехугольную рамку из вортон-отрезков для последующего построения сетки вокруг тела. Четырехугольная рамка отнаследована от многоугольной рамки.
*/
class FourFrame : public MultiFrame
{
private:
    void makeTriangles();
public:
    FourFrame();
    FourFrame(Vector3D r01, Vector3D r11, Vector3D r21, Vector3D r31, double eps);
    Vector3D q(const Vector3D& point) const;
    Vector3D qHelp(const Vector3D& point) const;
    Vector3D velocity(const Vector3D& point) const;
    void setVorticity(const double _vorticity);
    VelBsym VelAndBsym (const Vector3D& point) const;
    double solidAngleFrame(const Vector3D& point) const;
    QVector<Vorton> getVortons() const;
    int getTrianglesNum();
};

#endif // FOURFRAME_H
