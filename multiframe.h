﻿#ifndef MultiFrame_H
#define MultiFrame_H
#include "vorton.h"

/*!
    \brief Класс, представляющий собой многоугольную рамку
 
    Создает многоугольную рамку из вортон-отрезков для последующего построения сетки вокруг тела
*/

enum VortonsPart {Beginning, Center, Tail};

struct FramesSizes
{
    double minFrameSize;
    double averFrameSize;
    double maxFrameSize;
};


class TriangleFrame
{
private:
    Vector3D r0;
    Vector3D r1;
    Vector3D r2;
    Vector3D normal;
    Vector3D center;
public:
    TriangleFrame();
    TriangleFrame(Vector3D _r0, Vector3D _r1, Vector3D _r2, Vector3D _normal, Vector3D _center);
    void setR0(const Vector3D _r0);
    void setR1(const Vector3D _r1);
    void setR2(const Vector3D _r2);
    void setNormal(const Vector3D _normal);
    void setCenter(const Vector3D _center);

    Vector3D getR0() const;
    Vector3D getR1() const;
    Vector3D getR2() const;
    Vector3D getNormal() const;
    Vector3D getCenter() const;

    bool intersection(Vector3D ra, Vector3D rb) const;
    bool inside(Vector3D ra, Vector3D rb) const;
    bool colinear(Vector3D a, Vector3D b) const;
    double solidAngle(Vector3D r);
};

class MultiFrame
{
protected:
    int anglesNum; ///<Количество углов
    double vorticity; ///<Завихренность рамки (Гамма)
    QVector<Vorton> vortons; ///<Вектор вортон-отрезков, из которых состоит рамка
    Vector3D center; ///<Центр рамки в трехмерном пространстве
    QVector<TriangleFrame> triangles;

public:
    MultiFrame();
    virtual void makeTriangles();
    virtual ~MultiFrame();
    MultiFrame(const int anglesNumber, const Vector3D& r0, const Vector3D& r01, const Vector3D& r11, const double eps);
    virtual Vorton &operator [] (std::size_t i);
    virtual Vorton& at(int i);
    virtual Vector3D q(const Vector3D& point) const;
    virtual Vector3D qHelp(const Vector3D& point) const;
    virtual Vector3D velocity(const Vector3D& point) const;
    virtual void setVorticity(const double _vorticity);
    virtual VelBsym VelAndBsym (const Vector3D& point) const;
    virtual double length() const;
    virtual QVector<Vector3D> getThreeEdges() const;
    virtual double solidAngleFrame(const Vector3D& point) const;
    static double solidAngle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3, const Vector3D& point);
    virtual double fi(const Vector3D& point) const;
    virtual void translate(const Vector3D& translation);
    virtual QVector<Vorton> getVortons() const;
    virtual QVector<Vorton> getLiftedVortons(const Vector3D& translation) const;
    virtual double getVorticity() const;
    virtual int getAnglesNum() const;
    virtual void setRadius(double radius);
    virtual double getRadius() const;
    virtual void setCenter(const Vector3D& _center);
    virtual Vector3D getCenter() const;
    static TriangleFrame createTriangle(Vector3D r01, Vector3D r11, Vector3D r21, double vorticity, double eps);
    virtual TriangleFrame getTriangle(int num);
    virtual int intersection(Vector3D ra,Vector3D rb) const;
    virtual bool inside(Vector3D ra, Vector3D rb, int choosenNum, bool checking=false) const;
    virtual int getTrianglesNum();
    static Vector3D bissektCenter(Vector3D r1, Vector3D r2, Vector3D r3);
};

struct FrameData
{
    MultiFrame frame;
    Vector3D controlPoint;
    Vector3D controlPointRaised;
    Vector3D normal;
    double square;

    bool full;
    void clear();
};

#endif // MultiFrame_H
