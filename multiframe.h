#ifndef MultiFrame_H
#define MultiFrame_H
#include "vorton.h"

/*!
    \brief Класс, представляющий собой многоугольную рамку

    Создает многоугольную рамку из вортон-отрезков для последующего построения сетки вокруг тела
*/

class MultiFrame
{
protected:
    int anglesNum; ///<Количество углов
    double vorticity; ///<Завихренность рамки (Гамма)
    QVector<Vorton> vortons; ///<Вектор вортон-отрезков, из которых состоит рамка
    Vector3D center; ///<Центр рамки в трехмерном пространстве
public:
    MultiFrame();
    MultiFrame(const int anglesNumber, const Vector3D& r0, const Vector3D& r01, const Vector3D& r11, const double eps);
    virtual Vorton &operator [] (std::size_t i);
    virtual Vorton& at(int i);
    virtual Vector3D q(const Vector3D& point) const;
    virtual Vector3D qHelp(const Vector3D& point) const;
    virtual Vector3D velocity(const Vector3D& point) const;
    virtual void setVorticity(const double _vorticity);
    virtual VelBsym VelAndBsym (const Vector3D& point) const;
    virtual QVector<Vector3D> getThreeEdges() const;
    virtual double solidAngleFrame(const Vector3D& point) const;
    static double solidAngle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3, const Vector3D& point);
    virtual double fi(const Vector3D& point) const;
    virtual void translate(const Vector3D& translation);
    virtual QVector<Vorton> getVortons() const;
    virtual QVector<Vorton> getLiftedVortons(const Vector3D& translation) const;
    virtual double getVorticity() const;
    virtual int getAnglesNum() const;
    virtual void setCenter(const Vector3D& _center);
    virtual Vector3D getCenter() const;
};

#endif // MultiFrame_H
