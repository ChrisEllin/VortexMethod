#ifndef POLYGONALFRAME_H
#define POLYGONALFRAME_H

#include <QVector>
#include "vortonsegment.h"
#include <QGenericMatrix>

class PolygonalFrame
{
public:
    PolygonalFrame();
    PolygonalFrame(const int anglesQuantity,const QVector3D frameCenter,const QVector3D firstSideBeginning,const QVector3D firstSideEnding);
    QVector3D velocityFromFrame(const QVector3D point) const;
    double maxVortonLength() const;
    double minVortonLength() const;
    double solidAngleFrame(QVector3D point) const;

    void setFrameVortonsVorticity(const double vorticity);
    void setFrameVortonsRadius(const double radius);
    void setCenter(const QVector3D newCenter);

    VortonSegment& operator [](unsigned int i);
protected:
    int sidesQuantity;
    QVector3D center;
    QVector<VortonSegment> frameVortons;
    double solidAngle(const QVector3D firstVector, const QVector3D secondVector,const QVector3D thirdVector,const QVector3D point) const;
private:
    QVector3D rotation(const QVector3D rotateVector,const QVector3D rotatePoint,const double rotateAngle) const;
};

#endif // POLYGONALFRAME_H
