#ifndef FOURANGLEFRAME_H
#define FOURANGLEFRAME_H
#include "polygonalframe.h"

class FourangleFrame: public PolygonalFrame
{
public:
    FourangleFrame(QVector3D firstSide, QVector3D secondSide, QVector3D thirdSide, QVector3D fourthSide);
    double solidAngleFrame(QVector3D point);
};

#endif // FOURANGLEFRAME_H
