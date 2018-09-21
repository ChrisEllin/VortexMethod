#ifndef VORTONSMOVEMENT_H
#define VORTONSMOVEMENT_H

#include <QVector>
#include <vortonsegment.h>
#include <QtConcurrent/QtConcurrent>
#include <QGenericMatrix>

struct IntegrationStruct
{
    VortonSegment calcingVorton;
    QVector3D movements;
    QVector<VortonSegment>* vortonsVector;
    double step;
    double maxAngle;
    double maxRadius;

    IntegrationStruct();
};

class VortonsMovement
{
public:
    VortonsMovement(const double maxRadius, const double maxAngle, const double step, const QVector3D streamSpeed);

    QVector<QVector3D> movementVector(const QVector<VortonSegment>& vortons) const;
    void eylerIntegration(QVector<VortonSegment>& vortons) const;

    static VortonSegment parallelIntegrationEyler(const IntegrationStruct& integrationStruct);
    static void addVortonsToVector(QVector<VortonSegment>& vortons, const VortonSegment &returnedVorton);
private:
    double maxDeflectionAngle;
    double maxVortonRadius;
    double integrationStep;
    QVector3D streamVelocity;
};

#endif // VORTONSMOVEMENT_H
