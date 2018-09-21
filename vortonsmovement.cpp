#include "vortonsmovement.h"

IntegrationStruct::IntegrationStruct()
{
    calcingVorton=VortonSegment();
    movements=QVector3D();
    vortonsVector=nullptr;
    step=0.0;
    maxAngle=0.0;
    maxRadius=0.0;
}

VortonsMovement::VortonsMovement(const double maxRadius, const double maxAngle, const double step, const QVector3D streamSpeed)
{
    integrationStep=step;
    maxDeflectionAngle=maxAngle;
    maxVortonRadius=maxRadius;
    streamVelocity=streamSpeed;
}

QVector<QVector3D> VortonsMovement::movementVector(const QVector<VortonSegment> &vortons) const
{
    QVector<QVector3D> movement;
    QVector3D totalVelocity;
    for (int i=0; i<vortons.size(); i++)
    {
        totalVelocity=QVector3D();
        for (int j=0; j<vortons.size(); j++)
        {
            totalVelocity+=vortons[j].velocityAtPoint(vortons[i].getCenter());
        }
        totalVelocity+=streamVelocity;
        movement.push_back(integrationStep*totalVelocity);
    }
    return movement;
}

void VortonsMovement::eylerIntegration(QVector<VortonSegment> &vortons) const
{
    QVector<QVector3D> movement=movementVector(vortons);
    QVector<IntegrationStruct> integrationParameters;
    for (int i=0; i<vortons.size(); i++)
    {
        IntegrationStruct singleIntegrationElement{};
        singleIntegrationElement.calcingVorton=vortons[i];
        singleIntegrationElement.movements=movement[i];
        singleIntegrationElement.vortonsVector=&vortons;
        singleIntegrationElement.step=integrationStep;
        singleIntegrationElement.maxAngle=maxDeflectionAngle;
        singleIntegrationElement.maxRadius=maxVortonRadius;
        integrationParameters.push_back(singleIntegrationElement);
    }
    vortons = QtConcurrent::blockingMappedReduced(integrationParameters,
                                                                       &VortonsMovement::parallelIntegrationEyler,
                                                                       &VortonsMovement::addVortonsToVector,
                                                                       QtConcurrent::OrderedReduce);
}

VortonSegment VortonsMovement::parallelIntegrationEyler(const IntegrationStruct &integrationStruct)
{
    QGenericMatrix<3,3,double> tensorComponents;
    tensorComponents.fill(0.0);
    for (int i=0; i<integrationStruct.vortonsVector->size(); i++)
    {
        tensorComponents(0,0)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(1,1,integrationStruct.calcingVorton.getCenter());
        tensorComponents(0,1)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(1,2,integrationStruct.calcingVorton.getCenter());
        tensorComponents(0,2)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(1,3,integrationStruct.calcingVorton.getCenter());
        tensorComponents(1,0)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(2,1,integrationStruct.calcingVorton.getCenter());
        tensorComponents(1,1)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(2,2,integrationStruct.calcingVorton.getCenter());
        tensorComponents(1,2)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(2,3,integrationStruct.calcingVorton.getCenter());
        tensorComponents(2,0)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(3,1,integrationStruct.calcingVorton.getCenter());
        tensorComponents(2,1)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(3,2,integrationStruct.calcingVorton.getCenter());
        tensorComponents(2,2)+=integrationStruct.vortonsVector->
                at(i).symmetricDeformationTensor(3,3,integrationStruct.calcingVorton.getCenter());
    }

    QVector3D centerToTailDistance=integrationStruct.calcingVorton.getTail()-integrationStruct.calcingVorton.getCenter();
    QVector3D vortonElongation;

    vortonElongation.setX((tensorComponents(0,0))*centerToTailDistance.x()+(tensorComponents(0,1))*centerToTailDistance.y()
                          +(tensorComponents(0,2))*centerToTailDistance.z());
    vortonElongation.setY((tensorComponents(1,0))*centerToTailDistance.x()+(tensorComponents(1,1))*centerToTailDistance.y()
                          +(tensorComponents(1,2))*centerToTailDistance.z());
    vortonElongation.setZ((tensorComponents(2,0))*centerToTailDistance.x()+(tensorComponents(2,1))*centerToTailDistance.y()
                          +(tensorComponents(2,2))*centerToTailDistance.z());
    vortonElongation*=integrationStruct.step;

    QVector3D calcedCenterToTailDistance=integrationStruct.calcingVorton.getTail()+integrationStruct.movements+
            vortonElongation-(integrationStruct.calcingVorton.getCenter()+integrationStruct.movements);

    if((acos(QVector3D::dotProduct(centerToTailDistance,calcedCenterToTailDistance)/
             (centerToTailDistance.length()*calcedCenterToTailDistance.length()))>(integrationStruct.maxAngle))
            ||(fabs(calcedCenterToTailDistance.length()-centerToTailDistance.length())>integrationStruct.maxRadius))
        {
            vortonElongation=QVector3D();
        }
    VortonSegment returnedVorton;
    returnedVorton.setCenter(integrationStruct.calcingVorton.getCenter()+integrationStruct.movements);
    returnedVorton.setTail(integrationStruct.calcingVorton.getTail()+integrationStruct.movements+vortonElongation);
    returnedVorton.setVorticity(integrationStruct.calcingVorton.getVorticity());
    returnedVorton.setRadius(integrationStruct.calcingVorton.getRadius());
    return returnedVorton;
}

void VortonsMovement::addVortonsToVector(QVector<VortonSegment> &vortons, const VortonSegment &returnedVorton)
{
    vortons.push_back(returnedVorton);
}
