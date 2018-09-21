#include "polygonalframe.h"

PolygonalFrame::PolygonalFrame(): sidesQuantity(0), center()
{

}

PolygonalFrame::PolygonalFrame(const int anglesQuantity, const QVector3D frameCenter, const QVector3D firstSideBeginning, const QVector3D firstSideEnding)
{
    sidesQuantity=anglesQuantity;
    center=frameCenter;
    double rotateAngle=2*M_PI/anglesQuantity;
    QVector3D a, k;
    frameVortons.resize(sidesQuantity);
    frameVortons[0].setCenter((firstSideBeginning+firstSideEnding)*0.5);
    frameVortons[0].setTail(firstSideEnding);
    frameVortons[0].setVorticity(0.0);
    frameVortons[0].setRadius(0.0);
    QVector3D rotateVector=firstSideEnding-frameCenter;
    QVector3D initialRotatePoint = (-1.0*QVector3D::crossProduct((firstSideEnding-firstSideBeginning),(firstSideBeginning-frameCenter))).normalized();
    for (int i=1; i<sidesQuantity; i++)
    {

    }
    for (int i=1; i<sidesQuantity; i++)
    {
        rotateVector=rotation(rotateVector, initialRotatePoint, 0.5*rotateAngle)*cos( 0.5*rotateAngle);
        frameVortons[i].setCenter(frameCenter+rotateVector);
        rotateVector=rotation(rotateVector, initialRotatePoint, 0.5*rotateAngle)*cos( 0.5*rotateAngle);
        frameVortons[i].setTail(frameCenter+rotateVector);
        frameVortons[i].setVorticity(0.0);
        frameVortons[i].setRadius(0.0);
    }
}

QVector3D PolygonalFrame::rotation(const QVector3D rotateVector,const QVector3D rotatePoint,const double rotateAngle) const
{
    QGenericMatrix<3,3,double> matrix;
    matrix(0,0)=cos(rotateAngle)+(1-cos(rotateAngle))*rotatePoint.x()*rotatePoint.x();
    matrix(0,1)=(1-cos(rotateAngle))*rotatePoint.x()*rotatePoint.y()-sin(rotateAngle)*rotatePoint.z();
    matrix(0,2)=(1-cos(rotateAngle))*rotatePoint.x()*rotatePoint.z()+rotatePoint.y()*sin(rotateAngle);
    matrix(1,0)=(1-cos(rotateAngle))*rotatePoint.y()*rotatePoint.x()+rotatePoint.z()*sin(rotateAngle);
    matrix(1,1)=cos(rotateAngle)+(1-cos(rotateAngle))*rotatePoint.y()*rotatePoint.y();
    matrix(1,2)=(1-cos(rotateAngle))*rotatePoint.y()*rotatePoint.z()-rotatePoint.x()*sin(rotateAngle);
    matrix(2,0)=(1-cos(rotateAngle))*rotatePoint.z()*rotatePoint.x()-rotatePoint.y()*sin(rotateAngle);
    matrix(2,1)=(1-cos(rotateAngle))*rotatePoint.z()*rotatePoint.y()+rotatePoint.x()*sin(rotateAngle);
    matrix(2,2)=cos(rotateAngle)+(1-cos(rotateAngle))*rotatePoint.z()*rotatePoint.z();

    QVector3D turnedVector;
    turnedVector.setX(matrix(0,0)*rotateVector.x()+matrix(0,1)*rotateVector.y()+matrix(0,2)*rotateVector.z());
    turnedVector.setY(matrix(1,0)*rotateVector.x()+matrix(1,1)*rotateVector.y()+matrix(2,2)*rotateVector.z());
    turnedVector.setZ(matrix(2,0)*rotateVector.x()+matrix(2,1)*rotateVector.y()+matrix(2,2)*rotateVector.z());

    return turnedVector;
}

QVector3D PolygonalFrame::velocityFromFrame(const QVector3D point) const
{
    QVector3D velocity=QVector3D();
    for (int i=0; i<frameVortons.size(); i++)
    {
        velocity+=frameVortons[i].velocityAtPoint(point);
    }
    return velocity;
}

double PolygonalFrame::maxVortonLength() const
{
    VortonSegment maxVorton=*std::max_element(frameVortons.begin(), frameVortons.end());
    double maxLength=2.0*(maxVorton.getTail()-maxVorton.getCenter()).length();
    return maxLength;
}

double PolygonalFrame::minVortonLength() const
{
    VortonSegment minVorton=*std::min_element(frameVortons.begin(), frameVortons.end());
    double minLength=2.0*(minVorton.getTail()-minVorton.getCenter()).length();
    return minLength;
}

double PolygonalFrame::solidAngle(const QVector3D firstVector, const QVector3D secondVector,const QVector3D thirdVector,const QVector3D point) const
{

        QVector3D firstVectorDistance=firstVector-point;
        QVector3D secondVectorDistance=secondVector-point;
        QVector3D thirdVectorDistance=thirdVector-point;
        double mixedproduct=QVector3D::dotProduct(firstVectorDistance,QVector3D::crossProduct(secondVectorDistance,thirdVectorDistance));
        if (mixedproduct==0)
            return 0.0;
        else
        {
            double angle;
            angle=fabs(2.0*atan(mixedproduct/(firstVectorDistance.length()*secondVectorDistance.length()*thirdVectorDistance.length()
                                            +QVector3D::dotProduct(firstVectorDistance,secondVectorDistance)*thirdVectorDistance.length()+
                                            QVector3D::dotProduct(firstVectorDistance,thirdVectorDistance)*secondVectorDistance.length()+
                                            QVector3D::dotProduct(thirdVectorDistance,secondVectorDistance)*firstVectorDistance.length())));
            return angle;
        }
}

double PolygonalFrame::solidAngleFrame(QVector3D point) const
{
    QVector3D secondVortonDistance=frameVortons[1].getTail()-point;
    QVector3D firstVortonDistance=frameVortons[0].getTail()-point;
    QVector3D thirdVortonDistance=center-point;
    if (QVector3D::dotProduct(secondVortonDistance, QVector3D::crossProduct(firstVortonDistance, thirdVortonDistance))==0)
        return 0.0;
    else
    {
        double angle=0.0;
        for (int i=0; i<sidesQuantity-1; i++)
        {
            angle+=solidAngle(frameVortons[i].getTail(), frameVortons[i+1].getTail(), center, point);
        }
        angle+=solidAngle(frameVortons[sidesQuantity-1].getTail(), frameVortons[0].getTail(), center, point);
    return angle;
    }
}

void PolygonalFrame::setFrameVortonsVorticity(const double vorticity)
{
    for (int i=0; i<sidesQuantity; i++)
        frameVortons[i].setVorticity(vorticity);
}

void PolygonalFrame::setFrameVortonsRadius(const double radius)
{
    for (int i=0; i<sidesQuantity; i++)
        frameVortons[i].setRadius(radius);
}

void PolygonalFrame::setCenter(const QVector3D newCenter)
{
    center=newCenter;
}

VortonSegment &PolygonalFrame::operator [](unsigned int i)
{
    return frameVortons[i];
}
