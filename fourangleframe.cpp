#include "fourangleframe.h"

FourangleFrame::FourangleFrame(QVector3D firstSide, QVector3D secondSide, QVector3D thirdSide, QVector3D fourthSide)
{
    sidesQuantity=4;
    center=(firstSide+thirdSide)*0.5;
    frameVortons.resize(sidesQuantity);
    frameVortons[0].setCenter((firstSide+fourthSide)*0.5);
    frameVortons[0].setTail(firstSide);
    frameVortons[1].setCenter((firstSide+secondSide)*0.5);
    frameVortons[1].setTail(secondSide);
    frameVortons[2].setCenter((secondSide+thirdSide)*0.5);
    frameVortons[2].setTail(thirdSide);
    frameVortons[3].setCenter((thirdSide+fourthSide)*0.5);
    frameVortons[3].setTail(fourthSide);
}

double FourangleFrame::solidAngleFrame(QVector3D point)
{
    double angle=solidAngle(frameVortons[0].getTail(),frameVortons[1].getTail(), frameVortons[2].getTail(), point)+
            solidAngle(frameVortons[0].getTail(),frameVortons[2].getTail(), frameVortons[3].getTail(), point);
    return angle;
}
