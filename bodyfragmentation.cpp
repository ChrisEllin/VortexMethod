#include "bodyfragmentation.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
/*!
Установка соотвествующего значения в объект структуры параметров разбиения сферы 
\param i Номер поля структуры
\param value Значение поля структуры 
*/
void SphereParameters::setData(const int i, const double value)
{
    switch (i)
    {
    case 0:
        fiFragNum=static_cast<int>(value+0.5);
        break;
    case 1:
        tetaFragNum=static_cast<int>(value+0.5);
        break;
    case 2:
        radius=static_cast<int>(value+0.5);
        break;
    case 3:
        delta=value;
        break;
    case 4:
        raise=value;
        break;
    case 5:
        vortonsRad=value;
        break;
    }
}

/*!
Установка соотвествующего значения в объект структуры параметров разбиения цилиндра
\param i Номер поля структуры
\param value Значение поля структуры
*/
void CylinderParameters::setData(const int i, const double value)
{
    switch (i)
    {
    case 0:
        fiFragNum=static_cast<int>(value+0.5);
        break;
    case 1:
        radFragNum=static_cast<int>(value+0.5);
        break;
    case 2:
        heightFragNum=static_cast<int>(value+0.5);
        break;
    case 3:
        diameter=value;
        break;
    case 4:
        height=value;
        break;
    case 5:
        delta=value;
        break;
    case 6:
        raise=value;
        break;
    case 7:
        vortonsRad=value;
        break;
    }
}

/*!
Установка соотвествующего значения в объект структуры параметров разбиения тела вращения
\param i Номер поля структуры
\param value Значение поля структуры
*/
void RotationBodyParameters::setData(const int i, const double value)
{
    switch (i)
    {
    case 0:
        fiFragNum=static_cast<int>(value+0.5);
        break;
    case 1:
        partFragNum=static_cast<int>(value+0.5);
        break;
    case 2:
        xBeg=value;
        break;
    case 3:
        xEnd=value;
        break;
    case 4:
        sectionDistance=value;
        break;
    case 5:
        delta=value;
        break;
    case 6:
        raise=value;
        break;
    case 7:
        vortonsRad=value;
        break;
    }
}

/*!
Установка соотвествующего значения в объект структуры параметров разбиения тела вращения со срезом дна
\param i Номер поля структуры
\param value Значение поля структуры
*/
void RotationCutBodyParameters::setData(const int i, const double value)
{
    switch (i)
    {
    case 0:
        fiFragNum=static_cast<int>(value+0.5);
        break;
    case 1:
        partFragNum=static_cast<int>(value+0.5);
        break;
    case 2:
        rFragNum=static_cast<int>(value+0.5);
        break;
    case 3:
        xBeg=value;
        break;
    case 4:
        xEnd=value;
        break;
    case 5:
        sectionDistance=value;
        break;
    case 6:
        delta=value;
        break;
    case 7:
        raise=value;
        break;
    case 8:
        vortonsRad=value;
        break;
    }
}

/*!
Разбивает тело, определяя его тип.
\param body Тип тела
\param param Параметры разбиения
\param launch Необходимость решения задачи старта
*/

BodyFragmentation::BodyFragmentation(BodyType body, const FragmentationParameters &param, bool launch)
{
    switch(body)
    {
    case SPHERE:
    {
        sphere = SphereParameters {param.sphereFiFragNum, param.sphereTetaFragNum, param.sphereRad, param.delta, param.pointsRaising,param.vortonsRad};
        streamLinesSize=QPair<int,int>(static_cast<int>(ceil(param.sphereRad*2)),static_cast<int>(ceil(param.sphereRad)));
        sphereFragmentation();
        break;
    }
    case CYLINDER:
    {
        cylinder = CylinderParameters {param.cylinderFiFragNum, param.cylinderRadFragNum, param.cylinderHeightFragNum,param.cylinderDiameter, param.cylinderHeight,  param.delta, param.pointsRaising,param.vortonsRad};
        cylinderFragmentation();
        break;
    }
    case ROTATIONBODY:
    {
        rotationBody = RotationBodyParameters {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,param.rotationBodySectionEndDistance,  param.delta, param.pointsRaising, param.vortonsRad};
        forming=FormingParameters{param.formingDiameter,param.formingTailDiameter, param.formingLengthSectorOne,param.formingLengthSectorTwo,param.formingAngle,param.rotationBodyFormingType};
        streamLinesSize=QPair<int,int>(static_cast<int>(ceil(param.formingDiameter*0.5+param.formingLengthSectorOne+param.formingLengthSectorTwo)),static_cast<int>(ceil(param.formingDiameter*0.5)));
        rotationBodyFragmantation();
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        if (!launch)
        {
            rotationBottomCutBody = RotationCutBodyParameters {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum,param.rotationBodyRFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,param.rotationBodySectionEndDistance,  param.delta, param.pointsRaising, param.vortonsRad};
            formingRBC=FormingParametersRBC{param.formingEllipsoidDiameter,param.formingTailDiameter,param.formingEllisoidLength,param.formingConeLength,param.formingFullLength,param.rotationBodyRBCFormingType};
            streamLinesSize=QPair<int,int>(static_cast<int>(ceil(param.formingFullLength)),static_cast<int>(ceil(param.formingEllipsoidDiameter*0.5)));
            rotationCutBodyFragmantation();
        }
        else
        {
            rotationBottomCutBody = RotationCutBodyParameters {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum,param.rotationBodyRFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,param.rotationBodySectionEndDistance,  param.delta, param.pointsRaising, param.vortonsRad};
            formingRBC=FormingParametersRBC{param.formingEllipsoidDiameter,param.formingTailDiameter,param.formingEllisoidLength,param.formingConeLength,param.formingFullLength,param.rotationBodyRBCFormingType};
            streamLinesSize=QPair<int,int>(static_cast<int>(ceil(param.formingFullLength)),static_cast<int>(ceil(param.formingEllipsoidDiameter*0.5)));
        }
        break;
    }
    }
}

//BodyFragmentation::BodyFragmentation(const FragmentationParameters &param, const int i, const Vector3D &bodyVel, const double tau)
//{
//    RotationCutBodyParameters rotBodyCutPar {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum,param.rotationBodyRFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,  param.delta, param.pointsRaising, param.vortonsRad};
//    rotationCutBodyLaunchFragmentation(rotBodyCutPar,i,bodyVel,tau);

//}

/*!
Реализует разбиение сферы.
*/

void BodyFragmentation::sphereFragmentation()
{
    clearVectors();
    double fi0=2*M_PI/sphere.fiFragNum;
    double tetaMulti=M_PI/(sphere.tetaFragNum+2);
    double teta0=(M_PI-2*tetaMulti)/sphere.tetaFragNum;
    double R=sphere.radius+sphere.delta;
    for (int i=0; i<sphere.fiFragNum; i++)
    {
        for (int j=0; j<sphere.tetaFragNum; j++)
        {
            double fi=fi0*i;
            double teta=tetaMulti+teta0*j;
            Vector3D vec0(R*sin(teta)*cos(fi),R*sin(teta)*sin(fi),R*cos(teta));
            Vector3D vec1(R*sin(teta)*cos(fi+fi0),R*sin(teta)*sin(fi+fi0),R*cos(teta));
            Vector3D vec2(R*sin(teta+teta0)*cos(fi+fi0),R*sin(teta+teta0)*sin(fi+fi0),R*cos(teta+teta0));
            Vector3D vec3(R*sin(teta+teta0)*cos(fi),R*sin(teta+teta0)*sin(fi),R*cos(teta+teta0));
            frames.push_back(std::make_shared<FourFrame>(vec0,vec1,vec2,vec3,sphere.vortonsRad));
            controlPoints.push_back(Vector3D(sphere.radius*sin(teta+0.5*teta0)*cos(fi+0.5*fi0),sphere.radius*sin(teta+0.5*teta0)*sin(fi+0.5*fi0),sphere.radius*cos(teta+0.5*teta0)));
            normals.push_back(controlPoints.last()/sphere.radius);
            squares.push_back(0.5*Vector3D::crossProduct(vec2-vec0,vec3-vec1).length());
            controlPointsRaised.push_back(Vector3D((sphere.radius+sphere.raise)*sin(teta+0.5*teta0)*cos(fi+0.5*fi0),(sphere.radius+sphere.raise)*sin(teta+0.5*teta0)*sin(fi+0.5*fi0),(sphere.radius+sphere.raise)*cos(teta+0.5*teta0)));
        }
    }
    double teta=M_PI-tetaMulti;
    Vector3D vec0(0.0,0.0,R*cos(teta));
    Vector3D vec1(R*sin(teta)*cos(fi0),R*sin(teta)*sin(fi0),R*cos(teta));
    Vector3D vec2(R*sin(teta)*cos(2*fi0),R*sin(teta)*sin(2*fi0),R*cos(teta));
    frames.push_back(std::make_shared<MultiFrame>(sphere.fiFragNum, vec0,vec1,vec2,sphere.vortonsRad));
    controlPoints.push_back(Vector3D(0.0,0.0,sphere.radius*cos(teta)));
    normals.push_back(Vector3D(0.0,0.0,-1.0));
    squares.push_back(0.5*Vector3D::crossProduct(vec1-vec0,vec2-vec0).length()*sphere.fiFragNum);
    controlPointsRaised.push_back(Vector3D(0.0,0.0,(sphere.radius+sphere.raise)*cos(teta)));

    teta0 = tetaMulti;
    vec0 = Vector3D(0,0,R*cos(teta0));
    vec1 = Vector3D(R*sin(teta0)*cos(fi0),R*sin(teta0)*sin(fi0),R*cos(teta0));
    vec2 = Vector3D(R*sin(teta0)*cos(2*fi0),R*sin(teta0)*sin(2*fi0),R*cos(teta0));
    frames.push_back(std::make_shared<MultiFrame>(sphere.fiFragNum,vec0,vec2,vec1,sphere.vortonsRad));
    controlPoints.push_back(Vector3D(0,0,sphere.radius*cos(teta0)));
    normals.push_back (Vector3D(0.0,0.0,1.0));
    squares.push_back((0.5*Vector3D::crossProduct(vec1-vec0,vec2-vec0).length()*sphere.fiFragNum));
    controlPointsRaised.push_back(Vector3D(0.0,0.0,(sphere.radius+sphere.raise)*cos(teta0)));
}

/*!
Реализует разбиение цилиндра.
*/
void BodyFragmentation::cylinderFragmentation()
{
    clearVectors();

    double R=0.5*cylinder.diameter+cylinder.delta;
    double fi0=2*M_PI/cylinder.fiFragNum;
    double r0=(cylinder.diameter+2.0*cylinder.delta)/(2.0*cylinder.radFragNum);
    double h0=(cylinder.height+2.0*cylinder.delta)/cylinder.heightFragNum;
    Vector3D vec0 (0.0,-cylinder.delta,0.0);
    Vector3D vec1 (r0,-cylinder.delta,0.0);
    Vector3D vec2 (r0*cos(fi0),-cylinder.delta,r0*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(cylinder.fiFragNum,vec0,vec2,vec1,cylinder.vortonsRad));
    controlPoints.push_back(Vector3D());
    normals.push_back(Vector3D(0.0,-1.0,0.0));
    squares.append(0.5*(vec1-vec0).lengthSquared()*cylinder.fiFragNum*sin(2*M_PI/cylinder.fiFragNum));
    controlPointsRaised.push_back(controlPoints.last()+normals.last()*cylinder.raise);

    for (int i=1; i<cylinder.radFragNum; i++)
    {
        for (int j=0; j<cylinder.fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = r0*i;
            Vector3D vec01 (r*cos(fi),-cylinder.delta,r*sin(fi));
            Vector3D vec11 (r*cos(fi+fi0),-cylinder.delta,r*sin(fi+fi0));
            Vector3D vec21 ((r+r0)*cos(fi+fi0),-cylinder.delta,(r+r0)*sin(fi+fi0));
            Vector3D vec31 ((r+r0)*cos(fi),-cylinder.delta,(r+r0)*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(vec01,vec11,vec21,vec31,cylinder.vortonsRad));
            controlPoints.push_back(Vector3D((r+r0*0.5)*cos(fi+fi0*0.5),0.0,(r+r0*0.5)*sin(fi+fi0*0.5)));
            normals.push_back(Vector3D (0.0,-1.0,0.0));
            squares.push_back(Vector3D::crossProduct(vec21-vec01,vec31-vec11).length()*0.5);
            controlPointsRaised.push_back(controlPoints.last()+normals.last()*cylinder.raise);
        }
    }

    vec0=Vector3D(0.0,cylinder.height+cylinder.delta,0.0);
    vec1=Vector3D(r0,cylinder.height+cylinder.delta,0.0);
    vec2=Vector3D(r0*cos(fi0),cylinder.height+cylinder.delta,r0*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(cylinder.fiFragNum,vec0,vec1,vec2,cylinder.vortonsRad));
    controlPoints.push_back(Vector3D(0.0,cylinder.height,0.0));
    normals.push_back(Vector3D(0.0,1.0,0.0));
    squares.push_back(0.5*(vec1-vec0).lengthSquared()*cylinder.fiFragNum*sin(2*M_PI/cylinder.fiFragNum));
    controlPointsRaised.push_back(controlPoints.last()+normals.last()*cylinder.raise);

    for (int i=1; i<cylinder.radFragNum; i++)
    {
        for (int j=0; j<cylinder.fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = r0*i;
            Vector3D vec01(r*cos(fi),cylinder.height+cylinder.delta,r*sin(fi));
            Vector3D vec11 (r*cos(fi+fi0),cylinder.height+cylinder.delta,r*sin(fi+fi0));
            Vector3D vec21((r+r0)*cos(fi+fi0),cylinder.height+cylinder.delta,(r+r0)*sin(fi+fi0));
            Vector3D vec31((r+r0)*cos(fi),cylinder.height+cylinder.delta,(r+r0)*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(vec01,vec31,vec21,vec11,cylinder.vortonsRad));
            controlPoints.push_back(Vector3D((r+r0*0.5)*cos(fi+fi0*0.5),cylinder.height,(r+r0*0.5)*sin(fi+fi0*0.5)));
            normals.push_back(Vector3D (0.0,1.0,0.0));
            squares.push_back(Vector3D::crossProduct(vec21-vec01,vec31-vec11).length()*0.5);
            controlPointsRaised.push_back(controlPoints.last()+normals.last()*cylinder.raise);
        }
    }

    for (int i=0; i<cylinder.fiFragNum; i++)
    {
        for (int j=0; j<cylinder.heightFragNum; j++)
        {
            double s = h0*j-cylinder.delta;
            double fi=fi0*i;
            Vector3D vec0(R*cos(fi),s,R*sin(fi));
            Vector3D vec1(R*cos(fi+fi0),s,R*sin(fi+fi0));
            Vector3D vec2(R*cos(fi+fi0),s+h0,R*sin(fi+fi0));
            Vector3D vec3(R*cos(fi),s+h0,R*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(vec0,vec1,vec2,vec3,cylinder.vortonsRad));
            controlPoints.push_back(Vector3D(0.5*cylinder.diameter*cos(fi+fi0*0.5),s+h0*0.5,0.5*cylinder.diameter*sin(fi+fi0*0.5)));
            normals.push_back(Vector3D(cos(fi+fi0*0.5),0.0,sin(fi+fi0*0.5)));
            squares.push_back((Vector3D::crossProduct(vec2-vec0,vec3-vec1)).length()*0.5);
            controlPointsRaised.push_back(controlPoints.last()+normals.last()*cylinder.raise);

        }
    }
}

QPair<int, int> BodyFragmentation::getStreamLinesSizes()
{
    return streamLinesSize;
}

/*!
Реализует разбиение тела вращения.
*/
void BodyFragmentation::rotationBodyFragmantation()
{
    clearVectors();
    switch (forming.typeNum)
    {
    case 0:
    {
        rotationBody.xEnd=rotationBody.xBeg+forming.diameter*0.5+forming.sectorOneLength+forming.sectorTwoLength;
        break;
    }
    case 1:
    {
        rotationBody.xEnd=rotationBody.xBeg+forming.sectorOneLength;
        break;
    }
    case 2:
    {
        rotationBody.xEnd=rotationBody.xBeg+forming.sectorOneLength;
        break;
    }
    }
    QVector<Vector2D> part(rotationBody.partFragNum);
    QVector<Vector2D> forNormals(rotationBody.partFragNum-1);
    QVector<Vector2D> forControlPoint(rotationBody.partFragNum-1);
    QVector<Vector2D> forUp(rotationBody.partFragNum);
    const int NFRAG=400;

    QVector<double> s(NFRAG);
    QVector<double> xArr(NFRAG);
    QVector<double> yArr(NFRAG);

    double newBeg=rotationBody.xBeg+rotationBody.sectionDistance;
    double newEnd=rotationBody.xEnd-rotationBody.sectionEndDistance;
    double fi0 = 2*M_PI/rotationBody.fiFragNum;
    double height=(newEnd-newBeg)/(NFRAG-1);
    s[0]=0.0;
    xArr[0]=newBeg;
    yArr[0]=BodyFragmentation::presetFunctionF(newBeg,forming);
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunctionF(xArr[i],forming);
        double derivative=BodyFragmentation::presetDeriveFunctionF(xArr[i],forming);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }
    //    QFile curving("curving.csv");
    //    if (curving.open(QIODevice::WriteOnly))
    //    {
    //        QTextStream ts(&curving);
    //        for (int i=0; i<NFRAG;i++)
    //        {
    //            ts<<QString::number(xArr[i])+"\t"+QString::number(yArr[i])+"\n";
    //        }
    //    }
    //    curving.close();

    double length=s[s.size()-1];
    for (int i=0; i<rotationBody.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotationBody.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionF(xArr[num],forming),1).normalized();
    }

    for (int i=0; i<rotationBody.partFragNum-1; i++)
    {
        int translNum=findClosetElementFromArray(s,length/(rotationBody.partFragNum-1)*(i+0.5));
        forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
        forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionF(xArr[translNum],forming),1).normalized();
    }

    for (int i=0; i<part.size(); i++)
        part[i]+=forUp[i]*rotationBody.delta;

    for (int j=0; j<rotationBody.partFragNum-1; j++)
    {
        for (int i=0; i<rotationBody.fiFragNum; i++)
        {
            double fi=fi0*i;
            Vector3D r01=Vector3D(part[j].x(),part[j].y()*cos(fi),part[j].y()*sin(fi));
            Vector3D r11=Vector3D(part[j].x(),part[j].y()*cos(fi+fi0),part[j].y()*sin(fi+fi0));
            Vector3D r21=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi+fi0),part[j+1].y()*sin(fi+fi0));
            Vector3D r31=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi),part[j+1].y()*sin(fi));
            Vector3D controlPoint = Vector3D(forControlPoint[j].x(),forControlPoint[j].y()*cos(fi+fi0*0.5),forControlPoint[j].y()*sin(fi+fi0*0.5));
            Vector3D normal = Vector3D(forNormals[j].x(),forNormals[j].y()*cos(fi+fi0*0.5),forNormals[j].y()*sin(fi+fi0*0.5));
            frames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,rotationBody.vortonsRad));
            normals.push_back(normal);
            controlPoints.push_back(controlPoint);
            squares.push_back((0.5*Vector3D::crossProduct(r21-r01,r31-r11).length()));
            controlPointsRaised.push_back(controlPoint+rotationBody.raise*normal);
        }
    }

    Vector3D r0 = Vector3D(part[0].x(),0,0);
    Vector3D r11 = Vector3D(part[0].x(),part[0].y()*cos(fi0),part[0].y()*sin(fi0));
    Vector3D r21 = Vector3D(part[0].x(),part[0].y(),0);
    frames.push_back(std::make_shared<MultiFrame>(rotationBody.fiFragNum,r0,r11,r21,rotationBody.vortonsRad));
    Vector3D controlPoint = Vector3D(part[0].x(),0,0);
    controlPoints.push_back(controlPoint);
    Vector3D normal (-1,0,0);
    normals.push_back(normal);
    squares.push_back((0.5*Vector3D::crossProduct(r11-r0,r21-r0).length()*rotationBody.fiFragNum));
    controlPointsRaised.push_back(controlPoint+rotationBody.raise*normal);

    r0 = Vector3D(part[rotationBody.partFragNum-1].x(),0,0);
    r11 = Vector3D(part[rotationBody.partFragNum-1].x(),part[rotationBody.partFragNum-1].y(),0);
    r21 = Vector3D(part[rotationBody.partFragNum-1].x(),part[rotationBody.partFragNum-1].y()*cos(fi0),part[rotationBody.partFragNum-1].y()*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(rotationBody.fiFragNum,r0,r11,r21,rotationBody.vortonsRad));
    controlPoint = Vector3D(part[rotationBody.partFragNum-1].x(),0,0);
    controlPoints.push_back(controlPoint);
    normal =Vector3D (1,0,0);
    normals.push_back(normal);
    squares.push_back((0.5*Vector3D::crossProduct(r11-r0,r21-r0).length()*rotationBody.fiFragNum));
    controlPointsRaised.push_back(controlPoint+rotationBody.raise*normal);
}

/*!
Реализует разбиение тела вращения со срезом дна.
*/
void BodyFragmentation::rotationCutBodyFragmantation()
{
    clearVectors();
    switch (formingRBC.type)
    {
    case ELLIPSOID_CONE:
    {
        rotationBottomCutBody.xEnd=rotationBottomCutBody.xBeg+formingRBC.ellipsoidLength+formingRBC.coneLength;
        break;
    }
    case ELLIPSOID_CYLINDER:
    {
        rotationBottomCutBody.xEnd=rotationBottomCutBody.xBeg+formingRBC.fullLength;
        break;
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        rotationBottomCutBody.xEnd=rotationBottomCutBody.xBeg+formingRBC.fullLength;
        break;
    }
    }
    QVector<Vector2D> part(rotationBottomCutBody.partFragNum);
    QVector<Vector2D> forNormals(rotationBottomCutBody.partFragNum);
    QVector<Vector2D> forControlPoint(rotationBottomCutBody.partFragNum);
    QVector<Vector2D> forUp(rotationBottomCutBody.partFragNum);
    const int NFRAG=400;

    QVector<double> s(NFRAG);
    QVector<double> xArr(NFRAG);
    QVector<double> yArr(NFRAG);

    double newBeg=rotationBottomCutBody.xBeg+rotationBottomCutBody.sectionDistance;

    double newEnd=rotationBottomCutBody.xEnd+rotationBottomCutBody.delta;
    double fi0 = 2*M_PI/rotationBottomCutBody.fiFragNum;
    double height=(newEnd-newBeg)/(NFRAG-1);
    s[0]=0.0;
    xArr[0]=newBeg;
    yArr[0]=BodyFragmentation::presetFunctionG(newBeg,formingRBC);
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunctionG(xArr[i],formingRBC);
        double derivative=BodyFragmentation::presetDeriveFunctionG(xArr[i],formingRBC);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }

    double length=s[s.size()-2];
    for (int i=0; i<rotationBottomCutBody.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[num],formingRBC),1).normalized();
    }

    for (int i=0; i<rotationBottomCutBody.partFragNum-1; i++)
    {
        int translNum=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*(i+0.5));
        forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
        forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[translNum],formingRBC),1).normalized();
    }

    for (int i=0; i<part.size(); i++)
        part[i]+=forUp[i]*rotationBottomCutBody.delta;

    for (int i=0; i<part.size(); i++)
    {
        part[i]-=Vector2D(part[0].x()-rotationBottomCutBody.xBeg,0.0);
        if (i!=part.size()-1)
            forControlPoint[i]-=Vector2D(part[part.size()-1].x()-rotationBottomCutBody.xEnd,0.0);
    }

    for (int j=0; j<rotationBottomCutBody.partFragNum-1; j++)
    {
        for (int i=0; i<rotationBottomCutBody.fiFragNum; i++)
        {
            double fi=fi0*i;
            Vector3D r01=Vector3D(part[j].x(),part[j].y()*cos(fi),part[j].y()*sin(fi));
            Vector3D r11=Vector3D(part[j].x(),part[j].y()*cos(fi+fi0),part[j].y()*sin(fi+fi0));
            Vector3D r21=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi+fi0),part[j+1].y()*sin(fi+fi0));
            Vector3D r31=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi),part[j+1].y()*sin(fi));
            Vector3D controlPoint = Vector3D(forControlPoint[j].x(),forControlPoint[j].y()*cos(fi+fi0*0.5),forControlPoint[j].y()*sin(fi+fi0*0.5));
            Vector3D normal = Vector3D(forNormals[j].x(),forNormals[j].y()*cos(fi+fi0*0.5),forNormals[j].y()*sin(fi+fi0*0.5));
            frames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,rotationBottomCutBody.vortonsRad));
            normals.push_back(normal);
            controlPoints.push_back(controlPoint);
            squares.push_back((0.5*Vector3D::crossProduct(r21-r01,r31-r11).length()));
            controlPointsRaised.push_back(controlPoint+rotationBottomCutBody.raise*normal);
        }
    }

    Vector3D r0 = Vector3D(part[0].x(),0.0,0.0);
    Vector3D r11 = Vector3D(part[0].x(),part[0].y()*cos(fi0),part[0].y()*sin(fi0));
    Vector3D r21 = Vector3D(part[0].x(),part[0].y(),0.0);
    frames.push_back(std::make_shared<MultiFrame>(rotationBottomCutBody.fiFragNum,r0,r11,r21,rotationBottomCutBody.vortonsRad));
    Vector3D controlPoint = Vector3D(part[0].x(),0.0,0.0);
    controlPoints.push_back(controlPoint);
    Vector3D normal (-1,0,0);
    normals.push_back(normal);
    squares.push_back((0.5*Vector3D::crossProduct(r11-r0,r21-r0).length()*rotationBottomCutBody.fiFragNum));
    controlPointsRaised.push_back(controlPoint+rotationBottomCutBody.raise*normal);

    double rad0=part.last().y()/rotationBottomCutBody.rFragNum;

    r0 = Vector3D(part[rotationBottomCutBody.partFragNum-1].x(), 0.0, 0.0);
    r11 = Vector3D(part[rotationBottomCutBody.partFragNum-1].x(), rad0, 0.0);
    r21 = Vector3D(part[rotationBottomCutBody.partFragNum-1].x(), rad0*cos(fi0),rad0*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(rotationBottomCutBody.fiFragNum,r0,r11,r21,rotationBottomCutBody.vortonsRad));
    controlPoint = Vector3D(part[rotationBottomCutBody.partFragNum-1].x()-rotationBottomCutBody.delta,0.0,0);
    controlPoints.push_back(controlPoint);
    normal =Vector3D (1,0,0);
    normals.push_back(normal);
    squares.push_back(0.5*(r11-r0).lengthSquared()*rotationBottomCutBody.fiFragNum*sin(2*M_PI/rotationBottomCutBody.fiFragNum));
    controlPointsRaised.push_back(controlPoint+rotationBottomCutBody.raise*normal);

    for (int i=1; i<rotationBottomCutBody.rFragNum; i++)
    {
        for (int j=0; j<rotationBottomCutBody.fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = rad0*i;
            Vector3D r11 (part[rotationBottomCutBody.partFragNum-1].x(),r*cos(fi),r*sin(fi));
            Vector3D r01 (part[rotationBottomCutBody.partFragNum-1].x(),r*cos(fi+fi0),r*sin(fi+fi0));
            Vector3D r31 (part[rotationBottomCutBody.partFragNum-1].x(),(r+rad0)*cos(fi+fi0),(r+rad0)*sin(fi+fi0));
            Vector3D r21 (part[rotationBottomCutBody.partFragNum-1].x(),(r+rad0)*cos(fi),(r+rad0)*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,rotationBottomCutBody.vortonsRad));
            controlPoint = Vector3D(part[rotationBottomCutBody.partFragNum-1].x()-rotationBottomCutBody.delta,(r+0.5*rad0)*cos(fi+0.5*fi0),(r+rad0*0.5)*sin(fi+fi0*0.5));
            controlPoints.push_back(controlPoint);
            normal = Vector3D (1,0,0);
            normals.push_back(normal);
            squares.push_back((Vector3D::crossProduct(r21-r01,r31-r11)).length()*0.5);
            controlPointsRaised.append(controlPoint+rotationBottomCutBody.raise*normal);
        }
    }
}

/*!
Реализует разбиение тела вращения со срезом дна для решения задачи старта.
\param i Текущий шаг
\param bodyVel Скорость тела
\param tau Размер шага
*/
void BodyFragmentation::rotationCutBodyLaunchFragmentation(const int i, const Vector3D& bodyVel, const double tau,const double fullLength)
{

    double ledge=bodyVel.length()*tau*(i+1);

    if (fabs(ledge)<fullLength)
    {
        clearVectors();
        switch (formingRBC.type)
        {
        case ELLIPSOID_CONE:
        {
            rotationBottomCutBody.xEnd=rotationBottomCutBody.xBeg+formingRBC.ellipsoidLength+formingRBC.coneLength;
            break;
        }
        case ELLIPSOID_CYLINDER:
        {
            rotationBottomCutBody.xEnd=rotationBottomCutBody.xBeg+formingRBC.fullLength;
            break;
        }
        case ELLIPSOID_CYLINDER_CONE:
        {
            rotationBottomCutBody.xEnd=rotationBottomCutBody.xBeg+formingRBC.fullLength;
            break;
        }
        }
        //qDebug()<<rotationBottomCutBody.xEnd;
        QVector<Vector2D> part(rotationBottomCutBody.partFragNum);
        QVector<Vector2D> forNormals(rotationBottomCutBody.partFragNum-1);
        QVector<Vector2D> forControlPoint(rotationBottomCutBody.partFragNum-1);
        QVector<Vector2D> forUp(rotationBottomCutBody.partFragNum);
        const int NFRAG=400;

        QVector<double> s(NFRAG);
        QVector<double> xArr(NFRAG);
        QVector<double> yArr(NFRAG);

        double newBeg=rotationBottomCutBody.xBeg+rotationBottomCutBody.sectionDistance;
        double newEnd=rotationBottomCutBody.xEnd-rotationBottomCutBody.delta;
        double fi0 = 2*M_PI/rotationBottomCutBody.fiFragNum;
        double height=(newEnd-newBeg)/(NFRAG-1);
        s[0]=0.0;
        xArr[0]=newBeg;
        yArr[0]=BodyFragmentation::presetFunctionG(newBeg,formingRBC);
        for (int i=1; i<NFRAG; i++)
        {
            xArr[i]=newBeg+i*height;
            yArr[i]=BodyFragmentation::presetFunctionG(xArr[i],formingRBC);
            double derivative=BodyFragmentation::presetDeriveFunctionG(xArr[i],formingRBC);
            s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
        }
        //        for (int i=0; i<xArr.size();i++)
        //            xArr[i]-=xArr[xArr.size()-1];

        double length=s[s.size()-2];
        for (int i=0; i<rotationBottomCutBody.partFragNum; i++)
        {
            int num=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*i);
            part[i]=Vector2D(xArr[num],yArr[num]);
            forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[num],formingRBC),1).normalized();
        }

        for (int i=0; i<rotationBottomCutBody.partFragNum-1; i++)
        {
            int translNum=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*(i+0.5));
            forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
            forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[translNum],formingRBC),1).normalized();
        }
        int end=rotationBottomCutBody.partFragNum-1;
        double currentEnd=part[end].x();
        while (currentEnd>ledge)
        {
            part.removeLast();
            forUp.removeLast();
            forControlPoint.removeLast();
            forNormals.removeLast();
            end--;
            currentEnd=part[end].x();
        }
        //        part.removeLast();
        //        forUp.removeLast();
        //        forControlPoint.removeLast();
        //        forNormals.removeLast();
        //        end--;

        part.push_back(Vector2D(ledge, BodyFragmentation::presetFunctionG(ledge,formingRBC)));
        forUp.push_back(Vector2D(-BodyFragmentation::presetDeriveFunctionG(ledge,formingRBC),1.0).normalized());
        forControlPoint.push_back(Vector2D(0.5*(part[end]+part[end+1])));
        forNormals.push_back(Vector2D(-BodyFragmentation::presetDeriveFunctionG(forControlPoint.last().x(),formingRBC),1).normalized());


        for (int i=0; i<part.size(); i++)
            part[i]+=forUp[i]*rotationBottomCutBody.delta;

        for (int i=0; i<part.size(); i++)
        {
            part[i]-=Vector2D(part[part.size()-1].x(),0.0);
            if (i!=part.size()-1)
                forControlPoint[i]-=Vector2D(part[part.size()-1].x(),0.0);
        }
        int newPartFragmNum = part.size()-1;
        // qDebug()<<"ya zhiv1";
        for (int j=0; j<newPartFragmNum; j++)
        {
            for (int i=0; i<rotationBottomCutBody.fiFragNum; i++)
            {
                double fi=fi0*i;
                Vector3D r01=Vector3D(part[j].x(),part[j].y()*cos(fi),part[j].y()*sin(fi));
                Vector3D r11=Vector3D(part[j].x(),part[j].y()*cos(fi+fi0),part[j].y()*sin(fi+fi0));
                Vector3D r21=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi+fi0),part[j+1].y()*sin(fi+fi0));
                Vector3D r31=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi),part[j+1].y()*sin(fi));
                Vector3D controlPoint = Vector3D(forControlPoint[j].x(),forControlPoint[j].y()*cos(fi+fi0*0.5),forControlPoint[j].y()*sin(fi+fi0*0.5));
                Vector3D normal = Vector3D(forNormals[j].x(),forNormals[j].y()*cos(fi+fi0*0.5),forNormals[j].y()*sin(fi+fi0*0.5));
                frames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,rotationBottomCutBody.vortonsRad));
                normals.push_back(normal);
                controlPoints.push_back(controlPoint);
                squares.push_back((0.5*Vector3D::crossProduct(r21-r01,r31-r11).length()));
                controlPointsRaised.push_back(controlPoint+rotationBottomCutBody.raise*normal);
            }
        }
        // qDebug()<<"ya zhiv2";
        Vector3D r0 = Vector3D(part[0].x(),0,0);
        Vector3D r11 = Vector3D(part[0].x(),part[0].y()*cos(fi0),part[0].y()*sin(fi0));
        Vector3D r21 = Vector3D(part[0].x(),part[0].y(),0.0);
        frames.push_back(std::make_shared<MultiFrame>(rotationBottomCutBody.fiFragNum,r0,r11,r21,rotationBottomCutBody.vortonsRad));
        Vector3D controlPoint = Vector3D(part[0].x(),0.0,0.0);
        controlPoints.push_back(controlPoint);
        Vector3D normal (-1,0,0);
        normals.push_back(normal);
        squares.push_back((0.5*Vector3D::crossProduct(r11-r0,r21-r0).length()*rotationBottomCutBody.fiFragNum));
        controlPointsRaised.push_back(controlPoint+rotationBottomCutBody.raise*normal);

        double rad0=part.last().y()/rotationBottomCutBody.rFragNum;

        Vector3D r00 (part[newPartFragmNum].x(),0.0,0.0);
        r11=Vector3D(part[newPartFragmNum].x(),rad0,0);
        r21=Vector3D (part[newPartFragmNum].x(),rad0*cos(fi0),rad0*sin(fi0));
        frames.push_back(std::make_shared<MultiFrame>(rotationBottomCutBody.fiFragNum,r00,r11,r21,rotationBottomCutBody.vortonsRad));
        controlPoint =Vector3D(part[newPartFragmNum].x()-rotationBottomCutBody.delta,0.0,0.0);
        controlPoints.push_back(Vector3D(controlPoint));
        normal = Vector3D(1,0,0);
        normals.push_back(normal);
        controlPointsRaised.push_back(controlPoint+normal*rotationBottomCutBody.raise);
        squares.push_back(0.5*(r11-r00).lengthSquared()*rotationBottomCutBody.fiFragNum*sin(2*M_PI/rotationBottomCutBody.fiFragNum));
        //     qDebug()<<"ya zhiv";
        for (int i=1; i<rotationBottomCutBody.rFragNum; i++)
        {
            for (int j=0; j<rotationBottomCutBody.fiFragNum; j++)
            {
                double fi = fi0*j;
                double r = rad0*i;
                Vector3D r11 (part[newPartFragmNum].x(),r*cos(fi),r*sin(fi));
                Vector3D r01 (part[newPartFragmNum].x(),r*cos(fi+fi0),r*sin(fi+fi0));
                Vector3D r31 (part[newPartFragmNum].x(),(r+rad0)*cos(fi+fi0),(r+rad0)*sin(fi+fi0));
                Vector3D r21 (part[newPartFragmNum].x(),(r+rad0)*cos(fi),(r+rad0)*sin(fi));
                frames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,rotationBottomCutBody.vortonsRad));
                controlPoint = Vector3D(part[newPartFragmNum].x()-rotationBottomCutBody.delta,(r+0.5*rad0)*cos(fi+0.5*fi0),(r+rad0*0.5)*sin(fi+fi0*0.5));
                controlPoints.push_back(controlPoint);
                normal = Vector3D (1,0,0);
                normals.push_back(normal);
                squares.push_back((Vector3D::crossProduct(r21-r01,r31-r11)).length()*0.5);
                controlPointsRaised.push_back(controlPoint+normal*rotationBottomCutBody.raise);
            }
        }
    }
}

/*!
Очищает вектора контрольных точек, нормалей и т.д.
*/
void BodyFragmentation::clearVectors()
{
    controlPoints.clear();
    controlPointsRaised.clear();
    normals.clear();
    squares.clear();
    frames.clear();
}

void BodyFragmentation::prepareGraphsX0(QVector<std::shared_ptr<MultiFrame> > &xFrames, FormingParameters pars)
{
    double center;
    switch(pars.typeNum)
    {
    case 0:
    {
        center=(pars.sectorOneLength+pars.sectorTwoLength+pars.diameter*0.5)*0.5;
        break;
    }
    case 1:
    {
        center=pars.sectorOneLength*0.5;
        break;
    }
    case 2:
    {

        center=pars.sectorOneLength*0.5;
        break;
    }
    }

    QVector<Vector2D> part(rotationBody.partFragNum);
    QVector<Vector2D> forNormals(rotationBody.partFragNum-1);
    QVector<Vector2D> forControlPoint(rotationBody.partFragNum-1);
    QVector<Vector2D> forUp(rotationBody.partFragNum);
    const int NFRAG=400;

    QVector<double> s(NFRAG);
    QVector<double> xArr(NFRAG);
    QVector<double> yArr(NFRAG);

    double newBeg=rotationBody.xBeg+rotationBody.sectionDistance;
    double newEnd=rotationBody.xEnd-rotationBody.sectionEndDistance;
    double fi0 = 2*M_PI/rotationBody.fiFragNum;
    double height=(newEnd-newBeg)/(NFRAG-1);
    s[0]=0.0;
    xArr[0]=newBeg;
    yArr[0]=BodyFragmentation::presetFunctionF(newBeg,forming);
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunctionF(xArr[i],forming);
        double derivative=BodyFragmentation::presetDeriveFunctionF(xArr[i],forming);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }


    double length=s[s.size()-1];
    for (int i=0; i<rotationBody.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotationBody.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionF(xArr[num],forming),1).normalized();
    }
    for (int i=0; i<part.size(); i++)
        part[i]+=forUp[i]*rotationBody.delta;
    for (int j=0; j<rotationBody.partFragNum-1; j++)
    {
        for (int i=0; i<rotationBody.fiFragNum; i++)
        {
            double fi=fi0*i;
            Vector3D r01=Vector3D(part[j].x(),0.0,part[j].y()*sin(fi));
            Vector3D r11=Vector3D(part[j].x(),0.0,part[j].y()*sin(fi+fi0));
            Vector3D r21=Vector3D(part[j+1].x(),0.0,part[j+1].y()*sin(fi+fi0));
            Vector3D r31=Vector3D(part[j+1].x(),0.0,part[j+1].y()*sin(fi));
            xFrames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,0.0));
        }
    }
}

void BodyFragmentation::prepareGraphsY0(QVector<std::shared_ptr<MultiFrame> > &yFrames, FormingParameters pars)
{
    double center;
    switch(pars.typeNum)
    {
    case 0:
    {
        center=(pars.sectorOneLength+pars.sectorTwoLength+pars.diameter*0.5)*0.5;
        break;
    }
    case 1:
    {
        center=pars.sectorOneLength*0.5;
        break;
    }
    case 2:
    {

        center=pars.sectorOneLength*0.5;
        break;
    }
    }

    QVector<Vector2D> part(rotationBody.partFragNum);
    QVector<Vector2D> forNormals(rotationBody.partFragNum-1);
    QVector<Vector2D> forControlPoint(rotationBody.partFragNum-1);
    QVector<Vector2D> forUp(rotationBody.partFragNum);
    const int NFRAG=400;

    QVector<double> s(NFRAG);
    QVector<double> xArr(NFRAG);
    QVector<double> yArr(NFRAG);

    double newBeg=rotationBody.xBeg+rotationBody.sectionDistance;
    double newEnd=rotationBody.xEnd-rotationBody.sectionEndDistance;
    double fi0 = 2*M_PI/rotationBody.fiFragNum;
    double height=(newEnd-newBeg)/(NFRAG-1);
    s[0]=0.0;
    xArr[0]=newBeg;
    yArr[0]=BodyFragmentation::presetFunctionF(newBeg,forming)+rotationBody.delta;
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunctionF(xArr[i],forming);
        double derivative=BodyFragmentation::presetDeriveFunctionF(xArr[i],forming);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }


    double length=s[s.size()-1];
    for (int i=0; i<rotationBody.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotationBody.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionF(xArr[num],forming),1).normalized();
    }
    for (int i=0; i<part.size(); i++)
        part[i]+=forUp[i]*rotationBody.delta;
    for (int j=0; j<rotationBody.partFragNum-1; j++)
    {
        for (int i=0; i<rotationBody.fiFragNum; i++)
        {
            double fi=fi0*i;
            Vector3D r01=Vector3D(part[j].x(),part[j].y()*cos(fi),0.0);
            Vector3D r11=Vector3D(part[j].x(),part[j].y()*cos(fi+fi0),0.0);
            Vector3D r21=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi+fi0),0.0);
            Vector3D r31=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi),0.0);
            yFrames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,0.0));
        }
    }
}

void BodyFragmentation::prepareGraphsZ0(QVector<std::shared_ptr<MultiFrame> > &zFrames, FormingParameters pars)
{
    double center;
    switch(pars.typeNum)
    {
    case 0:
    {
        center=(pars.sectorOneLength+pars.sectorTwoLength+pars.diameter*0.5)*0.5;
        break;
    }
    case 1:
    {
        center=pars.sectorOneLength*0.5;
        break;
    }
    case 2:
    {

        center=pars.sectorOneLength*0.5;
        break;
    }
    }
    double rad0=(BodyFragmentation::presetFunctionF(center,pars)+rotationBody.delta)/4;
    qDebug()<<rad0;
    double fi0 = 2*M_PI/rotationBody.fiFragNum;
    Vector3D r0=Vector3D(center,0.0,0.0);
    Vector3D r1 = Vector3D(center, rad0, 0.0);
    Vector3D r2 = Vector3D(center, rad0*cos(fi0),rad0*sin(fi0));
    zFrames.push_back(std::make_shared<MultiFrame>(rotationBody.fiFragNum,r0,r1,r2,0));

    for (int i=1; i<4; i++)
    {
        for (int j=0; j<rotationBody.fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = rad0*i;
            Vector3D r11 (center,r*cos(fi),r*sin(fi));
            Vector3D r01 (center,r*cos(fi+fi0),r*sin(fi+fi0));
            Vector3D r31 (center,(r+rad0)*cos(fi+fi0),(r+rad0)*sin(fi+fi0));
            Vector3D r21 (center,(r+rad0)*cos(fi),(r+rad0)*sin(fi));
            zFrames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,0));
        }
    }
}

FormingParameters BodyFragmentation::getForming()
{
    return forming;
}

FormingParametersRBC BodyFragmentation::getFormingRBC()
{
    return formingRBC;
}

/*!
Высчитывает значение функции f(x)
\param x Значение х
\return Возвращаемое значение функции
*/
double BodyFragmentation::presetFunctionF(double x, FormingParameters parameters)
{
    //   if ((x>=0)&&(x<=2)) return sqrt(1-(x-1)*(x-1));
    switch (parameters.typeNum)
    {
    case 0:
    {
        if (x>=0.0&&x<=parameters.diameter*0.5)
            return sqrt(pow(parameters.diameter*0.5,2)-pow(x-parameters.diameter*0.5,2));
        else
        {
            if (x>parameters.diameter*0.5&&x<=parameters.sectorOneLength+parameters.diameter*0.5)
                return parameters.diameter*0.5;
            else
            {
                if (x>(parameters.sectorOneLength+parameters.diameter*0.5)&& x<=(parameters.sectorTwoLength+parameters.diameter*0.5+parameters.sectorOneLength))
                    return parameters.diameter*0.5*(1.0-(x-parameters.sectorOneLength-parameters.diameter*0.5)/parameters.sectorTwoLength);
                else
                    return 0;
            }
        }
    }
    case 1:
    {
        double height=parameters.diameter*0.5/tan(parameters.angle);
        if (x>=0.0 && x<height)
            return x*tan(parameters.angle);
        else
        {
            if(x>=height && x<parameters.sectorOneLength-height)
                return parameters.diameter*0.5;
            else
            {
                if (x>=parameters.sectorOneLength-height && x<parameters.sectorOneLength)
                    return parameters.diameter*0.5-(x-parameters.sectorOneLength+height)*tan(parameters.angle);
                else
                    break;
            }

        }
    }
    case 2:
    {
        if (x<parameters.sectorOneLength&&x>=0)
            return parameters.diameter*0.5*sqrt(1-(4.0*pow(x-parameters.sectorOneLength*0.5,2))/pow(parameters.sectorOneLength,2));
        else
            break;
    }
    }
    return 0.0;

}

/*!
Высчитывает значение производной функции f(x)
\param x Значение х
\return Возвращаемое значение производной функции
*/
double BodyFragmentation::presetDeriveFunctionF(double x, FormingParameters parameters)
{
    //    if ((x>=0)&&(x<=2)) return -(x-1)/presetFunctionF(x,parameters);
    switch (parameters.typeNum)
    {
    case 0:
    {
        if (x>=0.0&&x<=parameters.diameter*0.5)
            return -(x-parameters.diameter*0.5)/BodyFragmentation::presetFunctionF(x,parameters);
        else
        {
            if (x>parameters.diameter*0.5&&x<=parameters.sectorOneLength+parameters.diameter*0.5)
                return 0.0;
            else
            {
                if (x>(parameters.sectorOneLength+parameters.diameter*0.5)&& x<=(parameters.sectorTwoLength+parameters.diameter*0.5+parameters.sectorOneLength))
                    return -parameters.diameter*0.5/parameters.sectorTwoLength;
                else
                    return 0;
            }
        }
    }
    case 1:
    {
        double height=parameters.diameter*0.5/tan(parameters.angle);
        if (x>=0.0 && x<height)
            return tan(parameters.angle);
        else
        {
            if(x>=height && x<parameters.sectorOneLength-height)
                return 0.0;
            else
            {
                if (x>=parameters.sectorOneLength-height && x<parameters.sectorOneLength)
                    return -tan(parameters.angle);
                else
                    break;
            }

        }
    }
    case 2:
    {
        if (x<parameters.sectorOneLength&&x>=0)
            return -(x-parameters.sectorOneLength*0.5)/BodyFragmentation::presetFunctionF(x,parameters)*pow(parameters.diameter/parameters.sectorOneLength,2);
        else
            break;
    }
    }
    return 0.0;


}

/*!
Высчитывает значение функции g(x)
\param x Значение х
\return Возвращаемое значение функции
*/
double BodyFragmentation::presetFunctionG(double x, FormingParametersRBC parameters)
{
    switch (parameters.type)
    {
    case ELLIPSOID_CONE:
    {
        if (x>=0.0&&x<parameters.ellipsoidLength)
            return parameters.ellipsoidDiameter*0.5*sqrt(1-pow(x-parameters.ellipsoidLength,2)/pow(parameters.ellipsoidLength,2));
        else
        {
            if (x>=parameters.ellipsoidLength&&x<=parameters.coneLength+parameters.ellipsoidLength)
                return parameters.ellipsoidDiameter*0.5-(parameters.ellipsoidDiameter-parameters.tailDiameter)*0.5/parameters.coneLength*(x-parameters.ellipsoidLength);
            else
            {
                return 0.0;
            }
        }
    }
    case ELLIPSOID_CYLINDER:
    {
        if (x>=0.0 && x<parameters.ellipsoidLength)
            return parameters.ellipsoidDiameter*0.5*sqrt(1-pow(x-parameters.ellipsoidLength,2)/pow(parameters.ellipsoidLength,2));
        else {
            if (x>=parameters.ellipsoidLength && x<=parameters.fullLength)
                return parameters.ellipsoidDiameter*0.5;
            else
                return 0.0;
        }
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        if (x >=0.0 && x<=parameters.ellipsoidLength)
            return  parameters.ellipsoidDiameter*0.5*sqrt(1-pow(x-parameters.ellipsoidLength,2)/pow(parameters.ellipsoidLength,2));
        else
        {
            if (x>parameters.ellipsoidLength && x<parameters.fullLength-parameters.coneLength)
                return parameters.ellipsoidDiameter*0.5;
            else {
                if (x>=parameters.fullLength-parameters.coneLength && x<=parameters.fullLength+1)
                    return parameters.ellipsoidDiameter*0.5 - (parameters.ellipsoidDiameter-parameters.tailDiameter)*0.5/parameters.coneLength*(x-(parameters.fullLength-parameters.coneLength));
                else {
                    return 0.0;
                }
            }
        }
    }
    }
}

/*!
Высчитывает значение функции g(x)
\param x Значение х
\param xBeg Нижняя граница функции
\return Возвращаемое значение функции
*/
//double BodyFragmentation::presetFunctionG(double x, double xBeg, FormingParameters parameters)
//{
//    switch (parameters.typeNum)
//    {
//    case 0:
//    {
//        if (x>=0.0&&x<parameters.sectorOneLength)
//            return parameters.diameter*0.5*sqrt(1-pow(x-parameters.sectorOneLength,2)/pow(parameters.sectorOneLength,2));
//        else
//        {
//            if (x>=parameters.sectorOneLength&&x<=parameters.sectorTwoLength+parameters.sectorOneLength)
//                return parameters.diameter*0.5-(parameters.diameter-parameters.tailDiameter)*0.5/parameters.sectorTwoLength*(x-parameters.sectorOneLength);
//            else
//            {
//                    return 0.0;
//            }
//        }
//    }
//    default:
//        return 0.0;
//    }

////    if ((x>=xBeg)&&(x<=1.4+xBeg)) return 2*sqrt(1-(x-xBeg-1.4)*(x-xBeg-1.4)/(1.4*1.4));
////    if ((x>=1.4+xBeg)&&(x<=xBeg+5)) return 2;
////    return 0.0;
//}

/*!
Высчитывает значение производной функции g(x)
\param x Значение х
\return Возвращаемое значение производной функции
*/
double BodyFragmentation::presetDeriveFunctionG(double x, FormingParametersRBC parameters)
{
    switch (parameters.type)
    {
    case ELLIPSOID_CONE:
    {
        if (x>=0.0&&x<parameters.ellipsoidLength)
            return -parameters.ellipsoidDiameter*parameters.ellipsoidDiameter*0.25/pow(parameters.ellipsoidLength,2)*(x-parameters.ellipsoidLength)/presetFunctionG(x,parameters);

        else
        {
            if (x>=parameters.ellipsoidLength&&x<=parameters.ellipsoidLength+parameters.coneLength)
                return -(parameters.ellipsoidDiameter-parameters.tailDiameter)*0.5/parameters.coneLength;
            else
            {
                return 0.0;
            }
        }
    }
    case ELLIPSOID_CYLINDER:
    {
        if (x>=0.0 && x<parameters.ellipsoidLength)
            return -pow(parameters.ellipsoidDiameter,2)*0.25/pow(parameters.ellipsoidLength,2)*(x-parameters.ellipsoidLength)/presetFunctionG(x,parameters);
        else
            return 0.0;
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        if (x >=0.0 && x<=parameters.ellipsoidLength)
            return  -pow(parameters.ellipsoidDiameter*0.5/parameters.ellipsoidLength,2)*(x-parameters.ellipsoidLength)/BodyFragmentation::presetFunctionG(x,parameters);
        else
        {
            if (x>parameters.ellipsoidLength && x<parameters.fullLength-parameters.coneLength)
                return 0.0;
            else {
                if (x>=parameters.fullLength-parameters.coneLength && x<=parameters.fullLength)
                    return -(parameters.ellipsoidDiameter-parameters.tailDiameter)*0.5/parameters.coneLength;
                else {
                    return 0.0;
                }
            }
        }
    }
    }
    //    if ((x>=0)&&(x<=1.4)) return -2*(x-1.4)/sqrt(1.4*1.4-(x-1.4)*(x-1.4));
    //    if ((x>=1.4)&&(x<=5)) return 0.0;
    //    return 0.0;
}

/*!
Возвращает рассчитанные контрольные точки
\return Вектор контрольных точек
*/
QVector<Vector3D> BodyFragmentation::getControlPoints() const
{
    return controlPoints;
}

/*!
Возвращает рассчитанные нормали
\return Вектор нормалей
*/
QVector<Vector3D> BodyFragmentation::getNormals() const
{
    return normals;
}

/*!
Возвращает рассчитанные площади
\return Вектор площадей
*/
QVector<double> BodyFragmentation::getSquares() const
{
    return squares;
}

/*!
Возвращает рассчитанные контрольные точки для вычисления давления
\return Вектор контрольных точек для вычисления давления
*/
QVector<Vector3D> BodyFragmentation::getControlPointsRaised() const
{
    return controlPointsRaised;
}

/*!
Возвращает рассчитанные рамки
\return Вектор умных указателей на рамки
*/
QVector<std::shared_ptr<MultiFrame> > BodyFragmentation::getFrames() const
{
    return frames;
}

/*!
Поиск ближайшей рамки
\param point Точка, для которой ищется ближайшая рамка
\param controlPoints Вектор контрольных точек
\param normals Вектор нормалей
\return Пара из наименьшего расстояния до рамки и номера этой рамки
*/
QPair<double, int> BodyFragmentation::findClosest(const Vector3D point, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    QPair<double, int> closest = qMakePair(fabs(Vector3D::dotProduct(point-controlPoints[0],normals[0])),0);
    for (int i=1; i<controlPoints.size(); i++)
    {
        double newClosest=fabs(Vector3D::dotProduct(point-controlPoints[i],normals[i]));
        if (newClosest<closest.first)
        {
            closest=qMakePair(newClosest,i);
        }
    }
    return closest;
}

QPair<double, int> BodyFragmentation::findClosestTriangle(const Vector3D point, const QVector<std::shared_ptr<MultiFrame>> &frames, const QVector<Vector3D> &normals)
{
    QPair<double, int> closest = qMakePair(1000000000000000.0,0);
    for (int i=0; i<frames.size(); i++)
    {
        if (frames[i]->getAnglesNum()==4)
        {
            Vector3D a1=(frames[i]->at(0).getTail()+frames[i]->at(2).getTail())*0.5;
            Vector3D a2=(frames[i]->at(1).getTail()+frames[i]->at(2).getTail())*0.5;
            Vector3D n=Vector3D::crossProduct(frames[i]->at(2).getTail()-frames[i]->at(0).getTail(),frames[i]->at(1).getTail()-frames[i]->at(0).getTail());
            Vector3D n1=Vector3D::crossProduct(n,frames[i]->at(2).getTail()-frames[i]->at(0).getTail());
            Vector3D n2=Vector3D::crossProduct(n,frames[i]->at(2).getTail()-frames[i]->at(1).getTail());
            double t1;
            if (coDirectionallyCheck(Vector3D::crossProduct(a2-a2,n),Vector3D::crossProduct(n1,n2)))
                t1=Vector3D::crossProduct(a2-a1,n2).length()/Vector3D::crossProduct(n1,n2).length();
            else {
                t1=-Vector3D::crossProduct(a2-a1,n2).length()/Vector3D::crossProduct(n1,n2).length();
            }
            Vector3D center=a1+n1*t1;

            double newClosest=(point-center).length();
            if (newClosest<closest.first)
            {
                closest.first=newClosest;
                closest.second=i;
            }


            a1=(frames[i]->at(0).getTail()+frames[i]->at(2).getTail())*0.5;
            a2=(frames[i]->at(3).getTail()+frames[i]->at(2).getTail())*0.5;
            n=Vector3D::crossProduct(frames[i]->at(2).getTail()-frames[i]->at(0).getTail(),frames[i]->at(3).getTail()-frames[i]->at(0).getTail());
            n1=Vector3D::crossProduct(n,frames[i]->at(2).getTail()-frames[i]->at(0).getTail());
            n2=Vector3D::crossProduct(n,frames[i]->at(2).getTail()-frames[i]->at(3).getTail());
            if (coDirectionallyCheck(Vector3D::crossProduct(a2-a2,n),Vector3D::crossProduct(n1,n2)))
                t1=Vector3D::crossProduct(a2-a1,n2).length()/Vector3D::crossProduct(n1,n2).length();
            else {
                t1=-Vector3D::crossProduct(a2-a1,n2).length()/Vector3D::crossProduct(n1,n2).length();
            }
            center=a1+n1*t1;

            newClosest=(point-center).length();
            if (newClosest<closest.first)
            {
                closest.first=newClosest;
                closest.second=i;
            }
        }
        else {
            for (int j=1;j<frames[i]->getAnglesNum()-1;j++)
            {
                Vector3D a1=(frames[i]->at(0).getTail()+frames[i]->at(j+1).getTail())*0.5;
                Vector3D a2=(frames[i]->at(j).getTail()+frames[i]->at(j+1).getTail())*0.5;
                Vector3D n=Vector3D::crossProduct(frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail(),frames[i]->at(j).getTail()-frames[i]->at(0).getTail());
                Vector3D n1=Vector3D::crossProduct(n,frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail());
                Vector3D n2=Vector3D::crossProduct(n,frames[i]->at(j+1).getTail()-frames[i]->at(j).getTail());
                double t1;
                if (coDirectionallyCheck(Vector3D::crossProduct(a2-a2,n),Vector3D::crossProduct(n1,n2)))
                    t1=Vector3D::crossProduct(a2-a1,n2).length()/Vector3D::crossProduct(n1,n2).length();
                else {
                    t1=-Vector3D::crossProduct(a2-a1,n2).length()/Vector3D::crossProduct(n1,n2).length();
                }
                Vector3D center=a1+n1*t1;

                double newClosest=(point-center).length();
                if (newClosest<closest.first)
                {
                    closest.first=newClosest;
                    closest.second=i;
                }
            }

        }
    }
    return closest;
}

/*!
Поиск ближайшего элемента из массива к точке
\param arr Массив, в котором ищется ближайший элемент
\param point Точка, для которой ищется ближайшая рамка
\return Номер ближайшего элемента
*/
int BodyFragmentation::findClosetElementFromArray(const QVector<double> arr, const double point)
{
    double closest=arr[0]-point;
    int num = 0;
    for (int i=1; i<arr.size(); i++)
    {
        if ((fabs(arr[i]-point))<(fabs(closest)))
        {
            closest = arr[i]-point;
            num = i;
        }
    }
    return num;
}

bool BodyFragmentation::coDirectionallyCheck(const Vector3D a, const Vector3D b)
{
    Vector3D collinearCrossing=Vector3D::crossProduct(a,b);
    if (collinearCrossing.length()<0.0000001 && Vector3D::dotProduct(a,b)>0)
        return true;
    return false;
}
