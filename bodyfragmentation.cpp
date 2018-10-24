#include "bodyfragmentation.h"

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
        rotationBody = RotationBodyParameters {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,  param.delta, param.pointsRaising, param.vortonsRad};
        rotationBodyFragmantation();
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        if (!launch)
        {
            rotationBottomCutBody = RotationCutBodyParameters {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum,param.rotationBodyRFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,  param.delta, param.pointsRaising, param.vortonsRad};
            rotationCutBodyFragmantation();
        }
        else
        {
            rotationBottomCutBody = RotationCutBodyParameters {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum,param.rotationBodyRFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,  param.delta, param.pointsRaising, param.vortonsRad};
        }
        break;
    }
    default:
    {
        QMessageBox::critical(new QWidget(), tr("Ошибка"), tr("Попытка разбить несоответствующее тело"));
        exit(1);
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

/*!
Реализует разбиение тела вращения.
*/
void BodyFragmentation::rotationBodyFragmantation()
{
    clearVectors();
    QVector<Vector2D> part(rotationBody.partFragNum);
    QVector<Vector2D> forNormals(rotationBody.partFragNum);
    QVector<Vector2D> forControlPoint(rotationBody.partFragNum);
    QVector<Vector2D> forUp(rotationBody.partFragNum);
    const int NFRAG=400;

    QVector<double> s(NFRAG);
    QVector<double> xArr(NFRAG);
    QVector<double> yArr(NFRAG);

    double newBeg=rotationBody.xBeg+rotationBody.sectionDistance;
    double newEnd=rotationBody.xEnd-0.1;
    double fi0 = 2*M_PI/rotationBody.fiFragNum;
    double height=(newEnd-newBeg)/(NFRAG-1);
    s[0]=0.0;
    xArr[0]=newBeg;
    yArr[0]=BodyFragmentation::presetFunctionF(newBeg);
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunctionF(xArr[i]);
        double derivative=BodyFragmentation::presetDeriveFunctionF(xArr[i]);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }
    double length=s[s.size()-1];
    for (int i=0; i<rotationBody.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotationBody.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionF(xArr[num]),1).normalized();
    }

    for (int i=0; i<rotationBody.partFragNum-1; i++)
    {
        int translNum=findClosetElementFromArray(s,length/(rotationBody.partFragNum-1)*(i+0.5));
        forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
        forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionF(xArr[translNum]),1).normalized();
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
    yArr[0]=BodyFragmentation::presetFunctionG(newBeg);
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunctionG(xArr[i]);
        double derivative=BodyFragmentation::presetDeriveFunctionG(xArr[i]);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }
    double length=s[s.size()-2];
    for (int i=0; i<rotationBottomCutBody.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[num]),1).normalized();
    }

    for (int i=0; i<rotationBottomCutBody.partFragNum-1; i++)
    {
        int translNum=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*(i+0.5));
        forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
        forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[translNum]),1).normalized();
    }

    for (int i=0; i<part.size(); i++)
        part[i]+=forUp[i]*rotationBottomCutBody.delta;

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
void BodyFragmentation::rotationCutBodyLaunchFragmentation(const int i, const Vector3D& bodyVel, const double tau)
{

    double ledge=bodyVel.length()*tau*(i+1);

    if (ledge<rotationBottomCutBody.xEnd-rotationBottomCutBody.xBeg)
    {
        clearVectors();
        QVector<Vector2D> part(rotationBottomCutBody.partFragNum);
        QVector<Vector2D> forNormals(rotationBottomCutBody.partFragNum-1);
        QVector<Vector2D> forControlPoint(rotationBottomCutBody.partFragNum-1);
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
        yArr[0]=BodyFragmentation::presetFunctionG(newBeg);
        for (int i=1; i<NFRAG; i++)
        {
            xArr[i]=newBeg+i*height;
            yArr[i]=BodyFragmentation::presetFunctionG(xArr[i]);
            double derivative=BodyFragmentation::presetDeriveFunctionG(xArr[i]);
            s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
        }
        double length=s[s.size()-2];
        for (int i=0; i<rotationBottomCutBody.partFragNum; i++)
        {
            int num=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*i);
            part[i]=Vector2D(xArr[num],yArr[num]);
            forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[num]),1).normalized();
        }

        for (int i=0; i<rotationBottomCutBody.partFragNum-1; i++)
        {
            int translNum=findClosetElementFromArray(s,length/(rotationBottomCutBody.partFragNum-1)*(i+0.5));
            forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
            forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunctionG(xArr[translNum]),1).normalized();
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
        part.removeLast();
        forUp.removeLast();
        forControlPoint.removeLast();
        forNormals.removeLast();
        end--;

        part.push_back(Vector2D(ledge, BodyFragmentation::presetFunctionG(ledge)));
        forUp.push_back(Vector2D(-BodyFragmentation::presetDeriveFunctionG(ledge),1.0).normalized());
        forControlPoint.push_back(Vector2D(0.5*(part[end]+part[end+1])));
        forNormals.push_back(Vector2D(-BodyFragmentation::presetDeriveFunctionG(forControlPoint.last().x()),1).normalized());

        for (int i=0; i<part.size(); i++)
            part[i]+=forUp[i]*rotationBottomCutBody.delta;

        int newPartFragmNum = part.size()-1;

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

/*!
Высчитывает значение функции f(x)
\param x Значение х
\return Возвращаемое значение функции
*/
double BodyFragmentation::presetFunctionF(double x)
{
    if ((x>=0)&&(x<=0.5)) return sqrt(0.5*0.5-(x-0.5)*(x-0.5));
    if ((x>=0.5)&&(x<=2)) return 0.5;
    if ((x>=2)&&(x<=2.5)) return 2.5-x;
    return 0.0;
}

/*!
Высчитывает значение производной функции f(x)
\param x Значение х
\return Возвращаемое значение производной функции
*/
double BodyFragmentation::presetDeriveFunctionF(double x)
{
    if ((x>=0) && (x<=0.5)) return -(x-0.5)/sqrt(0.5*0.5-(x-0.5)*(x-0.5));
    if ((x>=0.5)&&(x<=2)) return 0.0;
    if ((x>=2)&&(x<=2.5)) return -1.0;
    return 0.0;
}

/*!
Высчитывает значение функции g(x)
\param x Значение х
\return Возвращаемое значение функции
*/
double BodyFragmentation::presetFunctionG(double x)
{
    if ((x>=0)&&(x<=1.4)) return 2*sqrt(1-(x-1.4)*(x-1.4)/(1.4*1.4));
    if ((x>=1.4)&&(x<=5)) return 2;
    return 0.0;
}

/*!
Высчитывает значение производной функции g(x)
\param x Значение х
\return Возвращаемое значение производной функции
*/
double BodyFragmentation::presetDeriveFunctionG(double x)
{
    if ((x>=0)&&(x<=1.4)) return -2*(x-1.4)/sqrt(1.4*1.4-(x-1.4)*(x-1.4));
    if ((x>=1.4)&&(x<=5)) return 0.0;
    return 0.0;
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
        if ((fabs(arr[i]-point))<(fabs(closest)))
        {
            closest = arr[i]-point;
            num = i;
        }
    return num;
}
