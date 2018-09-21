#include "bodyfragmentation.h"

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

BodyFragmentation::BodyFragmentation(BodyType body,const FragmentationParameters &param)
{
    switch(body)
    {
    case SPHERE:
    {
        SphereParameters sphPar {param.sphereFiFragNum, param.sphereTetaFragNum, param.sphereRad, param.delta, param.pointsRaising,param.vortonsRad};
        sphereFragmentation(sphPar);
        break;
    }
    case CYLINDER:
    {
        CylinderParameters cylPar {param.cylinderFiFragNum, param.cylinderRadFragNum, param.cylinderHeightFragNum,param.cylinderDiameter, param.cylinderHeight,  param.delta, param.pointsRaising,param.vortonsRad};
        cylinderFragmentation(cylPar);
        break;
    }
    case ROTATIONBODY:
    {
        RotationBodyParameters rotBodyPar {param.rotationBodyFiFragNum, param.rotationBodyPartFragNum, param.rotationBodyXBeg, param.rotationBodyXEnd,param.rotationBodySectionDistance,  param.delta, param.pointsRaising, param.vortonsRad};
        rotationBodyFragmantation(rotBodyPar);
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        //rotationBottomCutFragmentation();
        break;
    }
    default:
    {
        QMessageBox::critical(new QWidget(), tr("Ошибка"), tr("Попытка разбить несоответствующее тело"));
        exit(1);
    }
    }
}

void BodyFragmentation::sphereFragmentation(const SphereParameters& param)
{
    clearVectors();
    double fi0=2*M_PI/param.fiFragNum;
    double tetaMulti=M_PI/(param.tetaFragNum+2);
    double teta0=(M_PI-2*tetaMulti)/param.tetaFragNum;
    double R=param.radius+param.delta;
    for (int i=0; i<param.fiFragNum; i++)
    {
        for (int j=0; j<param.tetaFragNum; j++)
        {
            double fi=fi0*i;
            double teta=tetaMulti+teta0*j;
            Vector3D vec0(R*sin(teta)*cos(fi),R*sin(teta)*sin(fi),R*cos(teta));
            Vector3D vec1(R*sin(teta)*cos(fi+fi0),R*sin(teta)*sin(fi+fi0),R*cos(teta));
            Vector3D vec2(R*sin(teta+teta0)*cos(fi+fi0),R*sin(teta+teta0)*sin(fi+fi0),R*cos(teta+teta0));
            Vector3D vec3(R*sin(teta+teta0)*cos(fi),R*sin(teta+teta0)*sin(fi),R*cos(teta+teta0));
            frames.push_back(std::make_shared<FourFrame>(vec0,vec1,vec2,vec3,param.vortonsRad));
            controlPoints.push_back(Vector3D(param.radius*sin(teta+0.5*teta0)*cos(fi+0.5*fi0),param.radius*sin(teta+0.5*teta0)*sin(fi+0.5*fi0),param.radius*cos(teta+0.5*teta0)));
            normals.push_back(controlPoints.last()/param.radius);
            squares.push_back(0.5*Vector3D::crossProduct(vec2-vec0,vec3-vec1).length());
            controlPointsRaised.push_back(Vector3D((param.radius+param.raise)*sin(teta+0.5*teta0)*cos(fi+0.5*fi0),(param.radius+param.raise)*sin(teta+0.5*teta0)*sin(fi+0.5*fi0),(param.radius+param.raise)*cos(teta+0.5*teta0)));
        }
    }
    double teta=M_PI-tetaMulti;
    Vector3D vec0(0.0,0.0,R*cos(teta));
    Vector3D vec1(R*sin(teta)*cos(fi0),R*sin(teta)*sin(fi0),R*cos(teta));
    Vector3D vec2(R*sin(teta)*cos(2*fi0),R*sin(teta)*sin(2*fi0),R*cos(teta));
    frames.push_back(std::make_shared<MultiFrame>(param.fiFragNum, vec0,vec1,vec2,param.vortonsRad));
    controlPoints.push_back(Vector3D(0.0,0.0,param.radius*cos(teta)));
    normals.push_back(Vector3D(0.0,0.0,-1.0));
    squares.push_back(0.5*Vector3D::crossProduct(vec1-vec0,vec2-vec0).length()*param.fiFragNum);
    controlPointsRaised.push_back(Vector3D(0.0,0.0,(param.radius+param.raise)*cos(teta)));

    teta0 = tetaMulti;
    vec0 = Vector3D(0,0,R*cos(teta0));
    vec1 = Vector3D(R*sin(teta0)*cos(fi0),R*sin(teta0)*sin(fi0),R*cos(teta0));
    vec2 = Vector3D(R*sin(teta0)*cos(2*fi0),R*sin(teta0)*sin(2*fi0),R*cos(teta0));
    frames.push_back(std::make_shared<MultiFrame>(param.fiFragNum,vec0,vec2,vec1,param.vortonsRad));
    controlPoints.push_back(Vector3D(0,0,param.radius*cos(teta0)));
    normals.push_back (Vector3D(0.0,0.0,1.0));
    squares.push_back((0.5*Vector3D::crossProduct(vec1-vec0,vec2-vec0).length()*param.fiFragNum));
    controlPointsRaised.push_back(Vector3D(0.0,0.0,(param.radius+param.raise)*cos(teta0)));
}

void BodyFragmentation::cylinderFragmentation(const CylinderParameters &param)
{
    clearVectors();
    double R=0.5*param.diameter+param.delta;
    double fi0=2*M_PI/param.fiFragNum;
    double r0=(param.diameter+2.0*param.delta)/(2.0*param.radFragNum);
    double h0=(param.height+2.0*param.delta)/param.heightFragNum;
    Vector3D vec0 (0.0,-param.delta,0.0);
    Vector3D vec1 (r0,-param.delta,0.0);
    Vector3D vec2 (r0*cos(fi0),-param.delta,r0*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(param.fiFragNum,vec0,vec2,vec1,param.vortonsRad));
    controlPoints.push_back(Vector3D());
    normals.push_back(Vector3D(0.0,-1.0,0.0));
    squares.append(0.5*(vec1-vec0).lengthSquared()*param.fiFragNum*sin(2*M_PI/param.fiFragNum));
    controlPointsRaised.push_back(controlPoints.last()+normals.last()*param.raise);

    for (int i=1; i<param.radFragNum; i++)
    {
        for (int j=0; j<param.fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = r0*i;
            Vector3D vec01 (r*cos(fi),-param.delta,r*sin(fi));
            Vector3D vec11 (r*cos(fi+fi0),-param.delta,r*sin(fi+fi0));
            Vector3D vec21 ((r+r0)*cos(fi+fi0),-param.delta,(r+r0)*sin(fi+fi0));
            Vector3D vec31 ((r+r0)*cos(fi),-param.delta,(r+r0)*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(vec01,vec11,vec21,vec31,param.vortonsRad));
            controlPoints.push_back(Vector3D((r+r0*0.5)*cos(fi+fi0*0.5),0.0,(r+r0*0.5)*sin(fi+fi0*0.5)));
            normals.push_back(Vector3D (0.0,-1.0,0.0));
            squares.push_back(Vector3D::crossProduct(vec21-vec01,vec31-vec11).length()*0.5);
            controlPointsRaised.push_back(controlPoints.last()+normals.last()*param.raise);
        }
    }

    vec0=Vector3D(0.0,param.height+param.delta,0.0);
    vec1=Vector3D(r0,param.height+param.delta,0.0);
    vec2=Vector3D(r0*cos(fi0),param.height+param.delta,r0*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(param.fiFragNum,vec0,vec1,vec2,param.vortonsRad));
    controlPoints.push_back(Vector3D(0.0,param.height,0.0));
    normals.push_back(Vector3D(0.0,1.0,0.0));
    squares.push_back(0.5*(vec1-vec0).lengthSquared()*param.fiFragNum*sin(2*M_PI/param.fiFragNum));
    controlPointsRaised.push_back(controlPoints.last()+normals.last()*param.raise);

    for (int i=1; i<param.radFragNum; i++)
    {
        for (int j=0; j<param.fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = r0*i;
            Vector3D vec01(r*cos(fi),param.height+param.delta,r*sin(fi));
            Vector3D vec11 (r*cos(fi+fi0),param.height+param.delta,r*sin(fi+fi0));
            Vector3D vec21((r+r0)*cos(fi+fi0),param.height+param.delta,(r+r0)*sin(fi+fi0));
            Vector3D vec31((r+r0)*cos(fi),param.height+param.delta,(r+r0)*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(vec01,vec31,vec21,vec11,param.vortonsRad));
            controlPoints.push_back(Vector3D((r+r0*0.5)*cos(fi+fi0*0.5),param.height,(r+r0*0.5)*sin(fi+fi0*0.5)));
            normals.push_back(Vector3D (0.0,1.0,0.0));
            squares.push_back(Vector3D::crossProduct(vec21-vec01,vec31-vec11).length()*0.5);
            controlPointsRaised.push_back(controlPoints.last()+normals.last()*param.raise);
        }
    }

    for (int i=0; i<param.fiFragNum; i++)
    {
        for (int j=0; j<param.heightFragNum; j++)
        {
            double s = h0*j-param.delta;
            double fi=fi0*i;
            Vector3D vec0(R*cos(fi),s,R*sin(fi));
            Vector3D vec1(R*cos(fi+fi0),s,R*sin(fi+fi0));
            Vector3D vec2(R*cos(fi+fi0),s+h0,R*sin(fi+fi0));
            Vector3D vec3(R*cos(fi),s+h0,R*sin(fi));
            frames.push_back(std::make_shared <FourFrame>(vec0,vec1,vec2,vec3,param.vortonsRad));
            controlPoints.push_back(Vector3D(0.5*param.diameter*cos(fi+fi0*0.5),s+h0*0.5,0.5*param.diameter*sin(fi+fi0*0.5)));
            normals.push_back(Vector3D(cos(fi+fi0*0.5),0.0,sin(fi+fi0*0.5)));
            squares.push_back((Vector3D::crossProduct(vec2-vec0,vec3-vec1)).length()*0.5);
            controlPointsRaised.push_back(controlPoints.last()+normals.last()*param.raise);

        }
    }
}

void BodyFragmentation::rotationBodyFragmantation(const RotationBodyParameters& rotBodyPar)
{
    clearVectors();
    QVector<Vector2D> part(rotBodyPar.partFragNum);
    QVector<Vector2D> forNormals(rotBodyPar.partFragNum);
    QVector<Vector2D> forControlPoint(rotBodyPar.partFragNum);
    QVector<Vector2D> forUp(rotBodyPar.partFragNum);
    const int NFRAG=400;

    QVector<double> s(NFRAG);
    QVector<double> xArr(NFRAG);
    QVector<double> yArr(NFRAG);

    double newBeg=rotBodyPar.xBeg+rotBodyPar.sectionDistance;
    double newEnd=rotBodyPar.xEnd-0.1;
    double fi0 = 2*M_PI/rotBodyPar.fiFragNum;
    double height=(newEnd-newBeg)/(NFRAG-1);
    s[0]=0.0;
    xArr[0]=newBeg;
    yArr[0]=BodyFragmentation::presetFunction(newBeg);
    for (int i=1; i<NFRAG; i++)
    {
        xArr[i]=newBeg+i*height;
        yArr[i]=BodyFragmentation::presetFunction(xArr[i]);
        double derivative=BodyFragmentation::presetDeriveFunction(xArr[i]);
        s[i]=s[i-1]+height*sqrt(1+derivative*derivative);
    }
    double length=s[s.size()-1];
    for (int i=0; i<rotBodyPar.partFragNum; i++)
    {
        int num=findClosetElementFromArray(s,length/(rotBodyPar.partFragNum-1)*i);
        part[i]=Vector2D(xArr[num],yArr[num]);
        forUp[i]=Vector2D(-BodyFragmentation::presetDeriveFunction(xArr[num]),1).normalized();
    }

    for (int i=0; i<rotBodyPar.partFragNum-1; i++)
    {
        int translNum=findClosetElementFromArray(s,length/(rotBodyPar.partFragNum-1)*(i+0.5));
        forControlPoint[i]=Vector2D(xArr[translNum],yArr[translNum]);
        forNormals[i]=Vector2D(-BodyFragmentation::presetDeriveFunction(xArr[translNum]),1).normalized();
    }

    for (int i=0; i<part.size(); i++)
        part[i]+=forUp[i]*rotBodyPar.delta;

    for (int j=0; j<rotBodyPar.partFragNum-1; j++)
    {
        for (int i=0; i<rotBodyPar.fiFragNum; i++)
        {
            double fi=fi0*i;
            Vector3D r01=Vector3D(part[j].x(),part[j].y()*cos(fi),part[j].y()*sin(fi));
            Vector3D r11=Vector3D(part[j].x(),part[j].y()*cos(fi+fi0),part[j].y()*sin(fi+fi0));
            Vector3D r21=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi+fi0),part[j+1].y()*sin(fi+fi0));
            Vector3D r31=Vector3D(part[j+1].x(),part[j+1].y()*cos(fi),part[j+1].y()*sin(fi));
            Vector3D controlPoint = Vector3D(forControlPoint[j].x(),forControlPoint[j].y()*cos(fi+fi0*0.5),forControlPoint[j].y()*sin(fi+fi0*0.5));
            Vector3D normal = Vector3D(forNormals[j].x(),forNormals[j].y()*cos(fi+fi0*0.5),forNormals[j].y()*sin(fi+fi0*0.5));
            frames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,rotBodyPar.vortonsRad));
            normals.push_back(normal);
            controlPoints.push_back(controlPoint);
            squares.push_back((0.5*Vector3D::crossProduct(r21-r01,r31-r11).length()));
            controlPointsRaised.push_back(controlPoint+rotBodyPar.raise*normal);
        }
    }

    Vector3D r0 = Vector3D(part[0].x(),0,0);
    Vector3D r11 = Vector3D(part[0].x(),part[0].y()*cos(fi0),part[0].y()*sin(fi0));
    Vector3D r21 = Vector3D(part[0].x(),part[0].y(),0);
    frames.push_back(std::make_shared<MultiFrame>(rotBodyPar.fiFragNum,r0,r11,r21,rotBodyPar.vortonsRad));
    Vector3D controlPoint = Vector3D(part[0].x(),0,0);
    controlPoints.push_back(controlPoint);
    Vector3D normal (-1,0,0);
    normals.push_back(normal);
    squares.push_back((0.5*Vector3D::crossProduct(r11-r0,r21-r0).length()*rotBodyPar.fiFragNum));
    controlPointsRaised.push_back(controlPoint+rotBodyPar.raise*normal);

    r0 = Vector3D(part[rotBodyPar.partFragNum-1].x(),0,0);
    r11 = Vector3D(part[rotBodyPar.partFragNum-1].x(),part[rotBodyPar.partFragNum-1].y(),0);
    r21 = Vector3D(part[rotBodyPar.partFragNum-1].x(),part[rotBodyPar.partFragNum-1].y()*cos(fi0),part[rotBodyPar.partFragNum-1].y()*sin(fi0));
    frames.push_back(std::make_shared<MultiFrame>(rotBodyPar.fiFragNum,r0,r11,r21,rotBodyPar.vortonsRad));
    controlPoint = Vector3D(part[rotBodyPar.partFragNum-1].x(),0,0);
    controlPoints.push_back(controlPoint);
    normal =Vector3D (1,0,0);
    normals.push_back(normal);
    squares.push_back((0.5*Vector3D::crossProduct(r11-r0,r21-r0).length()*rotBodyPar.fiFragNum));
    controlPointsRaised.push_back(controlPoint+rotBodyPar.raise*normal);
}

void BodyFragmentation::clearVectors()
{
    controlPoints.clear();
    controlPointsRaised.clear();
    normals.clear();
    squares.clear();
}

double BodyFragmentation::presetFunction(double x)
{
    if ((x>=0)&&(x<=0.5)) return sqrt(0.5*0.5-(x-0.5)*(x-0.5));
    if ((x>=0.5)&&(x<=2)) return 0.5;
    if ((x>=2)&&(x<=2.5)) return 2.5-x;
    return 0.0;
}

double BodyFragmentation::presetDeriveFunction(double x)
{
    if ((x>=0) && (x<=0.5)) return -(x-0.5)/sqrt(0.5*0.5-(x-0.5)*(x-0.5));
    if ((x>=0.5)&&(x<=2)) return 0;
    if ((x>=2)&&(x<=2.5)) return -1;
    return 0.0;
}

QVector<Vector3D> BodyFragmentation::getControlPoints() const
{
    return controlPoints;
}

QVector<Vector3D> BodyFragmentation::getNormals() const
{
    return normals;
}

QVector<double> BodyFragmentation::getSquares() const
{
    return squares;
}

QVector<Vector3D> BodyFragmentation::getControlPointsRaised() const
{
    return controlPointsRaised;
}

QVector<std::shared_ptr<MultiFrame> > BodyFragmentation::getFrames() const
{
    return frames;
}

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
