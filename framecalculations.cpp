#include "framecalculations.h"

void Counters::clear()
{
    unitedNum=vorticityEliminated=tooFarNum=rotatedNum=gotBackNum=underScreenNum=0;
}

void Restrictions::clear()
{
    moveRestr=elongationRestr=turnRestr=0;
}

void Timers::clear()
{
    getBackAndRotateTimer=forceTimer=unionTimer=farTimer=integrationTimer=removeVorticityTimer=0.0;
}

FrameCalculations::FrameCalculations()
{

    counters.clear();
    restrictions.clear();
    timers.clear();
}

QVector<double> FrameCalculations::calcTetas(const int tetaFragNum)
{
    QVector<double> tetas;
    tetas.push_back(0.0);
    for (int i=1; i<tetaFragNum+2;i++)
        tetas.push_back(M_PI/(tetaFragNum+2)*(0.5+i));
    tetas.push_back(M_PI);
    return tetas;
}

void FrameCalculations::matrixCalc(QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    matrixSize=frames.size()+1;
    matrix.resize(matrixSize,matrixSize);
    for (int i=0; i<matrixSize-1; i++)
    {
        for (int j=0; j<matrixSize-1; j++)
            matrix(i,j)=Vector3D::dotProduct(frames[j]->qHelp(controlPoints[i]), normals[i]);
    }
    for (int i=0; i<matrixSize-1; i++)
        matrix(matrixSize-1,i)=matrix(i,matrixSize-1)=1.0; 
    matrix(matrixSize-1,matrixSize-1)=0.0;
    matrix=matrix.inverse();

}

Eigen::VectorXd FrameCalculations::columnCalc(const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<Vector3D> &normals, const QVector<Vector3D> controlPoints)
{
    Eigen::VectorXd column(matrixSize);
    for (int i=0; i<controlPoints.size(); i++)
        column(i)=Vector3D::dotProduct(-FrameCalculations::velocity(controlPoints[i], streamVel, vortons), normals[i]);
    column(controlPoints.size())=0.0;
    return column;
}

Eigen::VectorXd FrameCalculations::vorticitiesCalc(const Eigen::VectorXd &column)
{
    return Eigen::VectorXd(matrix*column);
}

void FrameCalculations::unionVortons(QVector<Vorton> &vortons,const double eStar,const double eDoubleStar,const double vortonRad)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        for (int j=i-1; j>=0; j--)
        {
            if((vortons[i].getMid()-vortons[j].getMid()).length()<eStar)
            {
                double psi=Vector3D::dotProduct((vortons[i].getTail()-vortons[i].getMid()).normalized(),(vortons[j].getTail()-vortons[j].getMid()).normalized());
                if (fabs(psi)>eDoubleStar)
                {
                    if (psi<0)
                        vortons[j].turn();
                    counters.unitedNum++;
                    Vector3D newMid = (fabs(vortons[i].getVorticity())*vortons[i].getMid()+fabs(vortons[j].getVorticity())*vortons[j].getMid())/
                    (fabs(vortons[i].getVorticity())+fabs(vortons[j].getVorticity()));
                    Vector3D selfLen = (fabs(vortons[i].getVorticity())*(vortons[i].getTail()-vortons[i].getMid())+fabs(vortons[j].getVorticity())*(vortons[j].getTail()-vortons[j].getMid()))/
                    (fabs(vortons[i].getVorticity())+fabs(vortons[j].getVorticity()));
                    double newVorticity=vortons[i].getVorticity()+vortons[j].getVorticity();
                    Vorton newVorton = Vorton (newMid, newMid+selfLen, newVorticity, vortonRad);
                    vortons[i]=newVorton;
                    i--;
                    vortons.remove(j);
                }
            }
        }
    }
    timers.unionTimer=start.elapsed()*0.01;
}

void FrameCalculations::removeSmallVorticity(QVector<Vorton> &vortons,const double minVorticity)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (fabs(vortons[i].getVorticity())<minVorticity)
        {
            vortons.remove(i);
            counters.vorticityEliminated++;
        }
    }
    timers.removeVorticityTimer=start.elapsed()*0.01;
}

void FrameCalculations::removeFarSphere(QVector<Vorton> &vortons, const double farDistance, const Vector3D bodyCenter)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if ((vortons[i].getMid()-bodyCenter).length()>farDistance)
        {
            vortons.remove(i);
            counters.tooFarNum++;
        }
    }
    timers.farTimer=start.elapsed()*0.01;
}

void FrameCalculations::removeFarCylinder(QVector<Vorton> &vortons, const double farDistance, const double height)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if ((vortons[i].getMid().y()>height+farDistance)||(vortons[i].getMid().y()<-farDistance)||sqrt(pow(vortons[i].getMid().x(),2)+pow(vortons[i].getMid().z(),2))>farDistance)
        {
            vortons.remove(i);
            counters.tooFarNum++;
        }
    }
    timers.farTimer=start.elapsed()*0.01;
}

void FrameCalculations::removeFarRotationBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter)
{
    removeFarSphere(vortons,farDistance,bodyCenter);
}

void FrameCalculations::removeFarRotationCutBody(QVector<Vorton> &vortons, const double farDistance, const Vector3D bodyCenter)
{
    removeFarSphere(vortons,farDistance,bodyCenter);
}

void FrameCalculations::displacementCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove)
{
    QTime start = QTime::currentTime();
    QVector<Vorton> vortons;
    vortons.append(freeVortons);
    vortons.append(newVortons);
    QVector<Parallel> paralVec;
    for (int i=0; i<vortons.size(); i++)
    {
        Parallel element {&vortons,nullptr, nullptr, i, streamVel, step};
        paralVec.push_back(element);
    }
    QVector<Vorton> resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacement, addToVortonsVec, QtConcurrent::OrderedReduce);

    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=fabs(selfLenBef.length()-selfLenAft.length());
        if (turnAngle>fiMax)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.turnRestr++;
        }
        if (lengthChange>eDelta)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.elongationRestr++;
        }
        if (resultedVec[i].getMove().length()>maxMove)
        {
            resultedVec[i].setMove(Vector3D());
            restrictions.moveRestr++;
        }

    }

    for (int i=0; i<freeVortons.size(); i++)
    {
        freeVortons[i].setMove(resultedVec[i].getMove());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.01;
}

void FrameCalculations::displacementLaunchCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove)
{
    QTime start = QTime::currentTime();
    QVector<Vorton> vortons;
    vortons.append(freeVortons);
    vortons.append(newVortons);
    vortons.append(symFreeVortons);
    vortons.append(symNewVortons);
    int phantomSize=symFreeVortons.size()+symNewVortons.size();
    QVector<Parallel> paralVec;
    for (int i=0; i<vortons.size()-phantomSize; i++)
    {
        Parallel element {&vortons,nullptr, nullptr, i, streamVel, step};
        paralVec.push_back(element);
    }
    QVector<Vorton> resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacement, addToVortonsVec, QtConcurrent::OrderedReduce);

    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=fabs(selfLenBef.length()-selfLenAft.length());
        if (turnAngle>fiMax)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.turnRestr++;
        }
        if (lengthChange>eDelta)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.elongationRestr++;
        }
        if (resultedVec[i].getMove().length()>maxMove)
        {
            resultedVec[i].setMove(Vector3D());
            restrictions.moveRestr++;
        }

    }

    for (int i=0; i<freeVortons.size(); i++)
    {
        freeVortons[i].setMove(resultedVec[i].getMove());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.01;
}

void FrameCalculations::setMatrixSize(int size)
{
    matrixSize=size;
}

void FrameCalculations::setVorticity(QVector<std::shared_ptr<MultiFrame> > frames, const Eigen::VectorXd vorticities)
{
    for (int i=0; i<vorticities.size()-1; i++)
        frames[i]->setVorticity(vorticities(i));
}

QVector<Vorton> FrameCalculations::getFrameVortons(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<Vorton> vortons;
    for (int i=0; i<frames.size(); i++)
        vortons.append(frames[i]->getVortons());
    return vortons;
}

QVector<Vorton> FrameCalculations::getLiftedFrameVortons(QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vector3D> &normals, const double deltaUp)
{
    QVector<Vorton> vortons;
    for (int i=0; i<frames.size(); i++)
        vortons.append(frames[i]->getLiftedVortons(deltaUp*normals[i]));
    return vortons;
}

double FrameCalculations::pressureCalc(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density, QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau)
{
    double velAdd=0.5*(Vector3D::dotProduct(streamVel, streamVel)-pow((FrameCalculations::velocity(point, streamVel, freeVortons, frames).length()),2));
    double scal = 0.0;
    double framesAdd = 0.0;
    for (int i=0; i<freeVortons.size(); i++)
        scal+=Vector3D::dotProduct(freeVortons[i].getMove(), freeVortons[i].velocity(point));
    for (int i=0; i<frames.size(); i++)
        framesAdd+=frames[i]->getVorticity()*frames[i]->fi(point);
    return streamPres+density*(velAdd+scal/tau-framesAdd/tau);
}

Vector3D FrameCalculations::forceCalc(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vorton> &freeVortons, const double tau, const QVector<double> &squares, const QVector<Vector3D> &controlPointsRaised, const QVector<Vector3D> &normals)
{
    QTime start = QTime::currentTime();
    Vector3D force;
    for (int i=0; i<controlPointsRaised.size(); i++)
    {
        double pressure=FrameCalculations::pressureCalc(controlPointsRaised[i], streamVel, streamPres, density, frames, freeVortons, tau);
        force-=pressure*squares[i]*normals[i];
    }
    timers.forceTimer=start.elapsed()*0.001;
    return force;
}

void FrameCalculations::cpSum(const int stepNum, QVector<double> &cp, const int fiFragNum, const double radius, const double pointsRaising, const QVector<double> &tetas, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const Vector3D center)
{
    if (stepNum>=200)
    {
        double fi=M_PI*5.0/fiFragNum;
        for (int i=0; i<tetas.size();i++)
        {
            Vector3D point((radius+pointsRaising)*sin(tetas[i])*cos(fi), (radius+pointsRaising)*sin(tetas[i])*sin(fi),(radius+pointsRaising)*cos(tetas[i]));
            point+=center;
            double pres=pressureCalc(point, streamVel,streamPres,density,frames,freeVortons,tau);
            cp[i]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        }
    }
}

void FrameCalculations::cpAverage(QVector<double> &cp, const int stepsNum)
{
    for (int i=0; i<cp.size(); i++)
        cp[i]/=stepsNum-200;
}

void FrameCalculations::getBackAndRotateSphere(QVector<Vorton> &vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D>& controlPoints, const QVector<Vector3D>& normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideSphere(vortons[i],center,radius))
        {
//            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
//            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
//            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            Vector3D axis=Vector3D(vortons[i].getTail()-center).normalized();
            vortons[i].translate(2*axis*(axis*radius-vortons[i].getMid()).length());
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideSphereLayer(vortons[i],center,radius,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::getBackAndRotateCylinder(QVector<Vorton> &vortons, const double height, const double diameter, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideCylinder(vortons[i],height,diameter))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideCylinderLayer(vortons[i],height,diameter,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::getBackAndRotateRotationBody(QVector<Vorton>& vortons, const double xBeg, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideRotationBody(vortons[i],xBeg,xEnd))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationBodyLayer(vortons[i],xBeg,xEnd,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::getBackAndRotateRotationCutBody(QVector<Vorton> &vortons, const double xBeg, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xBeg,xEnd))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xBeg,xEnd,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::getBackAndRotateRotationCutLaunchedBody(QVector<Vorton> &vortons, const double xBeg, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    QTime start=QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xBeg,xEnd))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xBeg,xEnd,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
        if (FrameCalculations::insideScreen(vortons[i]))
        {
            vortons.remove(i);
            counters.underScreenNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::addToVortonsVec(QVector<Vorton> &vortons, const Vorton vort)
{
    vortons.push_back(vort);
}

VelBsym FrameCalculations::velocityAndBsymm(const Vector3D point,const  Vector3D streamVel, const QVector<Vorton> &vortons)
{
    VelBsym res = VelBsym (streamVel);
    for (int i=0; i<vortons.size(); i++)
        res+=vortons[i].velAndBsym(point);
    return res;
}

Vector3D FrameCalculations::velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<std::shared_ptr<MultiFrame> > frames)
{
    Vector3D vel=streamVel;
    for (int i=0; i<vortons.size(); i++)
        vel+=vortons[i].velocity(point);
    for (int i=0; i<frames.size(); i++)
        vel+=frames[i]->velocity(point);
    return vel;
}

Vector3D FrameCalculations::velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons)
{
    Vector3D vel=streamVel;
    for (int i=0; i<vortons.size(); i++)
        vel+=vortons[i].velocity(point);
    return vel;
}

Vorton FrameCalculations::parallelDisplacement(const Parallel el)
{
    Vorton res=el.Vortons->at(el.num);
    VelBsym vb=FrameCalculations::velocityAndBsymm(el.Vortons->at(el.num).getMid(), el.streamVel, *el.Vortons);
    Vector3D selfLen=el.Vortons->at(el.num).getTail()-el.Vortons->at(el.num).getMid();
    double xElong=vb.B[0][0]*selfLen[0]+vb.B[0][1]*selfLen[1]+vb.B[0][2]*selfLen[2];
    double yElong=vb.B[1][0]*selfLen[0]+vb.B[1][1]*selfLen[1]+vb.B[1][2]*selfLen[2];
    double zElong=vb.B[2][0]*selfLen[0]+vb.B[2][1]*selfLen[1]+vb.B[2][2]*selfLen[2];
    res.setElongation(Vector3D(xElong, yElong, zElong)*el.tau);
    res.setMove(vb.Vel*el.tau);
    return res;
}

void FrameCalculations::displace(QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size(); i++)
    {
        vortons[i].setTail(vortons[i].getTail()+vortons[i].getMove()+vortons[i].getElongation());
        vortons[i].setMid(vortons[i].getMid()+vortons[i].getMove());
    }
}

bool FrameCalculations::insideSphere(const Vorton& vort, const Vector3D& center, const double radius)
{
    if ((vort.getMid()-center).length()<radius)
        return true;
    return false;
}

bool FrameCalculations::insideSphereLayer(const Vorton& vort, const Vector3D& center, const double radius, const double layerHeight)
{
    if ((vort.getMid()-center).length()<radius+layerHeight)
        return true;
    return false;
}

bool FrameCalculations::insideCylinder(const Vorton &vort, const double height, const double diameter)
{
    if ((vort.getMid().y()>0)&&(vort.getMid().y()<height)&&((pow(vort.getMid().x(),2)+pow(vort.getMid().z(),2))<0.25*pow(diameter,2)))
        return true;
    return false;
}

bool FrameCalculations::insideCylinderLayer(const Vorton &vort, const double height, const double diameter, const double layerHeight)
{
    if ((vort.getMid().y()>-layerHeight)&&(vort.getMid().y()<height+layerHeight)&&((pow(vort.getMid().x(),2)+pow(vort.getMid().z(),2))<pow(0.5*diameter+layerHeight,2)))
        return true;
    return false;
}

bool FrameCalculations::insideRotationBody(const Vorton &vort, const double xBeg, const double xEnd)
{
    if((vort.getMid().x()>=xBeg) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z(),2)+pow(vort.getMid().y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getMid().x()),2)))
        return true;
    return false;
}

bool FrameCalculations::insideRotationBodyLayer(const Vorton &vort, const double xBeg, const double xEnd, const double layerHeight)
{
    if((vort.getMid().x()>=xBeg-layerHeight) && (vort.getMid().x()<=xEnd+layerHeight) && ((pow(vort.getMid().z(),2)+pow(vort.getMid().y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getMid().x())+layerHeight,2)))
        return true;
    return false;
}

bool FrameCalculations::insideRotationCutBody(const Vorton &vort, const double xBeg, const double xEnd)
{
    if((vort.getMid().x()>=xBeg) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z(),2)+pow(vort.getMid().y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()),2)))
        return true;
    return false;
}

bool FrameCalculations::insideRotationCutBodyLayer(const Vorton &vort, const double xBeg, const double xEnd, const double layerHeight)
{
    if((vort.getMid().x()>=xBeg-layerHeight) && (vort.getMid().x()<=xEnd+layerHeight) && ((pow(vort.getMid().z(),2)+pow(vort.getMid().y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x())+layerHeight,2)))
        return true;
    return false;
}

bool FrameCalculations::insideScreen(Vorton &vort)
{
    if (vort.getMid().x()<0.0 || vort.getTail().x()<0.0)
        return true;
    return false;
}


bool FrameCalculations::exploseSphere(const QVector<Vorton> &vortons)
{
    int counter=0;
    for (int i=0; i<vortons.size(); i++)
    {
       if (fabs(vortons[i].getMid().y())>2.0||fabs(vortons[i].getMid().x())>2.0||vortons[i].getMid().z()<-1.0)
           counter++;
       if (fabs(vortons[i].getTail().y())>2.0||fabs(vortons[i].getTail().x())>2.0||vortons[i].getTail().z()<-1.0)
           counter++;
    }
    if (counter>=10)
        return true;
    return false;
}

double FrameCalculations::calcDispersion(const QVector<Vector3D> &cAerodynamics)
{
    Vector3D cAver;
    for (int i=100; i<cAerodynamics.size(); i++)
    {
        cAver+=cAerodynamics[i];
    }
    cAver=cAver/(cAerodynamics.size()-100);

    double dispersion=0.0;

    for (int i=100; i<cAerodynamics.size(); i++)
    {
        dispersion+=(cAerodynamics[i]-cAver).lengthSquared()/cAver.length();
    }
    return dispersion;
}

QVector<std::shared_ptr<MultiFrame> > FrameCalculations::copyFrames(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<std::shared_ptr<MultiFrame>> copyingFrames;
    for (int i=0; i<frames.size(); i++)
    {
        copyingFrames.push_back(std::make_shared<MultiFrame>(*frames[i].get()));
    }
    return copyingFrames;
}

void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised, Vector3D &center, double &xbeg, double &xend)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }
   center+=translation;
   xbeg+=translation.x();
   xend+=translation.x();
}

void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised, Vector3D &center)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }
   center+=translation;
}

void FrameCalculations::translateVortons(const Vector3D &translation, QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size(); i++)
        vortons[i].translate(translation);
}

Counters FrameCalculations::getCounters() const
{
    return counters;
}

Restrictions FrameCalculations::getRestrictions() const
{
    return restrictions;
}

Timers FrameCalculations::getTimers() const
{
    return timers;
}

void FrameCalculations::clearCounters()
{
    counters.clear();
}

void FrameCalculations::clearRestrictions()
{
    restrictions.clear();
}

void FrameCalculations::clearTimers()
{
    timers.clear();
}

void FrameCalculations::clear()
{
    clearTimers();
    clearRestrictions();
    clearCounters();
}

