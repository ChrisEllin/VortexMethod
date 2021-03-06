﻿#include "framecalculations.h"

/*!
Обнуляет значения всех счетчиков 
*/
void Counters::clear()
{
    unitedNum=vorticityEliminated=tooFarNum=rotatedNum=gotBackNum=underScreenNum=gotBackDeletedNum=0;
}

/*!
Обнуляет значения всех ограничений
*/
void Restrictions::clear()
{
    moveRestr=elongationRestr=turnRestr=0;
}

/*!
Обнуляет значения всех таймеров
*/
void Timers::clear()
{
    getBackAndRotateTimer=forceTimer=unionTimer=farTimer=integrationTimer=removeVorticityTimer=rotateTimer=0.0;
}

/*!
Создает объект класса для работы с рамками с обнуленными счетчиками, ограничениями и таймерами
*/
FrameCalculations::FrameCalculations()
{

    counters.clear();
    restrictions.clear();
    timers.clear();
}

double FrameCalculations::getConditionalNum()
{
    return conditionalNum;
}

/*!
Рассчитывает значения углов тета для сферы
\param tetaFragNum Количество разбиений тела по тета
\return Вектор с рассчитанными значениями углов тета
*/
QVector<double> FrameCalculations::calcTetas(const int tetaFragNum)
{
    QVector<double> tetas;
    tetas.push_back(0.0);
    for (int i=1; i<tetaFragNum+2;i++)
        tetas.push_back(M_PI/(tetaFragNum+2)*(0.5+i));
    tetas.push_back(M_PI);
    return tetas;
}



QVector<QPair<int, int> > FrameCalculations::unionFrameVortons(QVector<Vorton> frameVortons, const double eDoubleStar)
{
    QTime start = QTime::currentTime();
    QVector<QPair<int,int>> unionSchedule;
    QVector<int> iIndexes;
    QVector<int> jIndexes;
    for (int i=frameVortons.size()-1; i>=0; i--)
    {
        if (jIndexes.indexOf(i)!=-1)
            continue;
        for (int j=i-1; j>=0; j--)
        {
            if(qFuzzyIsNull((frameVortons[i].getMid()-frameVortons[j].getMid()).length()))
            {
                double psi=Vector3D::dotProduct((frameVortons[i].getTail()-frameVortons[i].getMid()).normalized(),(frameVortons[j].getTail()-frameVortons[j].getMid()).normalized());
                if (fabs(psi)>eDoubleStar)
                {
                    iIndexes.push_back(i);
                    jIndexes.push_back(j);
                    QPair<int,int> schedule(i,j);
                    unionSchedule.push_front(schedule);
                    //i--;
                    //frameVortons.remove(j);
                    break;
                }

            }
        }
    }

    timers.unionTimer=start.elapsed()*0.001;
    return unionSchedule;
}

void FrameCalculations::maxVortonsGamma(QVector<Vorton> &freeVortons,double maxInitialGamma)
{
    for (int i=0; i<freeVortons.size();i++)
    {
       if (fabs(freeVortons[i].getVorticity())>fabs(maxInitialGamma))
           qDebug()<<"Vorton № "<<i<<"has big gamma ="<<freeVortons[i].getVorticity();
    }
}

double FrameCalculations::getMaxGamma(QVector<Vorton> &vortons)
{
    double maxG=vortons[0].getVorticity();
    for (int i=1; i<vortons.size();i++)
        if (fabs(vortons[i].getVorticity())>fabs(maxG))
            maxG=vortons[i].getVorticity();
    return maxG;
}

void FrameCalculations::maxVortonsVelocity(const QVector<Vorton> &freeVortons, Vector3D velocityInf, double tau)
{
    for (int i=0; i<freeVortons.size();i++)
    {
       if (freeVortons[i].getMove().length()/tau>velocityInf.length())
           qDebug()<<"Vorton № "<<i<<"has big velocity ="<<freeVortons[i].getMove().x()/tau<<" "<<freeVortons[i].getMove().y()/tau<<" "<<freeVortons[i].getMove().z()/tau;
    }
}

void FrameCalculations::unionVortonsAuto(QVector<Vorton> &vortons, const double eStar, const double eDoubleStar, const double vortonRad, QVector<std::shared_ptr<MultiFrame> > &frames, const double maxInitialGamma, int currentStep, int &restrQuant, QVector<std::pair<double, double> > boundaries)
{
    restrQuant=0;
    if (currentStep<2)
        unionVortons(vortons,eStar,eDoubleStar,vortonRad,frames,boundaries);
    else {
        unionWithRestrictions(vortons,eStar,eDoubleStar,vortonRad,frames,maxInitialGamma,restrQuant,boundaries);
    }
}

void FrameCalculations::updateSpeed(Vector3D &currentSpeed, Vector3D streamVel, int acceleratedStepsNum, MotionType type)
{
    if (type==ACCELERATED && currentSpeed.length()<= streamVel.length())
    {
        if ((currentSpeed+streamVel/(acceleratedStepsNum+2)).length()<streamVel.length())
            currentSpeed+=streamVel/(acceleratedStepsNum+2);
        else
            currentSpeed=streamVel;
    }
}

QVector<double> FrameCalculations::normalVelocitiesCalculations(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons,QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &normals, QVector<Vector3D> &controlPoints, Vector3D streamVel, QVector<std::shared_ptr<MultiFrame>>& symFrames,PLACE_IN_SOLVER place)
{
    QVector<double> velocities;
    switch (place)
    {
    case AFTER_SLAU:
    {
        for (int i=0; i<controlPoints.size();i++)
            velocities.push_back(Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],streamVel,freeVortons,frames),normals[i]));
        for (int i=0; i<controlPoints.size();i++)
            velocities[i]+=(Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],Vector3D(),freeVortons,symFrames),normals[i]));
        qDebug()<<*std::max_element(velocities.begin(),velocities.end(),Vector3D::fabsCompare);
        break;
    }
    case AFTER_EPSILON:
    {
        for (int i=0; i<controlPoints.size();i++)
            velocities.push_back(Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],streamVel,freeVortons+newVortons),normals[i]));
        break;
    }
    case AFTER_INTEGRATION:
    {
        for (int i=0; i<controlPoints.size();i++)
            velocities.push_back(Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],streamVel,freeVortons),normals[i]));
        break;
    }
    case IN_THE_END:
    {
        for (int i=0; i<controlPoints.size();i++)
            velocities.push_back(Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],streamVel,freeVortons),normals[i]));
        break;
    }
    };
    return velocities;
}

void FrameCalculations::calcSymParameters(QVector<Vorton> &symNewVortons, QVector<Vorton> &symFreeVortons, const QVector<Vorton> newVortons, const QVector<Vorton> freeVortons, QVector<std::shared_ptr<MultiFrame> > &symFrames, QVector<std::shared_ptr<MultiFrame> > &frames, bool underwaterStart)
{
    if (underwaterStart)
    {
    symNewVortons=newVortons;
    symFreeVortons=freeVortons;
    symFrames=FrameCalculations::copyFrames(frames);
    FrameCalculations::reflect(symFreeVortons,symNewVortons,symFrames);
    }
}

void FrameCalculations::getBackUnderBody(QVector<Vorton> &freeVortons, QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    for (int k=0; k<freeVortons.size(); k++)
        getBackClosestFrame(freeVortons[k],boundaries,frames);
}

void FrameCalculations::getBackClosestFrame(Vorton &vort,QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    bool gotBack = false;
    if (insideFramesVector(frames, vort,boundaries, VortonsPart::Center))
    {
        //если вортон внутри, то оразить центр от ближайшей рамки
        QPair<int,int> num = findMaxSolidAngle(vort,frames);
        Vector3D norm = frames[num.first]->getTriangle(num.second).getNormal().normalized();

        //строго назад
        Vector3D translation = Vector3D::dotProduct(frames[num.first]->getTriangle(num.second).getCenter()-vort.getMid(),norm)*2.0*norm;

        //с сохранением касательной компоненты
        //Vector3D translation = vort.move-Vector3D::dotProduct(norm,vort.move)*norm;

        vort.translate(translation);
        gotBack = true;

        if (insideFramesVector(frames, vort,boundaries, VortonsPart::Center))
            qDebug()<<"SymmetricalGetBackWasNotSuccessfull";
    }

    //если после центр вортона снаружи, а какой-то из концов внутри, надо развернуть вортон
    if (insideFramesVector(frames, vort,boundaries, VortonsPart::Tail)||insideFramesVector(frames, vort,boundaries, VortonsPart::Beginning))
    {
        //найти рамку, пересекаемую отрезком
        QPair <int, int> answer1 = intersectsFramesVector(frames, vort,VortonsPart::Tail);
        if (answer1.second!=-1)
        {
            vort.rotateAroundNormal(frames[answer1.first]->getTriangle(answer1.second).getNormal().normalized());

            //vortonCounters.NrotGetBack++;
            //vortonCounters.Nback++
        }

    }
}

void FrameCalculations::getBackClosestFrameEdge(Vorton &vort, QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    bool gotBack = false;
    Vector3D norm, translation;
    if (insideFramesVector(frames, vort,boundaries, VortonsPart::Tail))
    {
        //если вортон внутри, то оразить центр от ближайшей рамки
        QPair<int,int> num = findMaxSolidAngle(vort,frames);
        norm = frames[num.first]->getTriangle(num.second).getNormal().normalized();

        //строго назад
        translation = Vector3D::dotProduct(frames[num.first]->getTriangle(num.second).getCenter()-vort.getMid(),norm)*2.0*norm;

        //с сохранением касательной компоненты
        //Vector3D translation = vort.move-Vector3D::dotProduct(norm,vort.move)*norm;

        vort.translate(translation);
        gotBack = true;

        if (insideFramesVector(frames, vort,boundaries, VortonsPart::Center))
            qDebug()<<"SymmetricalGetBackWasNotSuccessfull";
    }

    //если после центр вортона снаружи, а какой-то из концов внутри, надо развернуть вортон
    if (insideFramesVector(frames, vort,boundaries, VortonsPart::Beginning))
    {
        //найти рамку, пересекаемую отрезком
        QPair<int,int> num = findMaxSolidAngle(vort,frames);
        Vector3D norm2 = frames[num.first]->getTriangle(num.second).getNormal().normalized();
        Vector3D translation2 = Vector3D::dotProduct(frames[num.first]->getTriangle(num.second).getCenter()-vort.getMid(),norm2)*2.0*norm2;
        if (translation2.length()>translation.length())
        {
            translation=translation2;
            norm=norm2;

            //vortonCounters.NrotGetBack++;
            //vortonCounters.Nback++
        }

    }
    vort.translate(translation);
}

void FrameCalculations::recalcTrFrames(QVector<std::shared_ptr<MultiFrame> > &frames)
{
    for (int i=0;i<frames.size();i++)
        frames[i]->makeTriangles();
}

void FrameCalculations::recalcTau(double &tau, Vector3D currentVel, double panelLength)
{
    tau=panelLength/currentVel.length();
}

void FrameCalculations::updateEnd(double &xend,const Vector3D bodyNose, const double fullLength)
{
    if (bodyNose.x()+fullLength>=0.0)
        xend=0.0;
    else {
        xend=bodyNose.x()+fullLength;
    }
}

void FrameCalculations::updateImportantVectors(int step, QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals, QVector<double> &squares, QVector<Vector3D> &controlPointsRaised, QVector<std::shared_ptr<MultiFrame> > &frames, BodyFragmentation& frag,bool underwaterStart)
{
    if (underwaterStart || step==0)
    {
        controlPoints=frag.getControlPoints();
        normals=frag.getNormals();
        controlPointsRaised=frag.getControlPointsRaised();
        squares=frag.getSquares();
        frames=frag.getFrames();

    }
}

void FrameCalculations::calculateRelativeVel(const int step, const Vector3D streamVel, const Vector3D bodyVel, const int acceleratedNum, const MotionType motion, Vector3D& currentSpeed, bool underwaterStart)
{
    if (motion==ACCELERATED)
    {
        if (step==0)
        {
            currentSpeed=streamVel/(acceleratedNum+2);
        }
        else {
            updateSpeed(currentSpeed,streamVel,acceleratedNum,motion);
        }
    }
    else
        currentSpeed=streamVel;

    if (underwaterStart)
        currentSpeed=streamVel-bodyVel;

}

void FrameCalculations::fillPressures(QVector<Vorton> &freeVortons, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &controlPointsRaised, QVector<double> &pressures,Vector3D& streamVel, double streamPres, double density, double tau)
{
    pressures.clear();
    for (int i=0; i<controlPointsRaised.size(); i++)
        pressures.push_back((FrameCalculations::pressureCalc(controlPointsRaised[i], streamVel, streamPres,density, frames, freeVortons, tau)-streamPres)/
                (density*Vector3D::dotProduct(streamVel,streamVel)*0.5));
}

void FrameCalculations::setMaxInitialGamma(double &maxGamma, const int &stepNum, const QVector<Vorton>& freeVortons)
{
    if (stepNum==0)
    {
        maxGamma=freeVortons[0].getVorticity();
        for (int i=1; i<freeVortons.size();i++)
        {
            if (fabs(freeVortons[i].getVorticity())>fabs(maxGamma))
                maxGamma=freeVortons[i].getVorticity();
        }
    }

}

void FrameCalculations::setRightMove(QVector<Vorton> &originalVort, QVector<Vorton> &copyVort)
{
    for (int i=0; i<copyVort.size();i++)
        copyVort[i].setMove(originalVort[i].getMove());
}

QVector<QVector<double> > FrameCalculations::createCpArray(BodyType type, FragmentationParameters fragPar)
{
    switch (type) {
    case ROTATIONBODY:
    {
        QVector<double> cp0(fragPar.rotationBodyPartFragNum+1);
        QVector<double> cp90(fragPar.rotationBodyPartFragNum+1);
        QVector<double> cp180(fragPar.rotationBodyPartFragNum+1);
        QVector<double> cp270(fragPar.rotationBodyPartFragNum+1);
        QVector<QVector<double>> cpArray;
        cpArray.append(cp0);
        cpArray.append(cp90);
        cpArray.append(cp180);
        cpArray.append(cp270);
        return  cpArray;
    }
    case ROTATIONBOTTOMCUT:
    {
        QVector<double> cp0(fragPar.rotationBodyPartFragNum+1);
        QVector<double> cp90(fragPar.rotationBodyPartFragNum+1);
        QVector<double> cp180(fragPar.rotationBodyPartFragNum+1);
        QVector<double> cp270(fragPar.rotationBodyPartFragNum+1);
        QVector<QVector<double>> cpArray;
        cpArray.append(cp0);
        cpArray.append(cp90);
        cpArray.append(cp180);
        cpArray.append(cp270);
        return  cpArray;
    }
    case ROTATIONTWOBOTTOM:
    {
        QVector<double> cp0(fragPar.rotationBodyFiFragNum);

        QVector<QVector<double>> cpArray;
        cpArray.append(cp0);
        return  cpArray;
    }
    }
}

void FrameCalculations::setRightMove(QVector<Vorton> &freeVortons, QVector<Vorton> &copyVort, QVector<Vorton> &newVortons, QVector<Vorton> &copyNewVort)
{
    for (int i=0; i<copyVort.size();i++)
        copyVort[i].setMove(freeVortons[i].getMove());

    for (int i=0; i<copyNewVort.size();i++)
        copyNewVort[i].setMove(newVortons[i].getMove());

}

FrameData FrameCalculations::framesMagic(QVector<std::shared_ptr<MultiFrame> > &frames,QVector<Vector3D>& controlPoints,QVector<Vector3D>& controlPointsRaised,QVector<Vector3D>& normals,QVector<double>& squares)
{


    MultiFrame backFrame=*frames.last().get();
    frames.remove(frames.size()-1);
    Vector3D controlPoint=controlPoints.last();
    Vector3D normal=normals.last();
    Vector3D controlPointRaised=controlPointsRaised.last();
    double square=squares.last();
    controlPoints.remove(controlPoints.size()-1);
    controlPointsRaised.remove(controlPointsRaised.size()-1);
    normals.remove(normals.size()-1);
    squares.remove(squares.size()-1);
    FrameData data {backFrame,controlPoint,controlPointRaised,normal,square,true};
    return  data;
}

void FrameCalculations::reverseMagic(QVector<std::shared_ptr<MultiFrame> > &frames,QVector<Vector3D>& controlPoints,QVector<Vector3D>& controlPointsRaised,QVector<Vector3D>& normals,QVector<double>& squares,FrameData &framesAdd)
{
    if (framesAdd.full)
    {
    frames.push_back(std::make_shared<MultiFrame>(framesAdd.frame));
    controlPoints.push_back(framesAdd.controlPoint);
    controlPointsRaised.push_back(framesAdd.controlPointRaised);
    normals.push_back(framesAdd.normal);
    squares.push_back(framesAdd.square);
    framesAdd.clear();
    }
}

void FrameCalculations::unionWithSchedule(QVector<Vorton> &vortons, QVector<QPair<int, int> > &schedule)
{
    QVector<Vorton> correctVortons;
    for (int i=0; i<schedule.size(); i++)
    {
        double psi=Vector3D::dotProduct((vortons[schedule[i].first].getTail()-vortons[schedule[i].first].getMid()).normalized(),
                (vortons[schedule[i].second].getTail()-vortons[schedule[i].second].getMid()).normalized());
        if (psi<0)
            vortons[schedule[i].second].turn();
        counters.unitedNum++;
        Vector3D newMid = (fabs(vortons[schedule[i].first].getVorticity())*vortons[schedule[i].first].getMid()+
                fabs(vortons[schedule[i].second].getVorticity())*vortons[schedule[i].second].getMid())/
                (fabs(vortons[schedule[i].first].getVorticity())+fabs(vortons[schedule[i].second].getVorticity()));

        Vector3D selfLen = (fabs(vortons[schedule[i].first].getVorticity())*(vortons[schedule[i].first].getTail()-vortons[schedule[i].first].getMid())
                +fabs(vortons[schedule[i].second].getVorticity())*(vortons[schedule[i].second].getTail()-vortons[schedule[i].second].getMid()))/
                (fabs(vortons[schedule[i].first].getVorticity())+fabs(vortons[schedule[i].second].getVorticity()));
        double newVorticity=vortons[schedule[i].first].getVorticity()+vortons[schedule[i].second].getVorticity();
        Vorton newVorton = Vorton (newMid, newMid+selfLen, newVorticity, vortons[0].getRadius());
        correctVortons.push_back(newVorton);
    }
    vortons=correctVortons;
}

void FrameCalculations::epsZero(QVector<std::shared_ptr<MultiFrame>> &frames)
{
    for (int i=0; i<frames.size();i++)
        frames[i]->setRadius(0.0);
}

void FrameCalculations::epsNormal(QVector<Vorton> &newVortons, double eps)
{
    for (int i=0; i<newVortons.size();i++)
        newVortons[i].setRadius(eps);
}

void FrameCalculations::epsNormal(QVector<std::shared_ptr<MultiFrame> > &frames, double eps)
{
    for (int i=0; i<frames.size();i++)
        frames[i]->setRadius(eps);
}

bool FrameCalculations::fabsCompare(Vector3D a, Vector3D b)
{
    if (a.length()>b.length())
        return  true;
    return false;
}

/*!
Рассчитывает значения элементов матрицы, составленной из произведения единичных интенсивностей от каждой из рамок на соответствующую нормаль и вычисляет обратную к ней
\param frames Вектор рамок
\param controlPoints Вектор контрольных точек
\param normals Вектор нормалей
*/
void FrameCalculations::matrixCalc(QVector<std::shared_ptr<MultiFrame>> frames,QVector<std::shared_ptr<MultiFrame>> symFrames, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, bool underwater)
{




    matrixSize=frames.size()+1;
    matrix.resize(matrixSize,matrixSize);
    for (int i=0; i<matrixSize-1; i++)
    {
        for (int j=0; j<matrixSize-1; j++)
        {
            //matrix(i,j)=Vector3D::dotProduct(frames[j]->qHelp(controlPoints[i]), normals[i]);
            matrix(i,j)=Vector3D::dotProduct(frames[j]->qHelp(controlPoints[i]), normals[i]);
            matrix(i,j)-=Vector3D::dotProduct(symFrames[j]->qHelp(controlPoints[i]), normals[i]);
        }

    }
    for (int i=0; i<matrixSize-1; i++)
        matrix(matrixSize-1,i)=matrix(i,matrixSize-1)=1.0; 
    matrix(matrixSize-1,matrixSize-1)=0.0;

    double tqw=matrix.determinant();
    QVector<double> mat(matrixSize);
    for (int i=0; i<matrixSize; i++)
    {
        mat[i]=0.0;
        for (int j=0; j<matrixSize; j++)
        {
            mat[i]+=matrix(i,j);
        }
    }
    double normA=mat[0];
    for (int i=1; i<matrixSize; i++)
    {
        if (mat[i]>normA)
            normA=mat[i];
    }
//    Eigen::MatrixXd oldmatrix;
//    oldmatrix.resize(matrixSize,matrixSize);
//    oldmatrix=matrix;

    matrix=matrix.inverse();
    QVector<double> mat1(matrixSize);
    for (int i=0; i<matrixSize; i++)
    {
        for (int j=0; j<matrixSize; j++)
        {
            mat1[i]+=matrix(i,j);
        }
    }
    double normA1=mat1[0];
    for (int i=1; i<matrixSize; i++)
    {
        if (mat1[i]>normA1)
            normA1=mat1[i];
    }
    conditionalNum=normA*normA1;

//    matrix=matrix.inverse();

}

void FrameCalculations::matrixCalc(QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, bool underwater)
{
    matrixSize=frames.size()+1;
    matrix.resize(matrixSize,matrixSize);
    for (int i=0; i<matrixSize-1; i++)
    {
        for (int j=0; j<matrixSize-1; j++)
        {
            //matrix(i,j)=Vector3D::dotProduct(frames[j]->qHelp(controlPoints[i]), normals[i]);
            matrix(i,j)=Vector3D::dotProduct(frames[j]->qHelp(controlPoints[i]), normals[i]);

        }

    }
    for (int i=0; i<matrixSize-1; i++)
        matrix(matrixSize-1,i)=matrix(i,matrixSize-1)=1.0;
    matrix(matrixSize-1,matrixSize-1)=0.0;

    double tqw=matrix.determinant();
    QVector<double> mat(matrixSize);
    for (int i=0; i<matrixSize; i++)
    {
        mat[i]=0.0;
        for (int j=0; j<matrixSize; j++)
        {
            mat[i]+=matrix(i,j);
        }
    }
    double normA=mat[0];
    for (int i=1; i<matrixSize; i++)
    {
        if (mat[i]>normA)
            normA=mat[i];
    }
//    Eigen::MatrixXd oldmatrix;
//    oldmatrix.resize(matrixSize,matrixSize);
//    oldmatrix=matrix;

    matrix=matrix.inverse();
    QVector<double> mat1(matrixSize);
    for (int i=0; i<matrixSize; i++)
    {
        for (int j=0; j<matrixSize; j++)
        {
            mat1[i]+=matrix(i,j);
        }
    }
    double normA1=mat1[0];
    for (int i=1; i<matrixSize; i++)
    {
        if (mat1[i]>normA1)
            normA1=mat1[i];
    }
    conditionalNum=normA*normA1;

//    matrix=matrix.inverse();
}
/*!
Рассчитывает значения столбца b для решения СЛАУ вида A*x=b, где А-матрица, х-искомый столбец.
\param streamVel Скорость потока
\param vortons Вектор содержащий текущие вортоны в потоке
\param normals Вектор нормалей
\param controlPoints Вектор контрольных точек
\return Столбец b
*/

Eigen::VectorXd FrameCalculations::columnCalc(const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<Vector3D> &normals, const QVector<Vector3D> controlPoints)
{
        Eigen::VectorXd column(matrixSize);
        for (int i=0; i<controlPoints.size(); i++)
            column(i)=Vector3D::dotProduct(-FrameCalculations::velocity(controlPoints[i], streamVel, vortons), normals[i]);
        column(controlPoints.size())=0.0;
        return column;
}


Eigen::VectorXd FrameCalculations::columnCalc(const Vector3D streamVel, const QVector<Vorton> vortons, const QVector<Vector3D> &normals, const Vector3D angularVel, const QVector<Vector3D> &controlPoints, const Vector3D center)
{
    Eigen::VectorXd column(matrixSize);
    for (int i=0; i<controlPoints.size(); i++)
        column(i)=Vector3D::dotProduct(-(FrameCalculations::velocity(controlPoints[i], streamVel, vortons)+Vector3D::crossProduct(angularVel, controlPoints[i]-center)), normals[i]);

    column(controlPoints.size())=0.0;
    return column;
}

/*!
Рассчитывает значения столбца x путем решения СЛАУ вида A*x=b, где А-матрица, b-известный столбец.
\param column Столбец b
\return Столбец x, содержащий значения завихренностей рамок
*/
Eigen::VectorXd FrameCalculations::vorticitiesCalc(const Eigen::VectorXd &column)
{
    return Eigen::VectorXd(matrix*column);
}

int FrameCalculations::universalInside(const Vorton vort,const QVector<std::pair<double,double>> boundaries, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    if ((vort.getTail().x()>boundaries[0].second && (2.0*vort.getMid()-vort.getTail()).x()>boundaries[0].second) || (( vort.getTail().x()<boundaries[0].first) &&
            (2.0*vort.getMid()-vort.getTail()).x()<boundaries[0].first))
        return -1;
    if ((vort.getTail().y()>boundaries[1].second && (2.0*vort.getMid()-vort.getTail()).y()>boundaries[1].second) || (( vort.getTail().y()<boundaries[1].first) &&
               (2.0*vort.getMid()-vort.getTail()).y()<boundaries[1].first))
        return -1;
    if ((vort.getTail().z()>boundaries[2].second && (2.0*vort.getMid()-vort.getTail()).z()>boundaries[2].second) || (( vort.getTail().z()<boundaries[2].first) &&
             (2.0*vort.getMid()-vort.getTail()).z()<boundaries[2].first))
        return -1;

    srand(time(NULL));
    int num=rand() % frames.size();
    Vector3D choosen=frames[num]->at(0).getTail();
    for (int i=0; i<frames.size();i++)
    {
        if (i!=num)
        {
            if (frames[i]->getAnglesNum()==4)
            {
                Vector3D e1=frames[i]->at(1).getTail()-frames[i]->at(0).getTail();
                Vector3D e2=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                Vector3D n=Vector3D::crossProduct(e1,e2);
                Vector3D tau=(vort.getTail()-choosen).normalized();
                double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                if (t<=0.0||t>=1.0)
                {
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(1).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(2).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {
                        if (!(Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail())>0))
                            return 1;
                    }
                }
                tau=2.0*vort.getMid()-vort.getTail()-choosen;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                if (t<=0.0||t>=1.0)
                {
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(1).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(2).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {
                        if (!(Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()))>0))
                            return 0;
                    }
                }



                e1=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                e2=frames[i]->at(3).getTail()-frames[i]->at(0).getTail();
                n=Vector3D::crossProduct(e1,e2);
                tau=(vort.getTail()-choosen).normalized();
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                if (t<=0.0||t>=1.0)
                {
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(2).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(3).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {
                        if (!(Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail())>0))
                            return 1;
                    }
                }
                tau=2.0*vort.getMid()-vort.getTail()-choosen;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                if (t<=0.0||t>=1.0)
                {
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(2).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(3).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {
                        if (!(Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()))>0))
                            return 0;
                    }
                }


            }
            else {
                for (int j=1;j<frames[i]->getAnglesNum()-1;j++)
                {
                    Vector3D e1=frames[i]->at(j).getTail()-frames[i]->at(0).getTail();
                    Vector3D e2=frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail();
                    Vector3D n=Vector3D::crossProduct(e1,e2);
                    Vector3D tau=vort.getTail()-choosen;
                    double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    if (t<=0.0||t>=1.0)
                    {
                        Vector3D rtilda=choosen+t*tau;
                        Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                        Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                        Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;
                        if (coDirectionallyCheck(a1,a2,a3))
                        {
                            if (!(Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail())>0))
                                return 1;
                        }
                    }
                    tau=2.0*vort.getMid()-vort.getTail()-choosen;
                    t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    if (t<=0.0||t>=1.0)
                    {
                        Vector3D rtilda=choosen+t*tau;
                        Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                        Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                        Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;
                        if (coDirectionallyCheck(a1,a2,a3))
                        {
                            if (!(Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()))>0))
                                return 0;
                        }
                    }
                }
            }
        }
    }
   return -1;

}

void FrameCalculations::clearMovesWithoutIndexes(QVector<int> &indexes, QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size();i++)
    {
       if (indexes.indexOf(i)==-1)
           vortons[i].setMove(Vector3D());
    }
}

Inside FrameCalculations::universalInsideCorrect(const Vorton vort, const QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    Inside ress;
    if ((vort.getTail().x()>boundaries[0].second && (2.0*vort.getMid()-vort.getTail()).x()>boundaries[0].second) || (( vort.getTail().x()<boundaries[0].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).x()<boundaries[0].first))
        return Inside {-1,Vector3D(),0};
    if ((vort.getTail().y()>boundaries[1].second && (2.0*vort.getMid()-vort.getTail()).y()>boundaries[1].second) || (( vort.getTail().y()<boundaries[1].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).y()<boundaries[1].first))
        return Inside {-1,Vector3D(),0};
    if ((vort.getTail().z()>boundaries[2].second && (2.0*vort.getMid()-vort.getTail()).z()>boundaries[2].second) || (( vort.getTail().z()<boundaries[2].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).z()<boundaries[2].first))
        return Inside {-1,Vector3D(),0};

    srand(time(NULL));
    int num=/*rand() % frames.size()*/ 1;
    Vector3D choosen=frames[num]->at(0).getTail()+0.25*(frames[num]->at(1).getTail()-frames[num]->at(0).getTail())
            +0.25*(frames[num]->at(2).getTail()-frames[num]->at(0).getTail());
    for (int i=0; i<frames.size();i++)
    {
        if (i!=num )
        {
            if (frames[i]->getAnglesNum()==4)
            {
                Vector3D e1=frames[i]->at(1).getTail()-frames[i]->at(0).getTail();
                Vector3D e2=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                Vector3D n=Vector3D::crossProduct(e1,e2);
                Vector3D tau=(vort.getTail()-choosen);
                double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);


                Vector3D rtilda=choosen+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(1).getTail()-rtilda;
                Vector3D a3=frames[i]->at(2).getTail()-rtilda;


                if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                        coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                {
                    double dot=Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail());
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                           return Inside {1, rtilda,i};
                }
                tau=2.0*vort.getMid()-vort.getTail()-choosen;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                rtilda=choosen+t*tau;
                a1=frames[i]->at(0).getTail()-rtilda;
                a2=frames[i]->at(1).getTail()-rtilda;
                a3=frames[i]->at(2).getTail()-rtilda;
                if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                        coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                {
                    double dot=Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()));
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                            return Inside {0, rtilda,i};
                }

                e1=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                e2=frames[i]->at(3).getTail()-frames[i]->at(0).getTail();
                n=Vector3D::crossProduct(e1,e2);
                tau=(vort.getTail()-choosen);
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);

                rtilda=choosen+t*tau;
                a1=frames[i]->at(0).getTail()-rtilda;
                a2=frames[i]->at(2).getTail()-rtilda;
                a3=frames[i]->at(3).getTail()-rtilda;
                if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                        coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                {
                    double dot=Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail());
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                            return Inside {1, rtilda,i};
                }
                tau=2.0*vort.getMid()-vort.getTail()-choosen;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                rtilda=choosen+t*tau;
                a1=frames[i]->at(0).getTail()-rtilda;
                a2=frames[i]->at(2).getTail()-rtilda;
                a3=frames[i]->at(3).getTail()-rtilda;
                if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                        coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                {
                    double dot=Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()));
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                                return Inside {0, rtilda,i};
                }


            }
            else {
                for (int j=1;j<frames[i]->getAnglesNum()-1;j++)
                {

                    Vector3D e1=frames[i]->at(j).getTail()-frames[i]->at(0).getTail();
                    Vector3D e2=frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail();
                    Vector3D n=Vector3D::crossProduct(e1,e2);
                    Vector3D tau=vort.getTail()-choosen;
                    double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;
                    if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                            coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                    {
                        double dot=Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail());
                        if (!((dot)>0||qFuzzyIsNull(dot)))
                        return Inside {1, rtilda,i};
                    }
                    tau=2.0*vort.getMid()-vort.getTail()-choosen;
                    t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    rtilda=choosen+t*tau;
                    a1=frames[i]->at(0).getTail()-rtilda;
                    a2=frames[i]->at(j).getTail()-rtilda;
                    a3=frames[i]->at(j+1).getTail()-rtilda;
                    if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                            coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                    {
                        double dot=Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()));
                        if (!((dot)>0||qFuzzyIsNull(dot)))
                                    return Inside {0, rtilda,i};
                    }
                }
            }
        }
    }
    return Inside {-1,Vector3D(), 0};
}

void FrameCalculations::testInsideCorrect(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vorton> &vortons, QVector<std::pair<double, double> > boundaries)
{
    for (int v=0; v<vortons.size();v++)
    {
        if ((vortons[v].getTail().x()>boundaries[0].second && (2.0*vortons[v].getMid()-vortons[v].getTail()).x()>boundaries[0].second) || (( vortons[v].getTail().x()<boundaries[0].first) &&
                                                                                                                        (2.0*vortons[v].getMid()-vortons[v].getTail()).x()<boundaries[0].first))
            continue;
       if ((vortons[v].getTail().y()>boundaries[1].second && (2.0*vortons[v].getMid()-vortons[v].getTail()).y()>boundaries[1].second) || (( vortons[v].getTail().y()<boundaries[1].first) &&
                                                                                                                        (2.0*vortons[v].getMid()-vortons[v].getTail()).y()<boundaries[1].first))
           continue;
       if ((vortons[v].getTail().z()>boundaries[2].second && (2.0*vortons[v].getMid()-vortons[v].getTail()).z()>boundaries[2].second) || (( vortons[v].getTail().z()<boundaries[2].first) &&
                                                                                                                        (2.0*vortons[v].getMid()-vortons[v].getTail()).z()<boundaries[2].first))
           continue;

    Vector3D choosen=frames[0]->getTriangle(0).getCenter();
    bool r0=false;
    bool r1=false;
    bool r2=false;

    for (int i=0;i<frames.size();i++)
    {
        bool checking;
        if (i==0)
            checking=true;
        else checking=false;
        if (frames[i]->inside(choosen,vortons[v].getMid(),0,checking))
            r0=true;
        if (frames[i]->inside(choosen,vortons[v].getTail(),0,checking))
            r1=true;
        if (frames[i]->inside(choosen,2.0*vortons[v].getMid()-vortons[v].getTail(),0,checking))
            r2=true;
    }
    if (r0)
        qDebug()<<"Center №"<<v<<" is inside";
    if (r1)
        qDebug()<<"Tail №"<<v<<" is inside";
    if (r2)
        qDebug()<<"Beginning №"<<v<<" is inside";
    }
    qDebug()<<"End of checking";
    return;
}

bool FrameCalculations::insideFramesVector(QVector<std::shared_ptr<MultiFrame> > &frames, Vorton vort,QVector<std::pair<double, double> > boundaries, VortonsPart part)
{
  if ((vort.getTail().x()>boundaries[0].second && (2.0*vort.getMid()-vort.getTail()).x()>boundaries[0].second) || (( vort.getTail().x()<boundaries[0].first) &&
                                                                                                                  (2.0*vort.getMid()-vort.getTail()).x()<boundaries[0].first))
     return false;
 if ((vort.getTail().y()>boundaries[1].second && (2.0*vort.getMid()-vort.getTail()).y()>boundaries[1].second) || (( vort.getTail().y()<boundaries[1].first) &&
                                                                                                                  (2.0*vort.getMid()-vort.getTail()).y()<boundaries[1].first))
     return false;
 if ((vort.getTail().z()>boundaries[2].second && (2.0*vort.getMid()-vort.getTail()).z()>boundaries[2].second) || (( vort.getTail().z()<boundaries[2].first) &&
                                                                                                                  (2.0*vort.getMid()-vort.getTail()).z()<boundaries[2].first))
     return false;

 int numFr = 0;
 int numTrFr = 0;
 Vector3D choosen=frames[numFr]->getTriangle(numTrFr).getCenter();
 Vector3D vortonPart;
 switch (part)
 {
 case Beginning:
     vortonPart=2.0*vort.getMid()-vort.getTail();
     break;
 case Center:
     vortonPart=vort.getMid();
     break;
 case Tail:
     vortonPart=vort.getTail();
     break;
 }
 for (int i=0;i<frames.size();i++)
 {
     bool checking;
     checking = (i==numFr) ? true: false;

     if (frames[i]->inside(choosen,vortonPart,numTrFr,checking))
         return true;
 }
 return false;

}

QPair<int,int> FrameCalculations::intersectsFramesVector(QVector<std::shared_ptr<MultiFrame> > &frames, Vorton vort,VortonsPart part)
{
    for (int i=0;i<frames.size();i++)
    {
        Vector3D ra;
        Vector3D rb;
        switch (part)
        {
        case Center:
            ra=vort.getMid();
            rb=vort.getMid()-vort.getMove();
            break;
        case Beginning:
            rb=vort.getTail();
            ra=2.0*vort.getMid()-vort.getTail();
            break;
        case Tail:
            ra=vort.getTail();
            rb=2.0*vort.getMid()-vort.getTail();
            break;
        }

        int res=frames[i]->intersection(ra,rb);
        if (res!=-1)
            return QPair<int,int>(i,res);
    }
    return QPair<int,int>(-1,-1);
}

void FrameCalculations::universalGetBackTriangleFrames(QVector<Vorton> &vortons, QVector<Vorton> &vortonsOriginal, QVector<std::pair<double, double> > boundaries, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame> > &frames, bool screen, Vector3D translScreen)
{

}

void FrameCalculations::universalGetBackTriangleFrames(QVector<Vorton> &vortons, QVector<Vorton> &vortonsOriginal, QVector<std::pair<double, double> > boundaries, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<std::shared_ptr<MultiFrame> > &symFrames,bool screen, Vector3D translScreen)
{
    bool check;
    QVector<std::shared_ptr<MultiFrame>>copyFr1=copyFrames(symFrames);
    frames.append(copyFr1);
     QTime start=QTime::currentTime();
    for (int i=vortons.size()-1; i>=0;i--)
    {
        check=false;
        bool result=insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Center);
        if (result)
        {
            QPair<int,int> res=intersectsFramesVector(frames,vortons[i],VortonsPart::Center);
            if (res.first!=-1)
            {

                Vector3D tau=-vortons[i].getMove();
                double t=Vector3D::dotProduct(frames[res.first]->getTriangle(res.second).getR0()-vortons[i].getMid(),frames[res.first]->getTriangle(res.second).getNormal())/
                        Vector3D::dotProduct(tau,frames[res.first]->getTriangle(res.second).getNormal());
                Vector3D rtilda=vortons[i].getMid()+t*tau;
                Vector3D l=-vortons[i].getMove().normalized();

                //double d=Vector3D::dotProduct(rtilda-vortons[i].getMid(),vortons[i].getMove().normalized());
                double d=(rtilda-vortons[i].getMid()).length();
                Vector3D n=frames[res.first]->getTriangle(res.second).getNormal();
                vortons[i].translate(2.0*d*l);
                check=true;
                //vortons[i].translate(2.0*Vector3D::dotProduct(l,n)*d*n);
                 counters.gotBackNum++;
                //vortons[i].rotateAroundNormal(n);

            }
            else {
                getBackClosestFrame(vortons[i],boundaries,frames);
                 qDebug()<<"Vorton is inside, but intersection not found!";
            }


//            else {


//                int num=findMaxSolidAngle(vortons[i].getMid(),frames);
//                double dist=(vortons[i].getMid()-frames[num]->getCenter()).length();
//                if (dist<layerHeight)
//                {
//                    vortons[i].rotateAroundNormal(normals[num]);
//                    counters.rotatedNum++;
//                }

//            }

        }

        if (insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Tail) != insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Beginning))
        {
            QPair<int,int> res2=intersectsFramesVector(frames,vortons[i],VortonsPart::Tail);

            if (res2.first!=-1)
            {
                vortons[i].rotateAroundNormal(frames[res2.first]->getTriangle(res2.second).getNormal().normalized());
                counters.rotatedNum++;
                counters.gotBackNum++;
                if (insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Center) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Beginning) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Tail))
                {
                     qDebug()<<"Bad rotation Tail! (getBack rotation) Vorton is still inside!";

                }
            }
            else {
                getBackClosestFrameEdge(vortons[i],boundaries,frames);
                     qDebug()<<"Can't find intersected frame! (getBack rotation)";
            }
        }

        else {
            //QPair<int,int> normNum=findMaxSolidAngle(vortons[i].getMid(),frames);

            QPair<int,int> normNum=findMaxSolidAngle(vortons[i],frames);
            double dist=(vortons[i].getMid()-frames[normNum.first]->getTriangle(normNum.second).getCenter()).length();
            Vorton test=vortons[i];

            if (dist<layerHeight||(insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Center) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Beginning) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Tail)))
            {
                test.rotateAroundNormal(frames[normNum.first]->getTriangle(normNum.second).getNormal().normalized());

            }
            if (!(insideFramesVector(frames,test,boundaries,VortonsPart::Center) || insideFramesVector(frames,test,boundaries,VortonsPart::Beginning) || insideFramesVector(frames,test,boundaries,VortonsPart::Tail)))
            {


                 if (!(vortons[i]==test))
                 {

                    counters.rotatedNum++;
                 vortons[i]=test;
                 }
            }
            if ((insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Center) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Beginning) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Tail)))
            {
                 qDebug()<<"Bad main rotation! Vorton is still inside!";
            }

        }

        if ((insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Center) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Beginning) || insideFramesVector(frames,vortons[i],boundaries,VortonsPart::Tail)))
        {
            qDebug()<<"Vorton is inside after getBack!";
            vortons.remove(i);
            vortonsOriginal.remove(i);
            counters.gotBackDeletedNum++;
            i--;
        }

        if (!check)
            vortons[i].setMove(Vector3D());

        if (screen==true)
        {
            if (FrameCalculations::insideScreen(vortons[i],translScreen))
            {
                vortons.remove(i);
                vortonsOriginal.remove(i);
                counters.underScreenNum++;
            }
        }
    }
    frames.remove(frames.size()-copyFr1.size(),copyFr1.size());
    timers.getBackAndRotateTimer=start.elapsed()*0.001;

}

bool FrameCalculations::universalInsideR0Correct(const Vorton vort, const QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    if ((vort.getTail().x()>boundaries[0].second && (2.0*vort.getMid()-vort.getTail()).x()>boundaries[0].second) || (( vort.getTail().x()<boundaries[0].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).x()<boundaries[0].first))
        return false;
    if ((vort.getTail().y()>boundaries[1].second && (2.0*vort.getMid()-vort.getTail()).y()>boundaries[1].second) || (( vort.getTail().y()<boundaries[1].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).y()<boundaries[1].first))
        return false;
    if ((vort.getTail().z()>boundaries[2].second && (2.0*vort.getMid()-vort.getTail()).z()>boundaries[2].second) || (( vort.getTail().z()<boundaries[2].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).z()<boundaries[2].first))
        return false;

    srand(time(NULL));
    int num=/*rand() % frames.size()*/ 1;
    Vector3D choosen=frames[num]->at(0).getTail()+0.25*(frames[num]->at(1).getTail()-frames[num]->at(0).getTail())
            +0.25*(frames[num]->at(2).getTail()-frames[num]->at(0).getTail());
    for (int i=0; i<frames.size();i++)
    {
            for (int j=1; j<frames[i]->getAnglesNum()-1; j++)
            {

                if (i==num && j==1)
                    continue;
                Vector3D e1=frames[i]->at(j).getTail()-frames[i]->at(0).getTail();
                Vector3D e2=frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail();
                Vector3D n=Vector3D::crossProduct(e1,e2);
                Vector3D tau=(vort.getMid()-choosen);
                double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);


                Vector3D rtilda=choosen+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;


                if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                        coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
                {
                    double dot=Vector3D::dotProduct(choosen-vort.getMid(),rtilda-vort.getMid());
                    if (!((dot)>0||fabs(dot)<1e-10))
                           return true;
                }

            }
    }
    return false;
}

void FrameCalculations::universalGetBackR0Triangle(QVector<Vorton> &vortons, QVector<std::pair<double, double> > boundaries, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame> > &frames, bool screen)
{
    QTime start=QTime::currentTime();
     int sdfsd=0;
    for (int i=0; i<vortons.size();i++)
    {
        if (screen==true)
        {
            if (FrameCalculations::insideScreen(vortons[i]))
            {
                vortons.remove(i);
                counters.underScreenNum++;
            }
        }

        bool result=universalInsideR0Correct(vortons[i],boundaries,frames);

        if (result)
        {
//            Vorton a=vortons[i];
//            a.setMid(a.getMid()-a.getMove());
//            if (universalInsideR0Correct(a,boundaries,frames))
//                qDebug()<<"WTF?";
//            if (i==30)
//                qDebug()<<"stop";
            InsideWithNormal resSeg=universalInsideR0CorrectSegment(vortons[i],frames);

            if (resSeg.res!=-1)
            {
                Vector3D length=-vortons[i].getMove().normalized();
                double distance=(resSeg.rtilda-vortons[i].getMid()).length();
                vortons[i].translate(2.0*distance*length);
                vortons[i].rotateAroundNormal(resSeg.normal);
                counters.gotBackNum++;
                counters.rotatedNum++;

            }

            else {


                QPair<int,int> num=findMaxSolidAngle(vortons[i].getMid(),frames);
                double dist=(vortons[i].getMid()-frames[num.first]->getCenter()).length();
                if (dist<layerHeight)
                {
                    vortons[i].rotateAroundNormal(normals[num.first]);
                    counters.rotatedNum++;
                }

            }

        }
        else {
            QPair<int,int> num=findMaxSolidAngle(vortons[i].getMid(),frames);
            double dist=(vortons[i].getMid()-frames[num.first]->getCenter()).length();
            if (dist<layerHeight)
            {
                vortons[i].rotateAroundNormal(normals[num.first]);
                counters.rotatedNum++;
            }

        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}


InsideWithNormal FrameCalculations::universalInsideR0CorrectSegment(const Vorton vort, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    Vector3D choosen=vort.getMid();



    for (int i=0; i<frames.size();i++)
    {
        for (int j=1; j<frames[i]->getAnglesNum()-1; j++)
        {
            Vector3D e1=frames[i]->at(j).getTail()-frames[i]->at(0).getTail();
            Vector3D e2=frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail();
            Vector3D n=Vector3D::crossProduct(e1,e2);
            Vector3D tau=(vort.getMid()-vort.getMove()-choosen);
            double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);


            Vector3D rtilda=choosen+t*tau;
            Vector3D a1=frames[i]->at(0).getTail()-rtilda;
            Vector3D a2=frames[i]->at(j).getTail()-rtilda;
            Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;


            if (/*coDirectionallyCheck(a1,a2,a3)*/ coDirectionallyCheck(Vector3D::crossProduct(a2,a1),Vector3D::crossProduct(a3,a2)) &&
                    coDirectionallyCheck(Vector3D::crossProduct(a3,a2),Vector3D::crossProduct(a1,a3)))
            {
                if ((t>0.0 && t<1.0) || fabs(t)<1e-10 || fabs(t-1.0)<1e-10)
                {

                    //double dot=Vector3D::dotProduct(choosen-vort.getMid(),rtilda-vort.getMid());
                    //if (!((dot)>0||fabs(dot)<1e-10))
                    //{
                    //    qDebug()<<"naideno";
                    return InsideWithNormal {i,rtilda,n};
                    //}
                }
            }


        }
    }
    return InsideWithNormal {-1,Vector3D(),Vector3D()};
}

Inside FrameCalculations::testInsideCorrect(const Vorton vort, const QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames, int partNum)
{
    Inside ress;
    if ((vort.getTail().x()>boundaries[0].second && (2.0*vort.getMid()-vort.getTail()).x()>boundaries[0].second) || (( vort.getTail().x()<boundaries[0].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).x()<boundaries[0].first))
        return Inside {-1,Vector3D(),0};
    if ((vort.getTail().y()>boundaries[1].second && (2.0*vort.getMid()-vort.getTail()).y()>boundaries[1].second) || (( vort.getTail().y()<boundaries[1].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).y()<boundaries[1].first))
        return Inside {-1,Vector3D(),0};
    if ((vort.getTail().z()>boundaries[2].second && (2.0*vort.getMid()-vort.getTail()).z()>boundaries[2].second) || (( vort.getTail().z()<boundaries[2].first) &&
                                                                                                                     (2.0*vort.getMid()-vort.getTail()).z()<boundaries[2].first))
        return Inside {-1,Vector3D(),0};

    srand(time(NULL));
    int num=/*rand() % frames.size()*/ 1;

    Vector3D choosen;
    if (partNum==0)
        choosen=2.0*vort.getMid()-vort.getTail()-vort.getMove();
    else {
        choosen=vort.getTail()-vort.getMove();
    }
    for (int i=0; i<frames.size();i++)
    {
        if (i!=num)
        {
            if (frames[i]->getAnglesNum()==4)
            {
                Vector3D e1=frames[i]->at(1).getTail()-frames[i]->at(0).getTail();
                Vector3D e2=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                Vector3D n=Vector3D::crossProduct(e1,e2);
                Vector3D tau=(vort.getTail()-choosen);
                double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);


                Vector3D rtilda=choosen+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(1).getTail()-rtilda;
                Vector3D a3=frames[i]->at(2).getTail()-rtilda;


                if (coDirectionallyCheck(a1,a2,a3))
                {
                    double dot=Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail());
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                           Inside {1, rtilda,i};
                }
                tau=2.0*vort.getMid()-vort.getTail()-choosen;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                rtilda=choosen+t*tau;
                a1=frames[i]->at(0).getTail()-rtilda;
                a2=frames[i]->at(1).getTail()-rtilda;
                a3=frames[i]->at(2).getTail()-rtilda;
                if (coDirectionallyCheck(a1,a2,a3))
                {
                    double dot=Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()));
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                            Inside {0, rtilda,i};
                }

                e1=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                e2=frames[i]->at(3).getTail()-frames[i]->at(0).getTail();
                n=Vector3D::crossProduct(e1,e2);
                tau=(vort.getTail()-choosen);
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);

                rtilda=choosen+t*tau;
                a1=frames[i]->at(0).getTail()-rtilda;
                a2=frames[i]->at(2).getTail()-rtilda;
                a3=frames[i]->at(3).getTail()-rtilda;
                if (coDirectionallyCheck(a1,a2,a3))
                {
                    double dot=Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail());
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                            Inside {1, rtilda,i};
                }
                tau=2.0*vort.getMid()-vort.getTail()-choosen;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                rtilda=choosen+t*tau;
                a1=frames[i]->at(0).getTail()-rtilda;
                a2=frames[i]->at(2).getTail()-rtilda;
                a3=frames[i]->at(3).getTail()-rtilda;
                if (coDirectionallyCheck(a1,a2,a3))
                {
                    double dot=Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()));
                    if (!((dot)>0||qFuzzyIsNull(dot)))
                                Inside {0, rtilda,i};
                }


            }
            else {
                for (int j=1;j<frames[i]->getAnglesNum()-1;j++)
                {

                    Vector3D e1=frames[i]->at(j).getTail()-frames[i]->at(0).getTail();
                    Vector3D e2=frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail();
                    Vector3D n=Vector3D::crossProduct(e1,e2);
                    Vector3D tau=vort.getTail()-choosen;
                    double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {
                        double dot=Vector3D::dotProduct(choosen-vort.getTail(),rtilda-vort.getTail());
                        if (!((dot)>0||qFuzzyIsNull(dot)))
                        Inside {1, rtilda,i};
                    }
                    tau=2.0*vort.getMid()-vort.getTail()-choosen;
                    t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    rtilda=choosen+t*tau;
                    a1=frames[i]->at(0).getTail()-rtilda;
                    a2=frames[i]->at(j).getTail()-rtilda;
                    a3=frames[i]->at(j+1).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {
                        double dot=Vector3D::dotProduct(choosen-(2.0*vort.getMid()-vort.getTail()),rtilda-(2.0*vort.getMid()-vort.getTail()));
                        if (!((dot)>0||qFuzzyIsNull(dot)))
                                    Inside {0, rtilda,i};
                    }
                }
            }
        }
    }
    return Inside {-1,Vector3D(), 0};
}

InsideSeg FrameCalculations::universalInsideCorrectSegment(const Vorton vort, const QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame> > &frames, int part)
{

    Vector3D choosen=vort.getTail()-vort.getMove()-vort.getElongation();
    Vector3D choosenBeg=2.0*vort.getMid()-vort.getTail()-vort.getMove()+vort.getElongation();
    Inside res1=Inside {-1,Vector3D(), 0};;
    Inside res0=Inside {-1,Vector3D(), 0};;
    for (int i=0; i<frames.size();i++)
    {
//        if (i!=num)
//        {

            if (frames[i]->getAnglesNum()==4)
            {

                Vector3D e1=frames[i]->at(1).getTail()-frames[i]->at(0).getTail();
                Vector3D e2=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                Vector3D n=Vector3D::crossProduct(e1,e2);
                Vector3D tau=(vort.getTail()-choosen);
                double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);

                if (!(t<0 || t>1))
                {
                qDebug()<<"naideno dlya513";
                Vector3D rtilda=choosen+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(1).getTail()-rtilda;
                Vector3D a3=frames[i]->at(2).getTail()-rtilda;


                if (coDirectionallyCheck(a1,a2,a3))
                {
                        res1= Inside {1,rtilda, i};
                }
                }

                tau=2.0*vort.getMid()-vort.getTail()-choosenBeg;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosenBeg,n)/Vector3D::dotProduct(tau,n);

                if (!(t<0 || t>1))
                {
                    qDebug()<<"naideno dlya513";
                Vector3D rtilda=choosenBeg+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(1).getTail()-rtilda;
                Vector3D a3=frames[i]->at(2).getTail()-rtilda;
                if (coDirectionallyCheck(a1,a2,a3))
                {
                        res0= Inside {0,rtilda, i};
                }
                }


                e1=frames[i]->at(2).getTail()-frames[i]->at(0).getTail();
                e2=frames[i]->at(3).getTail()-frames[i]->at(0).getTail();
                n=Vector3D::crossProduct(e1,e2);
                tau=(vort.getTail()-choosen);
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);

                if (!(t<0 || t>1))
                {
                    qDebug()<<"naideno dlya513";
                Vector3D rtilda=choosen+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(2).getTail()-rtilda;
                Vector3D a3=frames[i]->at(3).getTail()-rtilda;
                if (coDirectionallyCheck(a1,a2,a3))
                {
                         res1= Inside {1,rtilda, i};
                }
                }

                tau=2.0*vort.getMid()-vort.getTail()-choosenBeg;
                t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosenBeg,n)/Vector3D::dotProduct(tau,n);

                if (!(t<0 || t>1))
                {
                    qDebug()<<"naideno dlya513";
                Vector3D rtilda=choosenBeg+t*tau;
                Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                Vector3D a2=frames[i]->at(2).getTail()-rtilda;
                Vector3D a3=frames[i]->at(3).getTail()-rtilda;
                if (coDirectionallyCheck(a1,a2,a3))
                {
                        res0= Inside {0,rtilda, i};
                }

                }
            }
            else {
                for (int j=1;j<frames[i]->getAnglesNum()-1;j++)
                {
                    Vector3D e1=frames[i]->at(j).getTail()-frames[i]->at(0).getTail();
                    Vector3D e2=frames[i]->at(j+1).getTail()-frames[i]->at(0).getTail();
                    Vector3D n=Vector3D::crossProduct(e1,e2);

                    Vector3D tau=vort.getTail()-choosen;
                    double t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosen,n)/Vector3D::dotProduct(tau,n);
                    if (!(t<0 || t>1))
                    {
                    qDebug()<<"naideno dlya513";
                    Vector3D rtilda=choosen+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {

                            res1= Inside {1,rtilda, i};
                    }
                    }

                    tau=2.0*vort.getMid()-vort.getTail()-choosenBeg;
                    t=Vector3D::dotProduct(frames[i]->at(0).getTail()-choosenBeg,n)/Vector3D::dotProduct(tau,n);
                    if (!(t<0 || t>1))
                    {
                        qDebug()<<"naideno dlya513";
                    Vector3D rtilda=choosenBeg+t*tau;
                    Vector3D a1=frames[i]->at(0).getTail()-rtilda;
                    Vector3D a2=frames[i]->at(j).getTail()-rtilda;
                    Vector3D a3=frames[i]->at(j+1).getTail()-rtilda;
                    if (coDirectionallyCheck(a1,a2,a3))
                    {

                            res0= Inside {0,rtilda, i};
                    }
                    }
                    }
            }
        //}
    }
    if (res0.res==-1 && res1.res==-1)
        qDebug()<<"Ne osilil";
    return InsideSeg {QPair<int,int>{res0.res,res1.res}, QPair<Vector3D,Vector3D>(res0.rtilda,res1.rtilda),QPair<int,int>(res0.num,res1.num)};
}


QVector<int> FrameCalculations::universalGetBack(QVector<Vorton> &vortons, QVector<std::pair<double,double>> boundaries, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame>>& frames, bool screen)
{
    QVector<int> results;
    for (int i=0; i<vortons.size();i++)
    {
        if (screen==true)
        {
            if (FrameCalculations::insideScreen(vortons[i]))
            {
                vortons.remove(i);
            }
        }
        int res;
        res=universalInside(vortons[i],boundaries,frames);
        results.push_back(res);
        if (res==1)
        {
            QPair<double,int> closest=BodyFragmentation::findClosestTriangle(vortons[i].getTail(),frames, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            if (universalInside(vortons[i],boundaries,frames)==0)
            {
                QPair<double,int> closest=BodyFragmentation::findClosestTriangle(2.0*vortons[i].getMid()-vortons[i].getTail(),frames, normals);
                vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
                vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
                vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            }

        }
        if (res==0)
        {
            QPair<double,int> closest=BodyFragmentation::findClosestTriangle(2.0*vortons[i].getMid()-vortons[i].getTail(),frames, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            if (universalInside(vortons[i],boundaries,frames)==1)
            {
                QPair<double,int> closest=BodyFragmentation::findClosestTriangle(vortons[i].getTail(),frames, normals);
                vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
                vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
                vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            }
        }
//        if (res==-1)
//        {

//            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
//            QPair<double,int> closestSec=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
//            if (closest.first<layerHeight||closestSec.first<layerHeight)
//                vortons[i].rotateAroundNormal(normals[closest.second]);

//        }
    }

    return results;
}

QVector<int> FrameCalculations::universalGetBackTriangle(QVector<Vorton> &vortons, QVector<std::pair<double, double> > boundaries, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame> > &frames, bool screen)
{
    QTime start=QTime::currentTime();
    QVector<int> results;
    QVector<QPair<int,int>> segRes;
     int k=0;
    for (int i=0; i<vortons.size();i++)
    {
        if (screen==true)
        {
            if (FrameCalculations::insideScreen(vortons[i]))
            {
                vortons.remove(i);
                counters.underScreenNum++;
            }
        }
        Inside res;
        if (i==153)
            qDebug()<<"naideno";
        res=universalInsideCorrect(vortons[i],boundaries,frames);
        results.push_back(res.res);

        if (res.res!=-1)
        {

            InsideSeg resn=universalInsideCorrectSegment(vortons[i],boundaries,frames,res.res);
            if (resn.res.first==-1 && resn.res.second==-1)
            {
                    Inside aa=testInsideCorrect(vortons[i],boundaries,frames,res.res);
            }
            segRes.push_back(QPair<int,int>(resn.res.first,resn.res.second));
            //QPair<double,int> closest=BodyFragmentation::findClosestTriangle(vortons[i].getTail(),frames, normals);
            double d;
            Vector3D l=-vortons[i].getMove().normalized();
            if (resn.res.first!=-1 && resn.res.second!=-1)
            {
                k++;
            d=std::max(fabs(Vector3D::dotProduct(resn.rtilda.second-vortons[i].getMid(),l)),
                    fabs(Vector3D::dotProduct(resn.rtilda.first-vortons[i].getMid(),l)));
            qDebug()<<d;
            }
            else{
            if (resn.res.second!=-1)
            {
                    d=fabs(Vector3D::dotProduct(resn.rtilda.second-vortons[i].getMid(),l));
                    k++;
            }
            else {
                if (resn.res.first!=-1)
                {
                 k++;
                    d=fabs(Vector3D::dotProduct(resn.rtilda.first-vortons[i].getMid(),l));
                }
                else continue;
            }
            }

            if (std::isnan(d)||std::isnan(vortons[i].getMove().normalized().length()))
                qDebug()<<i;
            if (d>0.5)
                qDebug()<<"so"<<i;
            Vector3D futTrans=2.0*l*d;
            vortons[i].translateWithMove(futTrans);

//            //vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
//            //vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
//            //vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
//            Inside resInter=universalInsideCorrect(vortons[i],boundaries,frames);
//            if (resInter.res==0)
//            {
//                Inside resn=universalInsideCorrectSegment(vortons[i],boundaries,frames,resInter.res);
//                double d=fabs(Vector3D::dotProduct(resn.rtilda-(2.0*vortons[i].getMid()-vortons[i].getTail()),normals[resn.num]));
//                if (std::isnan(d)||std::isnan(vortons[i].getMove().normalized().length()))
//                    qDebug()<<i;
//                vortons[i].translateWithMove(-2.0*vortons[i].getMove().normalized()*d+2.0*futTrans);

////                QPair<double,int> closest=BodyFragmentation::findClosestTriangle(2.0*vortons[i].getMid()-vortons[i].getTail(),frames, normals);
////                vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
////                vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
////                vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
//            }
//            else vortons[i].setMove(vortons[i].getMove()+futTrans);
            counters.gotBackNum++;

        }
//        if (res.res==0)
//        {
//            InsideSeg resn=universalInsideCorrectSegment(vortons[i],boundaries,frames,res.res);
//            double d;
//            if (resn.res.first!=-1 && resn.res.second!=-1)
//            d=std::max(fabs(Vector3D::dotProduct(resn.rtilda.second-vortons[i].getTail(),normals[resn.num.second])),
//                    fabs(Vector3D::dotProduct(resn.rtilda.first-2.0*vortons[i].getMid()+vortons[i].getTail(),normals[resn.num.first])));
//            else
//                if (resn.res.first!=-1)
//                    d= fabs(Vector3D::dotProduct(resn.rtilda.first-2.0*vortons[i].getMid()+vortons[i].getTail(),normals[resn.num.first]));
//                else {
//                    if (resn.res.second!=-1)
//                    d=fabs(Vector3D::dotProduct(resn.rtilda.second-vortons[i].getTail(),normals[resn.num.second]));
//                    else continue;
//                }
//            if (std::isnan(d)||std::isnan(vortons[i].getMove().normalized().length()))
//                qDebug()<<i;
//            if (d>0.5)
//                qDebug()<<"so"<<i;
//            Vector3D futTrans=-2.0*vortons[i].getMove().normalized()*d;
//            vortons[i].translateWithMove(futTrans);

//////            QPair<double,int> closest=BodyFragmentation::findClosestTriangle(2.0*vortons[i].getMid()-vortons[i].getTail(),frames, normals);
//////            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
//////            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
//////            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
////            Inside resInter=universalInsideCorrect(vortons[i],boundaries,frames);
////            if (resInter.res==1)
////            {
////                    Inside resn=universalInsideCorrectSegment(vortons[i],boundaries,frames,resInter.res);
////                  double d=fabs(Vector3D::dotProduct(resn.rtilda-vortons[i].getTail(),normals[resn.num]));
////                  if (std::isnan(d)||std::isnan(vortons[i].getMove().normalized().length()))
////                      qDebug()<<i;
////                  vortons[i].translateWithMove(-2.0*vortons[i].getMove().normalized()*d+2.0*futTrans);

//////                QPair<double,int> closest=BodyFragmentation::findClosestTriangle(vortons[i].getTail(),frames, normals);
//////                vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
//////                vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
//////                vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
////            }
////            else vortons[i].setMove(vortons[i].getMove()+futTrans);
//            counters.gotBackNum++;
//        }
    }
    QVector<int> res2;
for (int i=0; i<vortons.size();i++)
{

res2.push_back(universalInside(vortons[i],boundaries,frames));
}
int no_otskok=0;
for (int i=0;i<res2.size();i++)
if (res2[i]!=-1)
    no_otskok++;
qDebug()<<"no_otskok"<<no_otskok;
    timers.getBackAndRotateTimer+=start.elapsed()*0.001;
    return results;
}

void FrameCalculations::correctMove(QVector<Vorton> &freeVortons, QVector<Vorton> &copyVort)
{
    for (int i=0; i<freeVortons.size();i++)
    {
        freeVortons[i].setMove(copyVort[i].getMid()-freeVortons[i].getMid());

    }
}

void FrameCalculations::correctMove(QVector<Vorton> &freeVortons, QVector<Vorton> &copyVort, QVector<Vorton> &newVortons, QVector<Vorton> &copyNewVort)
{
    for (int i=0; i<freeVortons.size();i++)
    {
        freeVortons[i].setMove(copyVort[i].getMid()-freeVortons[i].getMid());

    }
    for (int i=0; i<newVortons.size();i++)
    {
        newVortons[i].setMove(copyNewVort[i].getMid()-newVortons[i].getMid());

    }

}

void FrameCalculations::getBackMove(QVector<Vector3D> &moveFromGetBack, QVector<Vorton> &newVortons, QVector<Vorton> &freeVortons)
{
    moveFromGetBack.clear();
    for (int i=0;i<freeVortons.size();i++)
        moveFromGetBack.push_back(freeVortons[i].getMove());
    for (int i=0;i<newVortons.size();i++)
        moveFromGetBack.push_back(newVortons[i].getMove());

}

void FrameCalculations::universalRotate(QVector<Vorton> vortons, QVector<int> res, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    for (int i=0; i<res.size();i++)
    {
        if (res[i]==-1)
        {
                    QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
                    QPair<double,int> closestSec=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
                    if (closest.first<layerHeight||closestSec.first<layerHeight)
                        vortons[i].rotateAroundNormal(normals[closest.second]);
        }
    }
}

void FrameCalculations::universalRotateTriangle(QVector<Vorton> &vortons, QVector<std::pair<double, double> > boundaries, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame>>& frames)
{

    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size();i++)
    {
        if (vortons[i].getMid().x()>(boundaries[0].second+layerHeight )
                || vortons[i].getMid().x()<(boundaries[0].first-layerHeight))
            continue;
        if (vortons[i].getMid().y()>(boundaries[1].second+layerHeight )
                || vortons[i].getMid().y()<(boundaries[1].first-layerHeight))
            continue;
        if (vortons[i].getMid().z()>(boundaries[2].second+layerHeight )
                || vortons[i].getMid().z()<(boundaries[2].first-layerHeight))
            continue;
        //if (res[i]==-1)
        //{
//                    QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
//                    QPair<double,int> closestSec=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
//                    if (closest.first<layerHeight||closestSec.first<layerHeight)
//                    {
//                        int num=findMaxSolidAngle(vortons[i].getMid(),frames);
//                        vortons[i].rotateAroundNormal(normals[num]);

//                    }
                    QPair<int,int> num=findMaxSolidAngle(vortons[i].getMid(),frames);
                    double dist=(vortons[i].getMid()-frames[num.first]->getCenter()).length();
                    if (dist<layerHeight)
                    {
                        vortons[i].rotateAroundNormal(normals[num.first]);
                        qDebug()<<i<<"razvernut";
                    }
                    counters.rotatedNum++;
        //}
    }
    timers.rotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::unionWithLift(QVector<Vorton> &vortons, double eStar, double eDoubleStar, double deltaUp, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &normals, QVector<std::pair<double, double> > boundaries)
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
                    int normFirstNum=vortonAffiliation(vortons[i],frames);
                    int normSecondNum=vortonAffiliation(vortons[j],frames);
                    Vector3D normal=0.5*(normals[normFirstNum]+normals[normSecondNum]);

                    Vorton temp=vortons[j];
                    if (psi<0)
                        temp.turn();
                    Vector3D newMid = (fabs(vortons[i].getVorticity())*vortons[i].getMid()+fabs(temp.getVorticity())*temp.getMid())/
                    (fabs(vortons[i].getVorticity())+fabs(temp.getVorticity()));
                    Vector3D selfLen = (fabs(vortons[i].getVorticity())*(vortons[i].getTail()-vortons[i].getMid())+fabs(temp.getVorticity())*(temp.getTail()-temp.getMid()))/
                    (fabs(vortons[i].getVorticity())+fabs(temp.getVorticity()));
                    double newVorticity=vortons[i].getVorticity()+temp.getVorticity();
                    Vorton newVorton = Vorton (newMid, newMid+selfLen, newVorticity, vortons[i].getRadius());
                    newVorton.translate(normal*deltaUp);
                    if (!(insideFramesVector(frames,newVorton,boundaries,VortonsPart::Center)||insideFramesVector(frames,newVorton,boundaries,VortonsPart::Beginning)||insideFramesVector(frames,newVorton,boundaries,VortonsPart::Tail)))
                    {
                    counters.unitedNum++;
                    vortons[i]=newVorton;
                    i--;
                    vortons.remove(j);
                    //break;
                    }

                }

            }
        }
    }
    timers.unionTimer=start.elapsed()*0.001;
}

void FrameCalculations::unionVortons(QVector<Vorton> &vortons, const double eStar, const double eDoubleStar, const double vortonRad, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<std::pair<double, double> > boundaries)
{
    QTime start = QTime::currentTime();
    int insideN=0;
    for (int i=vortons.size()-1; i>=0; i--)
    {
        for (int j=i-1; j>=0; j--)
        {
            if((vortons[i].getMid()-vortons[j].getMid()).length()<eStar)
            {
                double psi=Vector3D::dotProduct((vortons[i].getTail()-vortons[i].getMid()).normalized(),(vortons[j].getTail()-vortons[j].getMid()).normalized());
                if (fabs(psi)>eDoubleStar)
                {
                    Vorton temp=vortons[j];
                    if (psi<0)
                    {
                        temp.turn();
                        insideN++;
                    }
                    Vector3D newMid = (fabs(vortons[i].getVorticity())*vortons[i].getMid()+fabs(temp.getVorticity())*temp.getMid())/
                    (fabs(vortons[i].getVorticity())+fabs(temp.getVorticity()));
                    Vector3D selfLen = (fabs(vortons[i].getVorticity())*(vortons[i].getTail()-vortons[i].getMid())+fabs(temp.getVorticity())*(temp.getTail()-temp.getMid()))/
                    (fabs(vortons[i].getVorticity())+fabs(temp.getVorticity()));
                    double newVorticity=vortons[i].getVorticity()+temp.getVorticity();
                    Vorton newVorton = Vorton (newMid, newMid+selfLen, newVorticity, vortonRad);
                    if (!(insideFramesVector(frames,newVorton,boundaries,VortonsPart::Center)||insideFramesVector(frames,newVorton,boundaries,VortonsPart::Beginning)||insideFramesVector(frames,newVorton,boundaries,VortonsPart::Tail)))
                    {

                    counters.unitedNum++;
                    vortons[i]=newVorton;
                    i--;
                    vortons.remove(j);
                    //break;
                    }

                }

            }
        }
    }


    timers.unionTimer=start.elapsed()*0.001;
}

void FrameCalculations::unionWithRestrictions(QVector<Vorton> &vortons, const double eStar, const double eDoubleStar, const double vortonRad, QVector<std::shared_ptr<MultiFrame> > &frames, const double maxInitialGamma, int &quant, QVector<std::pair<double, double> > boundaries)
{
    QTime start = QTime::currentTime();

    quant=0;
    for (int i=vortons.size()-1; i>=0; i--)
    {
        for (int j=i-1; j>=0; j--)
        {
            if((vortons[i].getMid()-vortons[j].getMid()).length()<eStar)
            {
                double psi=Vector3D::dotProduct((vortons[i].getTail()-vortons[i].getMid()).normalized(),(vortons[j].getTail()-vortons[j].getMid()).normalized());
                if (fabs(psi)>eDoubleStar)
                {
                    Vorton temp=vortons[j];
                    if (psi<0)
                        temp.turn();

                    Vector3D newMid = (fabs(vortons[i].getVorticity())*vortons[i].getMid()+fabs(temp.getVorticity())*temp.getMid())/
                    (fabs(vortons[i].getVorticity())+fabs(temp.getVorticity()));
                    Vector3D selfLen = (fabs(vortons[i].getVorticity())*(vortons[i].getTail()-vortons[i].getMid())+fabs(temp.getVorticity())*(temp.getTail()-temp.getMid()))/
                    (fabs(vortons[i].getVorticity())+fabs(temp.getVorticity()));
                    double newVorticity=vortons[i].getVorticity()+temp.getVorticity();
                    if (fabs(newVorticity)>3.0*fabs(maxInitialGamma))
                    {
                        quant++;
                        continue;
                    }
                    Vorton newVorton = Vorton (newMid, newMid+selfLen, newVorticity, vortonRad);
                    if (!(insideFramesVector(frames,newVorton,boundaries,VortonsPart::Center)||insideFramesVector(frames,newVorton,boundaries,VortonsPart::Beginning)||insideFramesVector(frames,newVorton,boundaries,VortonsPart::Tail)))
                    {
                    counters.unitedNum++;
                    vortons[i]=newVorton;
                    i--;
                    vortons.remove(j);
                    //break;
                    }

                }

            }
        }
    }
    timers.unionTimer=start.elapsed()*0.001;
}

/*!
Функция объединения близлежащих вортонов
\param[in,out] vortons Вектор, содержащий вортоны для объединения
\param[in] eStar Максимальное расстояние для объединения
\param[in] eDoubleStar Минимальный косинус угла для объединения
\param[in] vortonRad Радиус вортон-отрезков
*/
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
                    //if ()
                    vortons[i]=newVorton;
                    i--;
                    vortons.remove(j);
                    break;
                }

            }
        }
    }
    timers.unionTimer=start.elapsed()*0.001;
}

int FrameCalculations::vortonAffiliation(Vorton &v, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    for (int i=0; i<frames.size();i++)
    {
        for (int j=0; j<frames[i]->getAnglesNum();j++)
        {
            if (v==frames[i]->at(j))
            {
                return i;
            };
        }
    }
}

/*!
Функция удаления вортонов с малой завихренностью
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] minVorticity Минимальное значение завихренности для объединения
*/
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
    timers.removeVorticityTimer=start.elapsed()*0.001;
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра сферы
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата центра сферы
*/
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
    timers.farTimer=start.elapsed()*0.001;
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра цилиндра
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата высота цилиндра
*/
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
    timers.farTimer=start.elapsed()*0.001;
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра тела вращения
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата центра тела вращения
*/
void FrameCalculations::removeFarRotationBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter)
{
    removeFarSphere(vortons,farDistance,bodyCenter);
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра тела вращения со срезом дна
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата центра тела вращения со срезом дна
*/
void FrameCalculations::removeFarRotationCutBody(QVector<Vorton> &vortons, const double farDistance, const Vector3D bodyCenter)
{
    removeFarSphere(vortons,farDistance,bodyCenter);
}

/*!
Функция расчета перемещений и удалений для вортонов с рамок и вортонов в потоке
\param[in,out] freeVortons Вектор, содержащий вортоны в потоке
\param[in,out] newVortons Вектор, содержащий вортоны с рамок
\param[in] step Размер шага
\param[in] streamVel Скорость потока
\param[in] eDelta Максимальное значение удлинения
\param[in] fiMax Максимальное значение угла поворота
\param[in] maxMove Максимальное перемещения
*/
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
    timers.integrationTimer=start.elapsed()*0.001;
}

void FrameCalculations::displacementPassiveCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, QVector<Vorton> frameVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove)
{
    QTime start = QTime::currentTime();
    QVector<Vorton> vortons;
    vortons.append(freeVortons);
    vortons.append(frameVortons);
    QVector<ParallelPassive> paralVec;
    for (int i=0; i<freeVortons.size(); i++)
    {
        ParallelPassive element {&vortons, freeVortons[i], streamVel, step};
        paralVec.push_back(element);
    }
    for (int i=0; i<newVortons.size();i++)
    {
        ParallelPassive element {&vortons, newVortons[i], streamVel, step};
        paralVec.push_back(element);
    }
    QVector<Vorton> resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacementPassive, addToVortonsVec, QtConcurrent::OrderedReduce);

    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=fabs(selfLenAft.length()-selfLenBef.length());
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
        if (std::isnan(resultedVec[i].getMove().length()))
            qDebug()<<"F*cking nan";
        freeVortons[i].setMove(resultedVec[i].getMove());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        if (std::isnan(resultedVec[i+freeVortons.size()].getMove().length()))
            qDebug()<<"F*cking nan";
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.001;
}

void FrameCalculations::displacementPassiveCalcDividedMoves(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, QVector<Vorton> frameVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove,QVector<Vector3D>& moveFromFrames, QVector<Vector3D>& moveFromVortons)
{
    QTime start = QTime::currentTime();
    QVector<Vorton> vortons;
    QVector<Vorton> resultedVec;
    QVector<Vector3D> elongationFromFrames;
    moveFromFrames.clear();
    moveFromVortons.clear();

    vortons.append(frameVortons);
    QVector<ParallelPassive> paralVec;
    for (int i=0; i<freeVortons.size(); i++)
    {
        ParallelPassive element {&vortons, freeVortons[i], Vector3D(), step};
        paralVec.push_back(element);
    }
    for (int i=0; i<newVortons.size();i++)
    {
        ParallelPassive element {&vortons, newVortons[i], Vector3D(), step};
        paralVec.push_back(element);
    }
    resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacementPassive, addToVortonsVec, QtConcurrent::OrderedReduce);
    for (int i=0; i<resultedVec.size();i++)
    {
        moveFromFrames.push_back(resultedVec[i].getMove());
        elongationFromFrames.push_back(resultedVec[i].getElongation());
    }
    resultedVec.clear();

    vortons.clear();
    vortons.append(freeVortons);
    paralVec.clear();
    for (int i=0; i<freeVortons.size(); i++)
    {
        ParallelPassive element {&vortons, freeVortons[i], Vector3D(), step};
        paralVec.push_back(element);
    }
    for (int i=0; i<newVortons.size();i++)
    {
        ParallelPassive element {&vortons, newVortons[i], Vector3D(), step};
        paralVec.push_back(element);
    }
    resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacementPassive, addToVortonsVec, QtConcurrent::OrderedReduce);
    for (int i=0; i<resultedVec.size();i++)
    {
        moveFromVortons.push_back(resultedVec[i].getMove());
        resultedVec[i].setMove(moveFromFrames[i]+moveFromVortons[i]+streamVel*step);
        resultedVec[i].setElongation(resultedVec[i].getElongation()+elongationFromFrames[i]);
    }



    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=fabs(selfLenAft.length()-selfLenBef.length());
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
        if (std::isnan(resultedVec[i].getMove().length()))
            qDebug()<<"F*cking nan";
        freeVortons[i].setMove(resultedVec[i].getMove());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        if (std::isnan(resultedVec[i+freeVortons.size()].getMove().length()))
            qDebug()<<"F*cking nan";
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.001;
}

void FrameCalculations::displacementCalcGauss3(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove, double dlMax, double dlMin)
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
    QVector<Vorton> resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacementGauss, addToVortonsVec, QtConcurrent::OrderedReduce);

    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        //const double dlmax=0.5*2.0*M_PI/36.0*2.0;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=(resultedVec[i].getTail()-resultedVec[i].getMid()+resultedVec[i].getElongation()).length();
        if (turnAngle>fiMax)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.turnRestr++;
        }
//        if ((lengthChange)>dlMax)
//        {
//            resultedVec[i].setElongation((resultedVec[i].getTail()-resultedVec[i].getMid())*(dlMax/(lengthChange)-1));
//            resultedVec[i].setTail(resultedVec[i].getMid()+(resultedVec[i].getTail()-resultedVec[i].getMid()+resultedVec[i].getElongation()))
//            restrictions.elongationRestr++;
//        }
//        else if ((lengthChange)<dlMin)
//        {
//            resultedVec[i].setElongation((resultedVec[i].getTail()-resultedVec[i].getMid())*(dlMin/(lengthChange)-1));
//            resultedVec[i].setTail(resultedVec[i].getMid());
//            restrictions.elongationRestr++;
//        }


        if (resultedVec[i].getElongation().length()>eDelta)
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
        freeVortons[i].setTail(resultedVec[i].getTail());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setTail(resultedVec[i+freeVortons.size()].getTail());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.001;
}

/*!
Функция расчета перемещений и удалений для вортонов с рамок и вортонов в потоке при решении задачи старта
\param[in,out] freeVortons Вектор, содержащий вортоны в потоке
\param[in,out] newVortons Вектор, содержащий вортоны с рамок
\param[in,out] symFreeVortons Симметричный вектор вектору, содержащему вортоны в потоке
\param[in,out] symNewVortons Симметричный вектор вектору, содержащему вортоны с рамок
\param[in] step Размер шага
\param[in] streamVel Скорость потока
\param[in] eDelta Максимальное значение удлинения
\param[in] fiMax Максимальное значение угла поворота
\param[in] maxMove Максимальное перемещения
*/
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
    timers.integrationTimer=start.elapsed()*0.001;
}

/*!
Функция установки размера матрицы
\param size Размер матрциы
*/
void FrameCalculations::setMatrixSize(int size)
{
    matrixSize=size;
}

void FrameCalculations::getBackAndRotate(bool getBackGA,QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame>> &frames, FormingParameters forming)
{
    if (!getBackGA)
        //getBackAndRotateRotationBody(vortons, bodyNose, xEnd,layerHeight,controlPoints, normals, forming);
        getBackAndRotateRotationBodyv2(vortons, bodyNose, xEnd,layerHeight,controlPoints, normals, forming);
    else
        getBackAndRotateRotationBodyGA(vortons, oldvortons,frames, xEnd,bodyNose,forming,layerHeight,controlPoints, normals);
}

void FrameCalculations::getBackAndRotate(bool getBackGA, QVector<Vorton> &vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    if (!getBackGA)
        getBackAndRotateSphere(vortons,center,radius,layerHeight,controlPoints,normals);
    else
        getBackAndRotateSphereGA(vortons,center,radius,layerHeight,controlPoints,normals,oldvortons,frames);
}

void FrameCalculations::getBackAndRotate(bool getBackGA, QVector<Vorton> &vortons, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParametersRBC forming)
{
    if (!getBackGA)
        getBackAndRotateRotationCutBody(vortons, xEnd,layerHeight,controlPoints, normals, bodyNose, forming);
    else
        getBackAndRotateRotationCutBodyGA(vortons, oldvortons,frames, xEnd,layerHeight,controlPoints, normals,bodyNose,forming);
}

IntegrationResults FrameCalculations::integrateParameters(double length, double bodyDensity, FormingParameters parameters)
{
    SimpsonIntegration integration(length,bodyDensity,parameters);
    return integration.integralsCalculator();
}

IntegrationResults FrameCalculations::integrateParameters(double length, double bodyDensity, FormingParametersRBC parameters)
{
    SimpsonIntegration integration(length,bodyDensity,parameters);
    return integration.integralsCalculator();
}

void FrameCalculations::calcSection(double diameter, double xCenter, int fiFragNum, QVector<std::shared_ptr<MultiFrame>>& sectionFrames, QVector<Vector3D>& centerFrames, QVector<Vector3D>& sectionNormals)
{
    double rad0=diameter*0.5/fiFragNum;
    double fi0 = 2*M_PI/fiFragNum;
    Vector3D r0 = Vector3D(xCenter, 0.0, 0.0);
    Vector3D r11 = Vector3D(xCenter, rad0, 0.0);
    Vector3D r21 = Vector3D(xCenter, rad0*cos(fi0),rad0*sin(fi0));
    sectionFrames.push_back(std::make_shared<MultiFrame>(fiFragNum,r0,r11,r21,0.1));
    centerFrames.push_back(Vector3D(xCenter,0.0,0));
    sectionNormals.push_back(Vector3D(1.0,0.0,0.0));
    for (int i=1; i<fiFragNum; i++)
    {
        for (int j=0; j<fiFragNum; j++)
        {
            double fi = fi0*j;
            double r = rad0*i;
            Vector3D r11 (xCenter,r*cos(fi),r*sin(fi));
            Vector3D r01 (xCenter,r*cos(fi+fi0),r*sin(fi+fi0));
            Vector3D r31 (xCenter,(r+rad0)*cos(fi+fi0),(r+rad0)*sin(fi+fi0));
            Vector3D r21 (xCenter,(r+rad0)*cos(fi),(r+rad0)*sin(fi));
            sectionFrames.push_back(std::make_shared <FourFrame>(r01,r11,r21,r31,0.1));
            centerFrames.push_back(Vector3D(xCenter,(r+0.5*rad0)*cos(fi+0.5*fi0),(r+rad0*0.5)*sin(fi+fi0*0.5)));
            sectionNormals.push_back(Vector3D(1.0,0.0,0.0));
        }
    }
}

FramesSizes FrameCalculations::calcFrameSizes(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<double> frameSizes;
    for (int i=0; i<frames.size(); i++)
        frameSizes.push_back(frames[i]->length());
    FramesSizes sizes;
    sizes.maxFrameSize=*std::max_element(frameSizes.begin(),frameSizes.end());
    sizes.minFrameSize=*std::min_element(frameSizes.begin(),frameSizes.end());
    double aver=0.0;
    for (int i=0; i<frameSizes.size();i++)
        aver+=frameSizes[i]/frameSizes.size();
    sizes.averFrameSize=aver;
    return sizes;
}

/*!
Функция установки завихренностей рамкам
\param[in,out] frames Вектор, содержащий рамки
\param[in] vorticities Вектор, содержащий завихренности для рамок
*/
void FrameCalculations::setVorticity(QVector<std::shared_ptr<MultiFrame> > frames, const Eigen::VectorXd vorticities)
{
    for (int i=0; i<vorticities.size()-1; i++)
        frames[i]->setVorticity(vorticities(i));
}

/*!
Функция, возвращающая вектор вортонов с рамок
\param frames Вектор, содержащий рамки
\return Вектор, содержащий вортоны с рамок
*/
QVector<Vorton> FrameCalculations::getFrameVortons(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<Vorton> vortons;
    for (int i=0; i<frames.size(); i++)
        vortons.append(frames[i]->getVortons());
    return vortons;
}

/*!
Функция, возвращающая вектор поднятых вортонов с рамок по нормали
\param frames Вектор, содержащий рамки
\param normals Вектор, содержащий нормали
\param deltaUp Величина подъема вортонов по нормали
\return Вектор, содержащий вортоны с рамок
*/
QVector<Vorton> FrameCalculations::getLiftedFrameVortons(QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vector3D> &normals, const double deltaUp)
{
    QVector<Vorton> vortons;
    for (int i=0; i<frames.size(); i++)
        vortons.append(frames[i]->getLiftedVortons(deltaUp*normals[i]));
    return vortons;
}

/*!
Функция расчета давления в точке
\param point Точка для расчета давления в точке
\param streamVel Скорость потока
\param streamPres Давление потока
\param density Плотность среды
\param frames Вектор, содержащий рамки
\param freeVortons Вектор, содержащий вортоны в потоке
\param tau Величина шага
\return Значение давления
*/
double FrameCalculations::pressureCalc(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density, QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau)
{
    double velAdd=0.5*(Vector3D::dotProduct(streamVel, streamVel)-pow((FrameCalculations::velocity(point, streamVel, freeVortons, frames).length()),2));
    double scal = 0.0;
    double framesAdd = 0.0;
    for (int i=0; i<freeVortons.size(); i++)
        scal+=Vector3D::dotProduct(freeVortons[i].getMove(), freeVortons[i].velocity(point));
    for (int i=0; i<frames.size(); i++)
    {
        framesAdd+=frames[i]->getVorticity()*frames[i]->fi(point);
    }
    return streamPres+density*(velAdd+scal/tau-framesAdd/tau);
}

double FrameCalculations::pressureSetuha(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density, QVector<std::shared_ptr<MultiFrame> > frames,QVector<double> oldGammas, QVector<Vorton> freeVortons, double tau)
{
    double velAdd=0.5*(Vector3D::dotProduct(streamVel, streamVel)-pow((FrameCalculations::velocity(point, streamVel, freeVortons, frames).length()),2));
    double scal = 0.0;
    double framesAdd = 0.0;
    for (int i=0; i<freeVortons.size(); i++)
        scal+=Vector3D::dotProduct(freeVortons[i].getMove(), freeVortons[i].velocity(point));
    for (int i=0; i<frames.size(); i++)
    {
        framesAdd+=(frames[i]->getVorticity()-oldGammas[i])*frames[i]->fi(point);
    }
    return streamPres+density*(velAdd-scal/tau-framesAdd/tau);
}

/*!
Функция расчета силы, действующей на тело
\param streamVel Скорость потока
\param streamPres Давление потока
\param density Плотность среды
\param frames Вектор, содержащий рамки
\param freeVortons Вектор, содержащий вортоны в потоке
\param tau Величина шага
\param squares Вектор, содержащий площади рамок
\param controlPointsRaised Вектор, содержащий контрольные точки для вычисления давлений
\param normals Вектор, содержащий нормали
\return Значение силы
*/
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

Vector3D FrameCalculations::forceSetuha(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vorton> &freeVortons, const double tau, const QVector<double> &squares, const QVector<Vector3D> &controlPointsRaised, const QVector<Vector3D> &normals, QVector<double> &gammas)
{
    QTime start = QTime::currentTime();
    Vector3D force;
    for (int i=0; i<controlPointsRaised.size(); i++)
    {
        double pressure=FrameCalculations::pressureSetuha(controlPointsRaised[i], streamVel, streamPres, density, frames, gammas,freeVortons, tau);
        force-=pressure*squares[i]*normals[i];
    }
    timers.forceTimer=start.elapsed()*0.001;
    return force;
}

void FrameCalculations::forceAndTorqueCalc(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vorton> &freeVortons, const double tau, const QVector<double> &squares, const QVector<Vector3D> &controlPointsRaised, const QVector<Vector3D> &normals, Vector3D center,Vector3D &force, Vector3D &tongue)
{
    QTime start = QTime::currentTime();
    force=Vector3D();
    tongue=Vector3D();
    for (int i=0; i<controlPointsRaised.size(); i++)
    {
        double pressure=FrameCalculations::pressureCalc(controlPointsRaised[i], streamVel, streamPres, density, frames, freeVortons, tau);
        force-=pressure*squares[i]*normals[i];
        tongue-=pressure*squares[i]*Vector3D::crossProduct(controlPointsRaised[i]-center,normals[i]);
    }
    timers.forceTimer=start.elapsed()*0.001;
}

void FrameCalculations::calculateCp(BodyType type,const int stepNum, int stepsQuant,QVector<QVector<double>>& cpArray,   const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vorton>& freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised, FragmentationParameters fragPar)
{
    switch(type)
    {
    case ROTATIONBODY:
    {
        cpRotationBodyDegree(stepNum,stepsQuant,cpArray[0],fragPar.rotationBodyFiFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,0);
        cpRotationBodyDegree(stepNum,stepsQuant,cpArray[1],fragPar.rotationBodyFiFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,90);
        cpRotationBodyDegree(stepNum,stepsQuant,cpArray[2],fragPar.rotationBodyFiFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,180);
        cpRotationBodyDegree(stepNum,stepsQuant,cpArray[3],fragPar.rotationBodyFiFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,270);
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        cpRotationCutBodyDegree(stepNum,stepsQuant,cpArray[0],fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum ,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,0);
        cpRotationCutBodyDegree(stepNum,stepsQuant,cpArray[1],fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,90);
        cpRotationCutBodyDegree(stepNum,stepsQuant,cpArray[2],fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,180);
        cpRotationCutBodyDegree(stepNum,stepsQuant,cpArray[3],fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,270);
        break;
    }
    case ROTATIONTWOBOTTOM:
    {
        cpRotationTwoBottomBody(stepNum,stepsQuant,cpArray[0],fragPar.rotationBodyFiFragNum,fragPar.rotationBodyPartFragNum,streamVel,streamPres,density,frames,freeVortons,tau,controlPointsRaised,0);
        break;
    }
    }

}

QVector<double> FrameCalculations::cpAverage(BodyType type, QVector<Vector3D> &controlPoints, QVector<QVector<double> > &cpArray, const int stepsNum, FragmentationParameters fragPar)
{
    switch (type) {
    case ROTATIONBODY:
    {

        cpAverage(cpArray[0],stepsNum);
        cpAverage(cpArray[1],stepsNum);
        cpAverage(cpArray[2],stepsNum);
        cpAverage(cpArray[3],stepsNum);
        QVector<double> fis;
        fis.push_back(controlPoints[controlPoints.size()-2].x());
        for (int i=0; i<controlPoints.size()-2;i+=fragPar.rotationBodyFiFragNum)
            fis.push_back(controlPoints[i].x());
        fis.push_back(controlPoints[controlPoints.size()-1].x());
        return fis;
    }
    case ROTATIONBOTTOMCUT:
    {

        cpAverage(cpArray[0],stepsNum);
        cpAverage(cpArray[1],stepsNum);
        cpAverage(cpArray[2],stepsNum);
        cpAverage(cpArray[3],stepsNum);
        QVector<double> fis;
        fis.push_back(controlPoints[controlPoints.size()-(fragPar.rotationBodyRFragNum-1)*fragPar.rotationBodyFiFragNum-2].x());
        for (int i=0; i<controlPoints.size()-(fragPar.rotationBodyRFragNum-1)*fragPar.rotationBodyFiFragNum-2;i+=fragPar.rotationBodyFiFragNum)
            fis.push_back(controlPoints[i].x());
        fis.push_back(controlPoints[controlPoints.size()-1].x());
        return fis;
    }
    }


}
/*!
Функция суммирования соответствующих значений ср
\param[in] stepNum Номер текущего шага
\param[in,out] cp Вектор для записи значений ср
\param[in] fiFragNum Количество разбиений по фи
\param[in] radius Радиус сферы
\param[in] pointsRaising Величина подъема точек для вычисления давления
\param[in] tetas Вектор значений углов тета
\param[in] streamVel Скорость потока
\param[in] streamPres Давление потока
\param[in] density Плотность среды
\param[in] frames Вектор, содержащий рамки
\param[in] freeVortons Вектор, содержащий вортоны в потоке
\param[in] tau Величина шага
\param[in] center Координата центра сферы
*/
void FrameCalculations::cpSum(const int stepNum, const int stepsQuant, QVector<double> &cp, const int fiFragNum, const double radius, const double pointsRaising, const QVector<double> &tetas, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const Vector3D center)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
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

void FrameCalculations::cpSumCylinder(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum, const double diameter, const double height, const double pointsRaising, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double fi0=M_PI*2.0/fiFragNum;
        for (int i=0; i<fiFragNum;i++)
        {
            double fi=fi0*(i+0.5);
            Vector3D point((diameter/2.0+pointsRaising)*cos(fi), height*0.5, (diameter/2.0+pointsRaising)*sin(fi));
            double pres=pressureCalc(/*controlpoints[i]*/point, streamVel,streamPres,density,frames,freeVortons,tau);
            cp[i]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        }
    }
}

void FrameCalculations::cpSumRotationBody(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum,  const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vorton>& freeVortons, double tau, const QVector<Vector3D>& controlPointsRaised)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-2],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[0]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        int j=1;
        for (int i=0; i<controlPointsRaised.size()-2; i=i+fiFragNum)
        {
            pres=pressureCalc(controlPointsRaised[i],streamVel,streamPres,density,frames,freeVortons,tau);
            cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
            j++;
        }
        pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-1],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
    }
}

void FrameCalculations::cpRotationBodyDegree(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised, int degree)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-2],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[0]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        int j=1;
        int iStart=static_cast<int>(static_cast<double>(degree)/360.0*fiFragNum);
        for (int i=iStart; i<controlPointsRaised.size()-2; i=i+fiFragNum)
        {
            double pres=pressureCalc(controlPointsRaised[i],streamVel,streamPres,density,frames,freeVortons,tau);
            cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
            j++;
        }
        pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-1],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
    }
}

void FrameCalculations::cpSumRotationCutBody(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum, const int rFragNum, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-2],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[0]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        int j=1;
        for (int i=0; i<controlPointsRaised.size()-rFragNum*fiFragNum-1; i=i+fiFragNum)
        {
            pres=pressureCalc(controlPointsRaised[i],streamVel,streamPres,density,frames,freeVortons,tau);
            cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
            j++;
        }
        pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-1],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
    }
}

void FrameCalculations::cpRotationCutBodyDegree(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum, const int rFragNum, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised, int degree)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-(rFragNum-1)*fiFragNum-2],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[0]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        int j=1;
        int iStart=static_cast<int>(static_cast<double>(degree)/360.0*fiFragNum);
        for (int i=iStart; i<controlPointsRaised.size()-(rFragNum-1)*fiFragNum-2; i=i+fiFragNum)
        {
            double pres=pressureCalc(controlPointsRaised[i],streamVel,streamPres,density,frames,freeVortons,tau);
            cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
            j++;
        }
        pres=pressureCalc(controlPointsRaised[controlPointsRaised.size()-(rFragNum-1)*fiFragNum-1],streamVel,streamPres,density,frames,freeVortons,tau);
        cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
    }
}

void FrameCalculations::cpRotationTwoBottomBody(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum, const int partFragNum,const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised, int degree)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        for (int i=0; i<fiFragNum;i++)
        {
            double pres=pressureCalc(controlPointsRaised[fiFragNum*partFragNum/2+i],streamVel,streamPres,density,frames,freeVortons,tau);
            cp[i]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        }

    }
}

/*!
Функция расчета средних значений ср
\param[in,out] cp Вектор значений ср
\param[in] stepsNum Количество шагов расчета
*/
void FrameCalculations::cpAverage(QVector<double> &cp, const int stepsNum)
{
    int divider;
    if (stepsNum>=200)
        divider=200;
    else
        divider=stepsNum;
    for (int i=0; i<cp.size(); i++)
        cp[i]/=divider;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности сферы в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] center Координата центра сферы
\param[in] radius Величина радиуса сферы
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateSphere(QVector<Vorton> &vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D>& controlPoints, const QVector<Vector3D>& normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideSphere(vortons[i],center,radius))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
//            Vector3D axis=Vector3D(vortons[i].getTail()-center).normalized();
//            vortons[i].translate(2*axis*(axis*radius-vortons[i].getMid()).length());
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
    qDebug()<<"getBack";
}

void FrameCalculations::getBackAndRotateSphereGA(QVector<Vorton> &vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideSphere(vortons[i],center,radius))
        {
            bool founded=false;
            Vector3D tau=oldvortons[i].getMid()-vortons[i].getMid();
            for (int i=0;i<frames.size();i++)
            {

                if (frames[i]->getAnglesNum()==4)
                {
                    int j=0;
                    while (j<3)
                    {
                        Vector3D r1=frames[i]->at(j).getTail();
                        Vector3D r2=frames[i]->at(j+1).getTail();
                        Vector3D r3;
                        if (j+2==4)
                            r3=frames[i]->at(0).getTail();
                        else
                            r3=frames[i]->at(j+2).getTail();
                        Vector3D e1=r2-r1;
                        Vector3D e2=r3-r1;
                        double mixedProd=Vector3D::mixedProduct(tau,e1,e2);
                        if (fabs(mixedProd)<0.0000000001)
                        {
                            double t=Vector3D::mixedProduct(r1-vortons[i].getMid(),e1,e2)/mixedProd;
                            if (t>0.0 && t<1.0)
                            {
                                Vector3D rtilda=vortons[i].getMid()+t*tau;
                                Vector3D a=r1-rtilda;
                                Vector3D b=r2-rtilda;
                                Vector3D c=r3-rtilda;


                                if (coDirectionallyCheck(a,b,c))
                                {
                                    vortons[i].setMid(vortons[i].getMid()+2*t*tau);
                                    vortons[i].setTail(vortons[i].getTail()+2*t*tau);
                                    founded=true;
                                    break;
                                }
                            }
                        }
                        j+=2;
                    }
                }
                else {
                    int j=0;
                    while (j<frames[i]->getAnglesNum())
                    {
                        Vector3D r2=frames[i]->at(j).getTail();
                        Vector3D r1=frames[i]->getCenter();
                        Vector3D r3;
                        if (j+1==frames[i]->getAnglesNum())
                            r3=frames[i]->at(0).getTail();
                        else
                            r3=frames[i]->at(j+1).getTail();
                        Vector3D e1=r2-r1;
                        Vector3D e2=r3-r1;
                        double mixedProd=Vector3D::mixedProduct(tau,e1,e2);
                        if (fabs(mixedProd)<0.0000000001)
                        {
                            double t=Vector3D::mixedProduct(r1-vortons[i].getMid(),e1,e2)/mixedProd;
                            if (t>0.0 && t<1.0)
                            {
                                Vector3D rtilda=vortons[i].getMid()+t*tau;
                                Vector3D a=r1-rtilda;
                                Vector3D b=r2-rtilda;
                                Vector3D c=r3-rtilda;
                                if (coDirectionallyCheck(a,b,c))
                                {
                                    vortons[i].setMid(vortons[i].getMid()+2*t*tau);
                                    vortons[i].setTail(vortons[i].getTail()+2*t*tau);
                                    founded=true;
                                    break;
                                }
                            }
                        }
                        j++;
                    }
                }
                if (founded)
                    break;
            }
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideSphereLayer(vortons[i],center,radius,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    qDebug()<<"getBackGA";
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности цилиндра в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] height Высота цилиндра
\param[in] diameter Диаметр цилиндра
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
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

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности тела вращения в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] xBeg Минимальное значение координаты х тела
\param[in] xEnd Максимальное значение координаты х тела
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations:: getBackAndRotateRotationBody(QVector<Vorton>& vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideRotationBody(vortons[i],bodyNose,xEnd,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationBodyLayer(vortons[i],bodyNose,xEnd,layerHeight, forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
    qDebug()<<"getBack";
}

void FrameCalculations::getBackAndRotateRotationBodyv2(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        int inside=FrameCalculations::insideRotationBodyv2(vortons[i],bodyNose,xEnd,forming);
        if (inside!=-1)
        {
            QPair<double,int> closest;
            if (inside==1)
                closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
            else {
                closest=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
            }
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        int insideLayer=FrameCalculations::insideRotationBodyLayer(vortons[i],bodyNose,xEnd,layerHeight, forming);
        if(insideLayer!=-1)
        {
            QPair<double,int> closest;
            if (insideLayer==1)
                    closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
            else {
                closest=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
            }
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
    qDebug()<<"getBack";
}

void FrameCalculations::getBackAndRotateMovingRotationBody(QVector<Vorton> &vortons,Vector3D centerMassWorld, Vector3D oldCenterWorld, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, Eigen::Matrix3d rotationMatrix, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    Eigen::Matrix3d inverted=rotationMatrix.inverse();
    for (int i=0; i<vortons.size();i++)
    {
        //qDebug()<<oldCenterWorld.x()-centerMassWorld.x()<<" "<<oldCenterWorld.y()-centerMassWorld.y()<<" "<<oldCenterWorld.z()-centerMassWorld.z();
        Vorton copyVort=vortons[i];
        copyVort.setMid(centerMassWorld+fromEigenVector(inverted*toEigenVector(copyVort.getMid()-centerMassWorld)));
        copyVort.setTail(centerMassWorld+fromEigenVector(inverted*toEigenVector(copyVort.getTail()-centerMassWorld)));
        qDebug()<<(oldCenterWorld-centerMassWorld).x()<<" "<<(oldCenterWorld-centerMassWorld).y()<<" "<<(oldCenterWorld-centerMassWorld).z();
        copyVort.setTail(copyVort.getTail()+oldCenterWorld-centerMassWorld);

        copyVort.setMid(copyVort.getMid()+oldCenterWorld-centerMassWorld);
        if (FrameCalculations::insideRotationBody(copyVort,bodyNose,xEnd,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationBodyLayer(copyVort,bodyNose,xEnd,layerHeight, forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::getBackAndRotateMovingRotationCutBody(QVector<Vorton> &vortons, Vector3D centerMassWorld, Vector3D oldCenterWorld, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, Eigen::Matrix3d rotationMatrix, FormingParametersRBC forming)
{
    QTime start=QTime::currentTime();
    Eigen::Matrix3d inverted=rotationMatrix.inverse();
    for (int i=0; i<vortons.size();i++)
    {
        //qDebug()<<oldCenterWorld.x()-centerMassWorld.x()<<" "<<oldCenterWorld.y()-centerMassWorld.y()<<" "<<oldCenterWorld.z()-centerMassWorld.z();
        Vorton copyVort=vortons[i];
        copyVort.setMid(centerMassWorld+fromEigenVector(inverted*toEigenVector(copyVort.getMid()-centerMassWorld)));
        copyVort.setTail(centerMassWorld+fromEigenVector(inverted*toEigenVector(copyVort.getTail()-centerMassWorld)));
        //qDebug()<<(oldCenterWorld-centerMassWorld).x()<<" "<<(oldCenterWorld-centerMassWorld).y()<<" "<<(oldCenterWorld-centerMassWorld).z();
        copyVort.setTail(copyVort.getTail()+oldCenterWorld-centerMassWorld);

        copyVort.setMid(copyVort.getMid()+oldCenterWorld-centerMassWorld);
        if (FrameCalculations::insideRotationCutBody(copyVort,xEnd,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(copyVort,xEnd,layerHeight, bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

void FrameCalculations::getBackAndRotateMovingLaunchedRotationCutBody(QVector<Vorton> &vortons, Vector3D centerMassWorld, Vector3D oldCenterWorld, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, Eigen::Matrix3d rotationMatrix, Eigen::Matrix3d rotationNullMatrix,FormingParametersRBC forming)
{
    QTime start=QTime::currentTime();
    Eigen::Matrix3d inverted=rotationMatrix.inverse();
    for (int i=vortons.size()-1; i>=0;i--)
    {
        //qDebug()<<oldCenterWorld.x()-centerMassWorld.x()<<" "<<oldCenterWorld.y()-centerMassWorld.y()<<" "<<oldCenterWorld.z()-centerMassWorld.z();
        Vorton copyVort=vortons[i];
        copyVort.setMid(centerMassWorld+fromEigenVector(rotationNullMatrix*inverted*toEigenVector(copyVort.getMid()-centerMassWorld)));
        copyVort.setTail(centerMassWorld+fromEigenVector(rotationNullMatrix*inverted*toEigenVector(copyVort.getTail()-centerMassWorld)));
        //qDebug()<<(oldCenterWorld-centerMassWorld).x()<<" "<<(oldCenterWorld-centerMassWorld).y()<<" "<<(oldCenterWorld-centerMassWorld).z();
        copyVort.setTail(copyVort.getTail()+oldCenterWorld-centerMassWorld);

        copyVort.setMid(copyVort.getMid()+oldCenterWorld-centerMassWorld);
        if (FrameCalculations::insideRotationCutBody(copyVort,xEnd,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(copyVort,xEnd,layerHeight, bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
        if (FrameCalculations::insideScreen(vortons[i]))
        {
            vortons.remove(i);
//            if (vortons[i].getMid().x()>0.0)
//                vortons[i].setMid(vortons[i].getMid()-2.0*Vector3D(vortons[i].getMid().x(),0.0,0.0));
//            if (vortons[i].getTail().x()>0.0)
//                vortons[i].setTail(vortons[i].getTail()-2.0*Vector3D(vortons[i].getTail().x(),0.0,0.0));
            counters.underScreenNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности тела вращения со срезом в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] xBeg Минимальное значение координаты х тела
\param[in] xEnd Максимальное значение координаты х тела
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateRotationCutBody(QVector<Vorton> &vortons, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParametersRBC forming)
{
    QTime start=QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xEnd, bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xEnd,layerHeight,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
    qDebug()<<"getBack";
}

void FrameCalculations::getBackAndRotateRotationCutBodyGA(QVector<Vorton> &vortons, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParametersRBC forming)
{
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xEnd, bodyNose,forming))
        {
            bool founded=false;
            Vector3D tau=oldvortons[i].getMid()-vortons[i].getMid();
            for (int i=0;i<frames.size();i++)
            {

                if (frames[i]->getAnglesNum()==4)
                {
                    int j=0;
                    while (j<3)
                    {
                        Vector3D r1=frames[i]->at(j).getTail();
                        Vector3D r2=frames[i]->at(j+1).getTail();
                        Vector3D r3;
                        if (j+2==4)
                            r3=frames[i]->at(0).getTail();
                        else
                            r3=frames[i]->at(j+2).getTail();
                        Vector3D e1=r2-r1;
                        Vector3D e2=r3-r1;
                        double mixedProd=Vector3D::mixedProduct(tau,e1,e2);
                        if (fabs(mixedProd)<0.0000000001)
                        {
                            double t=Vector3D::mixedProduct(r1-vortons[i].getMid(),e1,e2)/mixedProd;
                            if (t>0.0 && t<1.0)
                            {
                                Vector3D rtilda=vortons[i].getMid()+t*tau;
                                Vector3D a=r1-rtilda;
                                Vector3D b=r2-rtilda;
                                Vector3D c=r3-rtilda;


                                if (coDirectionallyCheck(a,b,c))
                                {
                                    vortons[i].setMid(vortons[i].getMid()+2*t*tau);
                                    vortons[i].setTail(vortons[i].getTail()+2*t*tau);
                                    founded=true;
                                    break;
                                }
                            }
                        }
                        j+=2;
                    }
                }
                else {
                    int j=0;
                    while (j<frames[i]->getAnglesNum())
                    {
                        Vector3D r2=frames[i]->at(j).getTail();
                        Vector3D r1=frames[i]->getCenter();
                        Vector3D r3;
                        if (j+1==frames[i]->getAnglesNum())
                            r3=frames[i]->at(0).getTail();
                        else
                            r3=frames[i]->at(j+1).getTail();
                        Vector3D e1=r2-r1;
                        Vector3D e2=r3-r1;
                        double mixedProd=Vector3D::mixedProduct(tau,e1,e2);
                        if (fabs(mixedProd)<0.0000000001)
                        {
                            double t=Vector3D::mixedProduct(r1-vortons[i].getMid(),e1,e2)/mixedProd;
                            if (t>0.0 && t<1.0)
                            {
                                Vector3D rtilda=vortons[i].getMid()+t*tau;
                                Vector3D a=r1-rtilda;
                                Vector3D b=r2-rtilda;
                                Vector3D c=r3-rtilda;
                                if (coDirectionallyCheck(a,b,c))
                                {
                                    vortons[i].setMid(vortons[i].getMid()+2*t*tau);
                                    vortons[i].setTail(vortons[i].getTail()+2*t*tau);
                                    founded=true;
                                    break;
                                }
                            }
                        }
                        j++;
                    }
                }
                if (founded)
                    break;
            }
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xEnd,layerHeight,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    qDebug()<<"getBackGA";
}

void FrameCalculations::getBackAndRotateRotationBodyGA(QVector<Vorton> &vortons,QVector<Vorton> &oldvortons,QVector<std::shared_ptr<MultiFrame>> &frames,const double xEnd, const Vector3D bodyNose, FormingParameters forming,const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationBody(vortons[i], bodyNose,xEnd,forming))
        {
            bool founded=false;
            Vector3D tau=oldvortons[i].getMid()-vortons[i].getMid();
            for (int i=0;i<frames.size();i++)
            {

                if (frames[i]->getAnglesNum()==4)
                {
                    int j=0;
                    while (j<3)
                    {
                        Vector3D r1=frames[i]->at(j).getTail();
                        Vector3D r2=frames[i]->at(j+1).getTail();
                        Vector3D r3;
                        if (j+2==4)
                            r3=frames[i]->at(0).getTail();
                        else
                            r3=frames[i]->at(j+2).getTail();
                        Vector3D e1=r2-r1;
                        Vector3D e2=r3-r1;
                        double mixedProd=Vector3D::mixedProduct(tau,e1,e2);
                        if (fabs(mixedProd)<0.0000000001)
                        {
                            double t=Vector3D::mixedProduct(r1-vortons[i].getMid(),e1,e2)/mixedProd;
                            if (t>0.0 && t<1.0)
                            {
                                Vector3D rtilda=vortons[i].getMid()+t*tau;
                                Vector3D a=r1-rtilda;
                                Vector3D b=r2-rtilda;
                                Vector3D c=r3-rtilda;


                                if (coDirectionallyCheck(a,b,c))
                                {
                                    vortons[i].setMid(vortons[i].getMid()+2*t*tau);
                                    vortons[i].setTail(vortons[i].getTail()+2*t*tau);
                                    founded=true;
                                    break;
                                }
                            }
                        }
                        j+=2;
                    }
                }
                else {
                    int j=0;
                    while (j<frames[i]->getAnglesNum())
                    {
                        Vector3D r2=frames[i]->at(j).getTail();
                        Vector3D r1=frames[i]->getCenter();
                        Vector3D r3;
                        if (j+1==frames[i]->getAnglesNum())
                            r3=frames[i]->at(0).getTail();
                        else
                            r3=frames[i]->at(j+1).getTail();
                        Vector3D e1=r2-r1;
                        Vector3D e2=r3-r1;
                        double mixedProd=Vector3D::mixedProduct(tau,e1,e2);
                        if (fabs(mixedProd)<0.0000000001)
                        {
                            double t=Vector3D::mixedProduct(r1-vortons[i].getMid(),e1,e2)/mixedProd;
                            if (t>0.0 && t<1.0)
                            {
                                Vector3D rtilda=vortons[i].getMid()+t*tau;
                                Vector3D a=r1-rtilda;
                                Vector3D b=r2-rtilda;
                                Vector3D c=r3-rtilda;
                                if (coDirectionallyCheck(a,b,c))
                                {
                                    vortons[i].setMid(vortons[i].getMid()+2*t*tau);
                                    vortons[i].setTail(vortons[i].getTail()+2*t*tau);
                                    founded=true;
                                    break;
                                }
                            }
                        }
                        j++;
                    }
                }
                if (founded)
                    break;
            }
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationBodyLayer(vortons[i],bodyNose,xEnd,layerHeight,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    qDebug()<<"getBackGA";
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности тела вращения со срезом в слое при решении задачи старта
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] xBeg Минимальное значение координаты х тела
\param[in] xEnd Максимальное значение координаты х тела
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateRotationCutLaunchedBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParametersRBC forming)
{
    QTime start=QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xEnd,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xEnd,layerHeight,bodyNose,forming))
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

void FrameCalculations::translateAndRotate(QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vorton>& vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel,
                                           QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals, QVector<Vector3D>& controlPointsRaised, QVector<std::shared_ptr<MultiFrame>>& oldFrames, QVector<Vector3D>& oldControlPoints, QVector<Vector3D>& oldNormals, QVector<Vector3D>& oldControlPointsRaised, Vector3D& angVel)
{

    dInitODE();
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world,0.0,0.0,0.0);
    dBodyID rocket = dBodyCreate(world);
    dBodySetPosition(rocket, nullCenter.x(),nullCenter.y(),nullCenter.z());

    dMatrix3 rot;
    for (int i=0; i<3;i++)
        for (int j=0; j<3; j++)
            rot[i*4+j]=rotationMatrix(i,j);

    dBodySetRotation(rocket,rot);
    dMass rocketMass;
    dMassSetZero(&rocketMass);

    dMassSetParameters(&rocketMass,mass,massCenter.x(),massCenter.y(),massCenter.z(),inertiaTensor(0,0),inertiaTensor(1,1),inertiaTensor(2,2),inertiaTensor(0,1),inertiaTensor(0,2),inertiaTensor(1,2));
    dBodySetMass(rocket,&rocketMass);
    dBodyAddForce(rocket,force.x(),force.y(),force.z());
//     qDebug()<<"force is "<<force.x()<<" "<< force.y()<<" "<<force.z();
    dBodyAddTorque(rocket, tongue.x(),tongue.y(),tongue.z());
//    qDebug()<<"torgue is "<<tongue.x()<<" "<< tongue.y()<<" "<<tongue.z();
    dBodySetLinearVel(rocket,linearVel.x(),linearVel.y(),linearVel.z());
    dWorldStep(world,time);
    const dReal* Pos;
    const dReal* angle;

    Pos = dBodyGetPosition(rocket);

    float pos[3] = { Pos[0], Pos[1], Pos[2] };
//    qDebug()<<"Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    Vector3D translation(pos[0]-nullCenter.x(),pos[1]-nullCenter.y(),pos[2]-nullCenter.z());
//    qDebug()<<translation.x()<<" "<<translation.y()<<" "<<translation.z();
//    qDebug()<<"translation is "<<translation.x()<<" "<< translation.y()<<" "<<translation.z();
    center=Vector3D(pos[0],pos[1],pos[2]);

    angle = dBodyGetRotation(rocket);
    const dReal* angularVel;
    angularVel=dBodyGetAngularVel(rocket);
    angVel=Vector3D(angularVel[0],angularVel[1],angularVel[2]);
    float vel[3]={angularVel[0],angularVel[1],angularVel[2]};
    float angles[12] = {angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]};
    //qDebug()<<vel[0]<<" "<<vel[1]<<" "<<vel[2];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            rotationMatrix(i,j)=angles[i*4+j];
    Eigen::Vector3d newTranslate;
    for (int i=0; i<frames.size();i++)
        for (int j=0; j<frames[i]->getAnglesNum();j++)
        {
//            frames[i]->at(j).setMid(oldFrames[i]->at(j).getMid()+translation);
//            frames[i]->at(j).setTail(oldFrames[i]->at(j).getTail()+translation);

            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*Eigen::Vector3d(oldFrames[i]->at(j).getMid().x()-massCenter.x(),oldFrames[i]->at(j).getMid().y()-massCenter.y(),oldFrames[i]->at(j).getMid().z()-massCenter.z());
            frames[i]->at(j).setMid(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                                rotationMatrix*Eigen::Vector3d(oldFrames[i]->at(j).getTail().x()-massCenter.x(),oldFrames[i]->at(j).getTail().y()-massCenter.y(),oldFrames[i]->at(j).getTail().z()-massCenter.z());
            frames[i]->at(j).setTail(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            frames[i]->setCenter(oldFrames[i]->getCenter()+translation);
        }

    for (int i=0; i<controlPoints.size(); i++)
    {


        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*Eigen::Vector3d(oldControlPoints[i].x()-massCenter.x(),oldControlPoints[i].y()-massCenter.y(),oldControlPoints[i].z()-massCenter.z());
        controlPoints[i]=/*oldControlPoints[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*Eigen::Vector3d(oldControlPointsRaised[i].x()-massCenter.x(),oldControlPointsRaised[i].y()-massCenter.y(),oldControlPointsRaised[i].z()-massCenter.z());
        controlPointsRaised[i]=/*oldControlPointsRaised[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*Eigen::Vector3d(oldNormals[i].x()-massCenter.x(),oldNormals[i].y()-massCenter.y(),oldNormals[i].z()-massCenter.z());
        normals[i]=/*oldNormals[i]+*/Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());

    }
//    for (int i=0; i<vortons.size();i++)
//    {
//        vortons[i].setMid(vortons[i].getMid()+translation);
//        vortons[i].setTail(vortons[i].getTail()+translation);
//        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
//                rotationMatrix*Eigen::Vector3d(vortons[i].getMid().x()-center.x(),vortons[i].getMid().y()-center.y(),vortons[i].getMid().z()-center.z());
//        vortons[i].setMid(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
//        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
//                rotationMatrix*Eigen::Vector3d(vortons[i].getTail().x()-center.x(),vortons[i].getTail().y()-center.y(),vortons[i].getTail().z()-center.z());
//         vortons[i].setTail(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
//    }

    dCloseODE();
}

void FrameCalculations::translateAndRotatev2(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vorton> &vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel, QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals, QVector<Vector3D> &controlPointsRaised, QVector<std::shared_ptr<MultiFrame> > &oldFrames, QVector<Vector3D> &oldControlPoints, QVector<Vector3D> &oldNormals, QVector<Vector3D> &oldControlPointsRaised, Vector3D &angVel, Vector3D& bodyNose, double& xend)
{
    dInitODE();
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world,0.0,0.0,0.0);
    dBodyID rocket = dBodyCreate(world);
    dBodySetPosition(rocket, center.x(),center.y(),center.z());
    //nullCenter=center;
    dMatrix3 rot;
    for (int i=0; i<3;i++)
        for (int j=0; j<3; j++)
            rot[i*4+j]=rotationMatrix(i,j);

    dBodySetRotation(rocket,rot);
    dMass rocketMass;
    dMassSetZero(&rocketMass);

    dMassSetParameters(&rocketMass,mass,massCenter.x(),massCenter.y(),massCenter.z(),inertiaTensor(0,0),inertiaTensor(1,1),inertiaTensor(2,2),inertiaTensor(0,1),inertiaTensor(0,2),inertiaTensor(1,2));
    dBodySetMass(rocket,&rocketMass);
    dBodyAddForce(rocket,force.x(),force.y(),force.z());
//     qDebug()<<"force is "<<force.x()<<" "<< force.y()<<" "<<force.z();
    dBodyAddTorque(rocket, tongue.x(),tongue.y(),tongue.z());
//    qDebug()<<"torgue is "<<tongue.x()<<" "<< tongue.y()<<" "<<tongue.z();
    dBodySetLinearVel(rocket,linearVel.x(),linearVel.y(),linearVel.z());
    dWorldStep(world,time);
    const dReal* Pos;
    const dReal* angle;

    Pos = dBodyGetPosition(rocket);

    float pos[3] = { Pos[0], Pos[1], Pos[2] };
//    qDebug()<<"Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    Vector3D translation(pos[0]-center.x(),pos[1]-center.y(),pos[2]-center.z());
//    qDebug()<<translation.x()<<" "<<translation.y()<<" "<<translation.z();
//    qDebug()<<"translation is "<<translation.x()<<" "<< translation.y()<<" "<<translation.z();
    center=Vector3D(pos[0],pos[1],pos[2]);
    bodyNose+=translation;
    xend+=translation.x();
    angle = dBodyGetRotation(rocket);
    const dReal* angularVel;
    angularVel=dBodyGetAngularVel(rocket);
    angVel=Vector3D(angularVel[0],angularVel[1],angularVel[2]);
    float vel[3]={angularVel[0],angularVel[1],angularVel[2]};
    float angles[12] = {angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]};
    //qDebug()<<vel[0]<<" "<<vel[1]<<" "<<vel[2];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            rotationMatrix(i,j)=angles[i*4+j];
    Eigen::Vector3d newTranslate;
    for (int i=0; i<frames.size();i++)
        for (int j=0; j<frames[i]->getAnglesNum();j++)
        {
//            frames[i]->at(j).setMid(oldFrames[i]->at(j).getMid()+translation);
//            frames[i]->at(j).setTail(oldFrames[i]->at(j).getTail()+translation);

            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*Eigen::Vector3d(frames[i]->at(j).getMid().x()-massCenter.x(),frames[i]->at(j).getMid().y()-massCenter.y(),frames[i]->at(j).getMid().z()-massCenter.z());
            frames[i]->at(j).setMid(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                                rotationMatrix*Eigen::Vector3d(frames[i]->at(j).getTail().x()-massCenter.x(),frames[i]->at(j).getTail().y()-massCenter.y(),frames[i]->at(j).getTail().z()-massCenter.z());
            frames[i]->at(j).setTail(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            frames[i]->setCenter(frames[i]->getCenter()+translation);
        }

    for (int i=0; i<controlPoints.size(); i++)
    {


        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*Eigen::Vector3d(controlPoints[i].x()-massCenter.x(),controlPoints[i].y()-massCenter.y(),controlPoints[i].z()-massCenter.z());
        controlPoints[i]=/*oldControlPoints[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*Eigen::Vector3d(controlPointsRaised[i].x()-massCenter.x(),controlPointsRaised[i].y()-massCenter.y(),controlPointsRaised[i].z()-massCenter.z());
        controlPointsRaised[i]=/*oldControlPointsRaised[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*Eigen::Vector3d(normals[i].x()-massCenter.x(),normals[i].y()-massCenter.y(),normals[i].z()-massCenter.z());
        normals[i]=/*oldNormals[i]+*/Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());

    }
//    for (int i=0; i<vortons.size();i++)
//    {
//        vortons[i].setMid(vortons[i].getMid()+translation);
//        vortons[i].setTail(vortons[i].getTail()+translation);
//        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
//                rotationMatrix*Eigen::Vector3d(vortons[i].getMid().x()-center.x(),vortons[i].getMid().y()-center.y(),vortons[i].getMid().z()-center.z());
//        vortons[i].setMid(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
//        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
//                rotationMatrix*Eigen::Vector3d(vortons[i].getTail().x()-center.x(),vortons[i].getTail().y()-center.y(),vortons[i].getTail().z()-center.z());
//         vortons[i].setTail(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
//    }

    dCloseODE();
}

void FrameCalculations::translateAndRotatev3(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vorton> &vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Eigen::Matrix3d &rotationNullMatrix, Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel, QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals, QVector<Vector3D> &controlPointsRaised, QVector<std::shared_ptr<MultiFrame> > &oldFrames, QVector<Vector3D> &oldControlPoints, QVector<Vector3D> &oldNormals, QVector<Vector3D> &oldControlPointsRaised, Vector3D &angVel, Vector3D &bodyNose, double &xend)
{
    dInitODE();
    Eigen::Matrix3d inverted=rotationNullMatrix.inverse();
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world,0.0,0.0,0.0);
    dBodyID rocket = dBodyCreate(world);
    dBodySetPosition(rocket, center.x(),center.y(),center.z());
    //nullCenter=center;
    dMatrix3 rot;
    for (int i=0; i<3;i++)
        for (int j=0; j<3; j++)
            rot[i*4+j]=rotationMatrix(i,j);

    dBodySetRotation(rocket,rot);
    dMass rocketMass;
    dMassSetZero(&rocketMass);

    dMassSetParameters(&rocketMass,mass,massCenter.x(),massCenter.y(),massCenter.z(),inertiaTensor(0,0),inertiaTensor(1,1),inertiaTensor(2,2),inertiaTensor(0,1),inertiaTensor(0,2),inertiaTensor(1,2));
    dBodySetMass(rocket,&rocketMass);
//    dBodyAddForce(rocket,force.x(),force.y(),force.z());
//     qDebug()<<"force is "<<force.x()<<" "<< force.y()<<" "<<force.z();
//    dBodyAddTorque(rocket, tongue.x(),tongue.y(),tongue.z());
//    qDebug()<<"torgue is "<<tongue.x()<<" "<< tongue.y()<<" "<<tongue.z();
    dBodySetLinearVel(rocket,linearVel.x(),linearVel.y(),linearVel.z());
    dWorldStep(world,time);
    const dReal* Pos;
    const dReal* angle;

    Pos = dBodyGetPosition(rocket);

    float pos[3] = { Pos[0], Pos[1], Pos[2] };
//    qDebug()<<"Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    Vector3D translation(pos[0]-center.x(),pos[1]-center.y(),pos[2]-center.z());
//    qDebug()<<translation.x()<<" "<<translation.y()<<" "<<translation.z();
//    qDebug()<<"translation is "<<translation.x()<<" "<< translation.y()<<" "<<translation.z();
    center=Vector3D(pos[0],pos[1],pos[2]);
    bodyNose+=translation;
    xend+=translation.x();
    angle = dBodyGetRotation(rocket);
    const dReal* angularVel;
    angularVel=dBodyGetAngularVel(rocket);
    angVel=Vector3D(angularVel[0],angularVel[1],angularVel[2]);
    float vel[3]={angularVel[0],angularVel[1],angularVel[2]};
    float angles[12] = {angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]};
    //qDebug()<<vel[0]<<" "<<vel[1]<<" "<<vel[2];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            rotationMatrix(i,j)=angles[i*4+j];
    Eigen::Vector3d newTranslate;
    for (int i=0; i<frames.size();i++)
        for (int j=0; j<frames[i]->getAnglesNum();j++)
        {
//            frames[i]->at(j).setMid(oldFrames[i]->at(j).getMid()+translation);
//            frames[i]->at(j).setTail(oldFrames[i]->at(j).getTail()+translation);

            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*inverted*Eigen::Vector3d(frames[i]->at(j).getMid().x()-massCenter.x(),frames[i]->at(j).getMid().y()-massCenter.y(),frames[i]->at(j).getMid().z()-massCenter.z());
            frames[i]->at(j).setMid(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                                rotationMatrix*inverted*Eigen::Vector3d(frames[i]->at(j).getTail().x()-massCenter.x(),frames[i]->at(j).getTail().y()-massCenter.y(),frames[i]->at(j).getTail().z()-massCenter.z());
            frames[i]->at(j).setTail(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            frames[i]->setCenter(frames[i]->getCenter()+translation);
        }


    for (int i=0; i<controlPoints.size(); i++)
    {


        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*inverted*Eigen::Vector3d(controlPoints[i].x()-massCenter.x(),controlPoints[i].y()-massCenter.y(),controlPoints[i].z()-massCenter.z());
        controlPoints[i]=/*oldControlPoints[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*inverted*Eigen::Vector3d(controlPointsRaised[i].x()-massCenter.x(),controlPointsRaised[i].y()-massCenter.y(),controlPointsRaised[i].z()-massCenter.z());
        controlPointsRaised[i]=/*oldControlPointsRaised[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*inverted*Eigen::Vector3d(normals[i].x()-massCenter.x(),normals[i].y()-massCenter.y(),normals[i].z()-massCenter.z());
        normals[i]=/*oldNormals[i]+*/Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());

    }
    for (int i=0; i<vortons.size();i++)
    {
        vortons[i].setMid(vortons[i].getMid()+translation);
        vortons[i].setTail(vortons[i].getTail()+translation);
        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
                rotationMatrix*Eigen::Vector3d(vortons[i].getMid().x()-center.x(),vortons[i].getMid().y()-center.y(),vortons[i].getMid().z()-center.z());
        vortons[i].setMid(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
                rotationMatrix*Eigen::Vector3d(vortons[i].getTail().x()-center.x(),vortons[i].getTail().y()-center.y(),vortons[i].getTail().z()-center.z());
         vortons[i].setTail(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
    }

    dCloseODE();
}

void FrameCalculations::translateAndRotatev3(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vorton> &vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Eigen::Matrix3d &rotationNullMatrix, Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel, QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals, QVector<Vector3D> &controlPointsRaised, QVector<std::shared_ptr<MultiFrame> > &oldFrames, QVector<Vector3D> &oldControlPoints, QVector<Vector3D> &oldNormals, QVector<Vector3D> &oldControlPointsRaised, Vector3D &angVel, Vector3D &bodyNose, double &xend,QVector<QVector<Vector3D>> xGraphNodes, QVector<QVector<Vector3D>>& yGraphNodes, QVector<QVector<Vector3D>>& zGraphNodes)
{
    dInitODE();
    Eigen::Matrix3d inverted=rotationNullMatrix.inverse();
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world,0.0,0.0,0.0);
    dBodyID rocket = dBodyCreate(world);
    dBodySetPosition(rocket, center.x(),center.y(),center.z());
    //nullCenter=center;
    dMatrix3 rot;
    for (int i=0; i<3;i++)
        for (int j=0; j<3; j++)
            rot[i*4+j]=rotationMatrix(i,j);

    dBodySetRotation(rocket,rot);
    dMass rocketMass;
    dMassSetZero(&rocketMass);

    dMassSetParameters(&rocketMass,mass,massCenter.x(),massCenter.y(),massCenter.z(),inertiaTensor(0,0),inertiaTensor(1,1),inertiaTensor(2,2),inertiaTensor(0,1),inertiaTensor(0,2),inertiaTensor(1,2));
    dBodySetMass(rocket,&rocketMass);
    dBodyAddForce(rocket,force.x(),force.y(),force.z());
//     qDebug()<<"force is "<<force.x()<<" "<< force.y()<<" "<<force.z();
    dBodyAddTorque(rocket, tongue.x(),tongue.y(),tongue.z());
//    qDebug()<<"torgue is "<<tongue.x()<<" "<< tongue.y()<<" "<<tongue.z();
    dBodySetLinearVel(rocket,linearVel.x(),linearVel.y(),linearVel.z());
    dWorldStep(world,time);
    const dReal* Pos;
    const dReal* angle;

    Pos = dBodyGetPosition(rocket);

    float pos[3] = { Pos[0], Pos[1], Pos[2] };
//    qDebug()<<"Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    Vector3D translation(pos[0]-center.x(),pos[1]-center.y(),pos[2]-center.z());
    qDebug()<<"translation is"<<translation.x()<<" "<<translation.y()<<" "<<translation.z();
//    qDebug()<<translation.x()<<" "<<translation.y()<<" "<<translation.z();
//    qDebug()<<"translation is "<<translation.x()<<" "<< translation.y()<<" "<<translation.z();
    center=Vector3D(pos[0],pos[1],pos[2]);
    bodyNose+=translation;
    xend+=translation.x();
    angle = dBodyGetRotation(rocket);
    const dReal* angularVel;
    angularVel=dBodyGetAngularVel(rocket);
    angVel=Vector3D(angularVel[0],angularVel[1],angularVel[2]);
    float vel[3]={angularVel[0],angularVel[1],angularVel[2]};
    float angles[12] = {angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]};
    //qDebug()<<vel[0]<<" "<<vel[1]<<" "<<vel[2];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            rotationMatrix(i,j)=angles[i*4+j];
    Eigen::Vector3d newTranslate;
    for (int i=0; i<frames.size();i++)
        for (int j=0; j<frames[i]->getAnglesNum();j++)
        {
//            frames[i]->at(j).setMid(oldFrames[i]->at(j).getMid()+translation);
//            frames[i]->at(j).setTail(oldFrames[i]->at(j).getTail()+translation);

            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*inverted*Eigen::Vector3d(frames[i]->at(j).getMid().x()-massCenter.x(),frames[i]->at(j).getMid().y()-massCenter.y(),frames[i]->at(j).getMid().z()-massCenter.z());
            frames[i]->at(j).setMid(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                                rotationMatrix*inverted*Eigen::Vector3d(frames[i]->at(j).getTail().x()-massCenter.x(),frames[i]->at(j).getTail().y()-massCenter.y(),frames[i]->at(j).getTail().z()-massCenter.z());
            frames[i]->at(j).setTail(translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
            frames[i]->setCenter(frames[i]->getCenter()+translation);
        }

    for (int i=0; i<xGraphNodes.size();i++)
        for (int j=0;j<xGraphNodes.size();j++)
        {
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*inverted*Eigen::Vector3d(xGraphNodes[i][j].x()-massCenter.x(),xGraphNodes[i][j].y()-massCenter.y(),xGraphNodes[i][j].z()-massCenter.z());
            xGraphNodes[i][j]=translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        }
    for (int i=0; i<yGraphNodes.size();i++)
        for (int j=0;j<yGraphNodes.size();j++)
        {
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*inverted*Eigen::Vector3d(yGraphNodes[i][j].x()-massCenter.x(),yGraphNodes[i][j].y()-massCenter.y(),yGraphNodes[i][j].z()-massCenter.z());
            yGraphNodes[i][j]=translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        }
    for (int i=0; i<zGraphNodes.size();i++)
        for (int j=0;j<zGraphNodes.size();j++)
        {
            newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                    rotationMatrix*inverted*Eigen::Vector3d(zGraphNodes[i][j].x()-massCenter.x(),zGraphNodes[i][j].y()-massCenter.y(),zGraphNodes[i][j].z()-massCenter.z());
            zGraphNodes[i][j]=translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        }
    for (int i=0; i<controlPoints.size(); i++)
    {


        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*inverted*Eigen::Vector3d(controlPoints[i].x()-massCenter.x(),controlPoints[i].y()-massCenter.y(),controlPoints[i].z()-massCenter.z());
        controlPoints[i]=/*oldControlPoints[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*inverted*Eigen::Vector3d(controlPointsRaised[i].x()-massCenter.x(),controlPointsRaised[i].y()-massCenter.y(),controlPointsRaised[i].z()-massCenter.z());
        controlPointsRaised[i]=/*oldControlPointsRaised[i]+*/translation+Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());
        newTranslate=Eigen::Vector3d(massCenter.x(),massCenter.y(),massCenter.z())+
                rotationMatrix*inverted*Eigen::Vector3d(normals[i].x()-massCenter.x(),normals[i].y()-massCenter.y(),normals[i].z()-massCenter.z());
        normals[i]=/*oldNormals[i]+*/Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z());

    }
//    for (int i=0; i<vortons.size();i++)
//    {
//        vortons[i].setMid(vortons[i].getMid()+translation);
//        vortons[i].setTail(vortons[i].getTail()+translation);
//        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
//                rotationMatrix*Eigen::Vector3d(vortons[i].getMid().x()-center.x(),vortons[i].getMid().y()-center.y(),vortons[i].getMid().z()-center.z());
//        vortons[i].setMid(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
//        newTranslate=Eigen::Vector3d(center.x(),center.y(),center.z())+
//                rotationMatrix*Eigen::Vector3d(vortons[i].getTail().x()-center.x(),vortons[i].getTail().y()-center.y(),vortons[i].getTail().z()-center.z());
//         vortons[i].setTail(Vector3D(newTranslate.x(),newTranslate.y(),newTranslate.z()));
//    }

    dCloseODE();
}

void FrameCalculations::displace(QVector<Vorton> &vortons, double dlMax, double dlMin)
{
    for (int i=0; i<vortons.size(); i++)
    {
        vortons[i].setMid(vortons[i].getMid()+vortons[i].getMove());
        vortons[i].setTail(vortons[i].getTail()+vortons[i].getMove()+vortons[i].getElongation());

        double diff=(vortons[i].getTail()-vortons[i].getMid()).length();
       if (diff<dlMin)
       {
           vortons[i].setTail(vortons[i].getMid()+(vortons[i].getTail()-vortons[i].getMid())*(dlMin/diff));
       }
       if (diff>dlMax)
       {
           vortons[i].setTail(vortons[i].getMid()+(vortons[i].getTail()-vortons[i].getMid())*(dlMax/diff));
       }

    }
}

void FrameCalculations::getBackRotationBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        int inside=FrameCalculations::insideRotationBodyv2(vortons[i],bodyNose,xEnd,forming);
        if (inside!=-1)
        {
            QPair<double,int> closest;
            if (inside==1)
                closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
            else {
                closest=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
            }
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            vortons[i].setMove(vortons[i].getMove()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
    qDebug()<<"getBack";
}

void FrameCalculations::rotateRotationBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming)
{
    for (int i=0; i<vortons.size(); i++)
    {
        int insideLayer=FrameCalculations::insideRotationBodyLayer(vortons[i],bodyNose,xEnd,layerHeight, forming);
        if(insideLayer!=-1)
        {
            QPair<double,int> closest;
            if (insideLayer==1)
                closest=BodyFragmentation::findClosest(vortons[i].getTail(),controlPoints, normals);
            else {
                closest=BodyFragmentation::findClosest(2.0*vortons[i].getMid()-vortons[i].getTail(),controlPoints, normals);
            }
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
}

void FrameCalculations::velForStreamLines(QVector<Vector3D> &velocities, Vector3D streamVel,double step, QVector<Vorton> &freeVortons,  QPair<int,int> boundaries)
{
    velocities.clear();
     for (double i1=-1*(boundaries.second+1); i1<=boundaries.second+1.0000001;i1=i1+step)
    for (double j1=-1*(boundaries.second+1);j1<=boundaries.second+1.0000001;j1=j1+step)
        for (double k1=-1.0;k1<=boundaries.first+1.0000001;k1=k1+step)
            velocities.push_back(FrameCalculations::velocity(Vector3D(k1,j1,i1),streamVel,freeVortons));
}

bool FrameCalculations::coDirectionallyCheck(const Vector3D a, const Vector3D b, const Vector3D c)
{
    Vector3D cross1=Vector3D::crossProduct(b,a);
    Vector3D cross2=Vector3D::crossProduct(c,b);
    Vector3D cross3=Vector3D::crossProduct(a,c);

    Vector3D collinearCrossing1=Vector3D::crossProduct(cross1,cross2);
    Vector3D collinearCrossing2=Vector3D::crossProduct(cross2,cross3);

    double dot1=Vector3D::dotProduct(cross1.normalized(),cross2.normalized());
    double dot2=Vector3D::dotProduct(cross2.normalized(),cross3.normalized());

    if (collinearCrossing1.length()<0.0000001 &&collinearCrossing2.length()<0.0000001
            && (dot1>0.0||fabs(dot1)<1e-10) && (dot2>0.0||fabs(dot2)<1e-10))
            return true;
    return false;
}

bool FrameCalculations::coDirectionallyCheck(const Vector3D a, const Vector3D b)
{
    if (a.length()<1e-10||b.length()<1e-10)
        return true;
    Vector3D a1=a;
    a1=a1.normalized();
    Vector3D a2=b;
    a2=a2.normalized();
    Vector3D p=a1-a2;
    if (p.length()<1e-10)
        return true;
    return false;
}

void FrameCalculations::calcControlPointsRaised(QVector<Vector3D> &controlPointsRaised, QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals, const double raise)
{
    for (int i=0; i<controlPoints.size();i++)
    {
        controlPointsRaised.push_back(controlPoints[i]+normals[i]*raise);
    }
}

/*!
Функция, добавляющая вортон в вектор. Создана для совместимости с QtConcurrent
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] vort Вортон для добавления
*/
void FrameCalculations::addToVortonsVec(QVector<Vorton> &vortons, const Vorton vort)
{
    vortons.push_back(vort);
}

/*!
Функция расчета скорости и тензора деформации от вектора вортонов в точке.
\param point Точка расчета
\param streamVel Скорость потока
\param vortons Вектор, содержащий вортоны
\return Значение скорости и тензора деформации
*/
VelBsym FrameCalculations::velocityAndBsymm(const Vector3D point,const  Vector3D streamVel, const QVector<Vorton> &vortons)
{
    VelBsym res = VelBsym (streamVel);
    for (int i=0; i<vortons.size(); i++)
        res+=vortons[i].velAndBsym(point);
    return res;
}

VelBsym FrameCalculations::velocityAndBsymmGauss3(const Vector3D point, const Vector3D deltar, const Vector3D streamVel, const QVector<Vorton> &vortons)
{
    VelBsym res = VelBsym (streamVel);
    for (int i=0; i<vortons.size(); i++)
        res+=vortons[i].velAndBsymGauss3(point,deltar);
    return res;
}

/*!
Функция расчета скорости от вектора вортонов и вектора рамок в точке.
\param point Точка расчета
\param streamVel Скорость потока
\param vortons Вектор, содержащий вортоны
\param frames Вектор, содержащий рамки
\return Значение скорости
*/
Vector3D FrameCalculations::velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<std::shared_ptr<MultiFrame>> frames)
{
    Vector3D vel=streamVel;
    for (int i=0; i<vortons.size(); i++)
        vel+=vortons[i].velocity(point);
    for (int i=0; i<frames.size(); i++)
        vel+=frames[i]->velocity(point);
    return vel;
}

/*!
Функция расчета скорости от вектора вортонов  в точке.
\overload
\param point Точка расчета
\param streamVel Скорость потока
\param vortons Вектор, содержащий вортоны
\return Значение скорости
*/
Vector3D FrameCalculations::velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons)
{
    Vector3D vel=streamVel;
    for (int i=0; i<vortons.size(); i++)
        vel+=vortons[i].velocity(point);
    return vel;
}

/*!
Функция расчета перемещения и удлинения вортона. Создана для совместимости с QtConcurrent
\param el Элемент структуры для расчета
\return Вортон с рассчитанным значением перемещения и удлинения
*/
Vorton FrameCalculations::parallelDisplacement(const Parallel el)
{
    Vorton res=el.Vortons->at(el.num);

    VelBsym vb=FrameCalculations::velocityAndBsymm(el.Vortons->at(el.num).getMid(),el.streamVel, *el.Vortons);
     Vector3D selfLen=el.Vortons->at(el.num).getTail()-el.Vortons->at(el.num).getMid();
    double xElong=vb.B[0][0]*selfLen[0]+vb.B[0][1]*selfLen[1]+vb.B[0][2]*selfLen[2];
    double yElong=vb.B[1][0]*selfLen[0]+vb.B[1][1]*selfLen[1]+vb.B[1][2]*selfLen[2];
    double zElong=vb.B[2][0]*selfLen[0]+vb.B[2][1]*selfLen[1]+vb.B[2][2]*selfLen[2];
    res.setElongation(Vector3D(xElong, yElong, zElong)*el.tau);
    res.setMove(vb.Vel*el.tau);
    return res;
}

Vorton FrameCalculations::parallelDisplacementPassive(const ParallelPassive el)
{
    Vorton res=el.centralVort;

    VelBsym vb=FrameCalculations::velocityAndBsymm(res.getMid(),el.streamVel, *el.vortons);
    Vector3D selfLen=res.getTail()-res.getMid();
    double xElong=(vb.B[0][0]*selfLen[0]+vb.B[0][1]*selfLen[1]+vb.B[0][2]*selfLen[2])*el.tau;
    double yElong=(vb.B[1][0]*selfLen[0]+vb.B[1][1]*selfLen[1]+vb.B[1][2]*selfLen[2])*el.tau;
    double zElong=(vb.B[2][0]*selfLen[0]+vb.B[2][1]*selfLen[1]+vb.B[2][2]*selfLen[2])*el.tau;
    res.setElongation(Vector3D(xElong, yElong, zElong));
    res.setMove(vb.Vel*el.tau);
    return res;
}

Vorton FrameCalculations::parallelDisplacementGauss(const Parallel el)
{
    Vorton res=el.Vortons->at(el.num);
    Vector3D selfLen=el.Vortons->at(el.num).getTail()-el.Vortons->at(el.num).getMid();
    VelBsym vb=FrameCalculations::velocityAndBsymmGauss3(el.Vortons->at(el.num).getMid(), selfLen, el.streamVel, *el.Vortons);
    selfLen=el.Vortons->at(el.num).getTail()-el.Vortons->at(el.num).getMid();
    //VelBsym vb=FrameCalculations::velocityAndBsymm(el.Vortons->at(el.num).getMid(),  el.streamVel, *el.Vortons);
    double xElong=vb.B[0][0]*selfLen[0]+vb.B[0][1]*selfLen[1]+vb.B[0][2]*selfLen[2];
    double yElong=vb.B[1][0]*selfLen[0]+vb.B[1][1]*selfLen[1]+vb.B[1][2]*selfLen[2];
    double zElong=vb.B[2][0]*selfLen[0]+vb.B[2][1]*selfLen[1]+vb.B[2][2]*selfLen[2];
    res.setElongation(/*0.5**/Vector3D(xElong, yElong, zElong)*el.tau);
    res.setMove(/*0.5**/vb.Vel*el.tau);
    return res;
}

/*!
Функция для изменения положения вортона согласно рассчитанному перемещению и изменению.
\param[in,out] vortons Вектор вортонов
*/
void FrameCalculations::displace(QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size(); i++)
    {
        vortons[i].setMid(vortons[i].getMid()+vortons[i].getMove());
        vortons[i].setTail(vortons[i].getTail()+vortons[i].getMove()+vortons[i].getElongation());


//       if ((vortons[i].getTail()-vortons[i].getMid()).length()<

    }
}

/*!
Функция для отражения положения вортонов и рамок относительно оси х
\param[in,out] symFreeVortons Вектор вортонов в слое
\param[in,out] symNewVortons Вектор вортонов с рамок
\param[in,out] symFrames Вектор рамок
*/
void FrameCalculations::reflect(QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, QVector<std::shared_ptr<MultiFrame>>& symFrames)
{
    for (int i=0; i<symFreeVortons.size(); i++)
    {
        symFreeVortons[i].setMid(Vector3D(-symFreeVortons[i].getMid().x(),symFreeVortons[i].getMid().y(),symFreeVortons[i].getMid().z()));
        symFreeVortons[i].setTail(Vector3D(-symFreeVortons[i].getTail().x(),symFreeVortons[i].getTail().y(),symFreeVortons[i].getTail().z()));
        symFreeVortons[i].setVorticity(-symFreeVortons[i].getVorticity());
    }

    for (int i=0; i<symNewVortons.size(); i++)
    {
        symNewVortons[i].setMid(Vector3D(-symNewVortons[i].getMid().x(),symNewVortons[i].getMid().y(),symNewVortons[i].getMid().z()));
        symNewVortons[i].setTail(Vector3D(-symNewVortons[i].getTail().x(),symNewVortons[i].getTail().y(),symNewVortons[i].getTail().z()));
        symNewVortons[i].setVorticity(-symNewVortons[i].getVorticity());

    }

    for (int i=0; i<symFrames.size(); i++)
    {
        for (int j=0; j<symFrames[i]->getAnglesNum(); j++)
        {
            symFrames[i]->at(j).setMid(Vector3D(-symFrames[i]->at(j).getMid().x(),symFrames[i]->at(j).getMid().y(),symFrames[i]->at(j).getMid().z()));
            symFrames[i]->at(j).setTail(Vector3D(-symFrames[i]->at(j).getTail().x(),symFrames[i]->at(j).getTail().y(),symFrames[i]->at(j).getTail().z()));
            symFrames[i]->at(j).setVorticity(-symFrames[i]->at(j).getVorticity());
        }
        symFrames[i]->setCenter(Vector3D(-symFrames[i]->getCenter().x(), symFrames[i]->getCenter().y(), symFrames[i]->getCenter().z()));
    }
}

void FrameCalculations::reflectMove(QVector<Vorton> &symFreeVortons, QVector<Vorton> &freeVortons)
{
    for (int i=0; i<symFreeVortons.size(); i++)
    {
        symFreeVortons[i].setMove(Vector3D(-freeVortons[i].getMove().x(),freeVortons[i].getMove().y(),freeVortons[i].getMove().z()));
    }
}

/*!
Функция для проверки попадания вортона внутрь сферы
\param vorton Вортон
\param center Центра сферы
\param radius Радиус сферы
\return Определение попадания внутрь сферы
*/
bool FrameCalculations::insideSphere(const Vorton& vort, const Vector3D& center, const double radius)
{
    if ((vort.getMid()-center).length()<radius)
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг сферы
\param vorton Вортон
\param center Центр сферы
\param radius Радиус сферы
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг сферы
*/
bool FrameCalculations::insideSphereLayer(const Vorton& vort, const Vector3D& center, const double radius, const double layerHeight)
{
    if ((vort.getMid()-center).length()<radius+layerHeight)
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь цилиндра
\param vorton Вортон
\param height Высота цилиндра
\param diameter Диаметр цилиндра
\return Определение попадания внутрь цилиндра
*/
bool FrameCalculations::insideCylinder(const Vorton &vort, const double height, const double diameter)
{
    if ((vort.getMid().y()>0)&&(vort.getMid().y()<height)&&((pow(vort.getMid().x(),2)+pow(vort.getMid().z(),2))<0.25*pow(diameter,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг цилиндра
\param vorton Вортон
\param height Высота цилиндра
\param diameter Диаметр цилиндра
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг цилиндра
*/
bool FrameCalculations::insideCylinderLayer(const Vorton &vort, const double height, const double diameter, const double layerHeight)
{
    if ((vort.getMid().y()>-layerHeight)&&(vort.getMid().y()<height+layerHeight)&&((pow(vort.getMid().x(),2)+pow(vort.getMid().z(),2))<pow(0.5*diameter+layerHeight,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь тела вращения
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\return Определение попадания внутрь тела вращения
*/
bool FrameCalculations::insideRotationBody(const Vorton &vort, const Vector3D bodyNose, const double xEnd, FormingParameters forming)
{
    if((vort.getMid().x()>=bodyNose.x()) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getMid().x()-bodyNose.x(),forming),2)))
        return true;
//    if((vort.getTail().x()>=bodyNose.x()) && (vort.getTail().x()<=xEnd) && ((pow(vort.getTail().z()-bodyNose.z(),2)+pow(vort.getTail().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getTail().x()-bodyNose.x(),forming),2)))
//        return true;
//    if(((2.0*vort.getMid()-vort.getTail()).x()>=bodyNose.x()) && ((2.0*vort.getMid()-vort.getTail()).x()<=xEnd) && ((pow((2.0*vort.getMid()-vort.getTail()).z()-bodyNose.z(),2)+pow((2.0*vort.getMid()-vort.getTail()).y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF((2.0*vort.getMid()-vort.getTail()).x()-bodyNose.x(),forming),2)))
//        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг тела вращения
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг тела вращения
*/
bool FrameCalculations::insideRotationBodyLayer(const Vorton &vort, const Vector3D bodyNose, const double xEnd, const double layerHeight, FormingParameters forming)
{
    if((vort.getMid().x()>=bodyNose.x()-layerHeight) && (vort.getMid().x()<=xEnd+layerHeight) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getMid().x()-bodyNose.x(),forming)+layerHeight,2)))
        return true;
//    if((vort.getTail().x()>=bodyNose.x()-layerHeight) && (vort.getTail().x()<=xEnd+layerHeight) && ((pow(vort.getTail().z()-bodyNose.z(),2)+pow(vort.getTail().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getTail().x()-bodyNose.x(),forming)+layerHeight,2)))
//        return true;
//    if(((2.0*vort.getMid()-vort.getTail()).x()>=bodyNose.x()-layerHeight) && ((2.0*vort.getMid()-vort.getTail()).x()<=xEnd+layerHeight) && ((pow((2.0*vort.getMid()-vort.getTail()).z()-bodyNose.z(),2)+pow((2.0*vort.getMid()-vort.getTail()).y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF((2.0*vort.getMid()-vort.getTail()).x()-bodyNose.x(),forming)+layerHeight,2)))
//        return true;
    return false;
}

int FrameCalculations::insideRotationBodyv2(const Vorton &vort, const Vector3D bodyNose, const double xEnd, FormingParameters forming)
{
    if((vort.getTail().x()>=bodyNose.x()) && (vort.getTail().x()<=xEnd) && ((pow(vort.getTail().z()-bodyNose.z(),2)+pow(vort.getTail().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getTail().x()-bodyNose.x(),forming),2)))
        return 1;
    if(((2.0*vort.getMid()-vort.getTail()).x()>=bodyNose.x()) && ((2.0*vort.getMid()-vort.getTail()).x()<=xEnd) && ((pow((2.0*vort.getMid()-vort.getTail()).z()-bodyNose.z(),2)+pow((2.0*vort.getMid()-vort.getTail()).y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF((2.0*vort.getMid()-vort.getTail()).x()-bodyNose.x(),forming),2)))
        return 0;
    return -1;
}

int FrameCalculations::insideRotationBodyLayerv2(const Vorton &vort, const Vector3D bodyNose, const double xEnd, const double layerHeight, FormingParameters forming)
{

    if((vort.getTail().x()>=bodyNose.x()-layerHeight) && (vort.getTail().x()<=xEnd+layerHeight) && ((pow(vort.getTail().z()-bodyNose.z(),2)+pow(vort.getTail().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getTail().x()-bodyNose.x(),forming)+layerHeight,2)))
        return 1;
    if(((2.0*vort.getMid()-vort.getTail()).x()>=bodyNose.x()-layerHeight) && ((2.0*vort.getMid()-vort.getTail()).x()<=xEnd+layerHeight) && ((pow((2.0*vort.getMid()-vort.getTail()).z()-bodyNose.z(),2)+pow((2.0*vort.getMid()-vort.getTail()).y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF((2.0*vort.getMid()-vort.getTail()).x()-bodyNose.x(),forming)+layerHeight,2)))
        return 0;
    return -1;
}

/*!
Функция для проверки попадания вортона внутрь тела вращения со срезом дна
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\return Определение попадания внутрь тела вращения со срезом дна
*/
bool FrameCalculations::insideRotationCutBody(const Vorton &vort, const double xBeg, const double xEnd, FormingParametersRBC forming)
{
    if((vort.getMid().x()>=xBeg) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z(),2)+pow(vort.getMid().y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()-xBeg,forming),2)))
        return true;
    return false;
}

bool FrameCalculations::insideRotationCutBody(const Vorton &vort,const double xEnd, const Vector3D bodyNose, FormingParametersRBC forming)
{
    if((vort.getMid().x()>=bodyNose.x()) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()-bodyNose.x(),forming),2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг тела вращения со срезом дна
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг тела вращения со срезом дна
*/
bool FrameCalculations::insideRotationCutBodyLayer(const Vorton &vort, const double xEnd, const double layerHeight, const Vector3D bodyNose, FormingParametersRBC forming)
{
    if((vort.getMid().x()>=bodyNose.x()-layerHeight) && (vort.getMid().x()<=xEnd+layerHeight) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()-bodyNose.x(),forming)+layerHeight,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона под экран
\param vorton Вортон
\return Определение вортона под экран
*/
bool FrameCalculations::insideScreen(Vorton &vort,Vector3D translScreen)
{
    if ((2.0*vort.getMid().x()-vort.getTail().x())>translScreen.x() || vort.getTail().x()>translScreen.x())
        return true;
    return false;
}

/*!
Функция для проверки возможного "взрыва" вортонов
\param vortons Вектор вортонов
\return Определение вортона под экран
*/
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

/*!
Функция для расчета дисперсии вектора С
\param cAerodynamics Вектор С
\return Значение дисперсии
*/
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
        dispersion+=(cAerodynamics[i]-cAver).lengthSquared()/(cAerodynamics.size()-100);
    }
    dispersion=sqrt(dispersion)/cAver.length();
    return dispersion;
}

/*!
Функция для копирования вектора рамок
\param frames Вектор рамок
\return Скопированный вектор рамок
*/
QVector<std::shared_ptr<MultiFrame> > FrameCalculations::copyFrames(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<std::shared_ptr<MultiFrame>> copyingFrames;
    for (int i=0; i<frames.size(); i++)
    {
        copyingFrames.push_back(std::make_shared<MultiFrame>(*frames[i].get()));
    }
    return copyingFrames;
}

/*!
Функция для перемещения тела при решении задачи старта
\param[in] translation Вектор переноса
\param[in,out] frames Вектор рамок
\param[in,out] controlPoints Вектор контрольных точек
\param[in,out] controlPointsRaised Вектор контрольных точек для расчета давлений
\param[in,out] center Центр тела
\param[in] xBeg Минимальная координата х тела
\param[in] xEnd Максимальная координата х тела
\param[in] fragPar Параметры разбиения
*/
void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>> &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised, Vector3D &center, Vector3D &bodyNose, double &xend, const FragmentationParameters &fragPar)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }

   //if (xbeg!=fragPar.rotationBodyXBeg)
   bodyNose+=translation;
    center+=translation;
//   if (xend-bodyNose.x()>fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)
        xend+=translation.x();
//   if (xend!=0.0)
//       qDebug()<<"Stop";
}

/*!
Функция для перемещения тела.
\overload
\param[in] translation Вектор переноса
\param[in,out] frames Вектор рамок
\param[in,out] controlPoints Вектор контрольных точек
\param[in,out] controlPointsRaised Вектор контрольных точек для расчета давлений
\param[in,out] center Центр тела
*/
void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>> &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised, Vector3D &center)
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

void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }
}

void FrameCalculations::updateBoundaries(Vector3D &bodynose, Vector3D &translation, Vector3D &center, QVector<Vorton>& freeVortons)
{
    bodynose+=translation;
    qDebug()<<"bodyNose: "<<bodynose.x();
    center=bodynose/2;
    for (int i=0; i<freeVortons.size();i++)
    {
        freeVortons[i].setMid(freeVortons[i].getMid()+translation);
        freeVortons[i].setTail(freeVortons[i].getTail()+translation);
    }
}

/*!
Функция для перемещения вектора вортонов
\param[in] translation Вектор переноса
\param[in,out] vortons Вектор вортонов
*/
void FrameCalculations::translateVortons(const Vector3D &translation, QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size(); i++)
        vortons[i].translate(translation);
}

QPair<int,int> FrameCalculations::findMaxSolidAngle(Vector3D point, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    QPair<int,int> closest;
    double maxAngle=frames[0]->getTriangle(0).solidAngle(point);
    closest.first=0;
    closest.second=0;

    for (int i=0; i<frames.size();i++)
    {
        for (int j=0; j<frames[i]->getAnglesNum()-2;j++)
        if (frames[i]->getTriangle(j).solidAngle(point)>maxAngle)
        {
            maxAngle=frames[i]->getTriangle(j).solidAngle(point);
            closest.first=i;
            closest.second=j;
        }
    }
    return closest;
}

QPair<int, int> FrameCalculations::findMaxSolidAngle(Vorton &vort, QVector<std::shared_ptr<MultiFrame> > &frames)
{
    QPair<int,int> closest;
    double maxAngle=frames[0]->getTriangle(0).solidAngle(vort.getMid())+frames[0]->getTriangle(0).solidAngle(vort.getTail())+frames[0]->getTriangle(0).solidAngle(vort.getBeggining());
    closest.first=0;
    closest.second=0;

    for (int i=0; i<frames.size();i++)
    {
        for (int j=0; j<frames[i]->getTrianglesNum();j++)
        {
        double solid=frames[i]->getTriangle(j).solidAngle(vort.getMid())+frames[i]->getTriangle(j).solidAngle(vort.getTail())+frames[i]->getTriangle(j).solidAngle(vort.getBeggining());
        if (solid>maxAngle)
        {
            maxAngle=solid;
            closest.first=i;
            closest.second=j;
        }
        }
    }
    return closest;
}

/*!
Функция возвращающая значения счетчиков
\return Структура с текущими значениями счетчиков
*/
Counters FrameCalculations::getCounters() const
{
    return counters;
}

/*!
Функция возвращающая значения сработавших ограничений
\return Структура с текущими значениями сработавших ограничений
*/
Restrictions FrameCalculations::getRestrictions() const
{
    return restrictions;
}

/*!
Функция возвращающая значения таймеров
\return Структура с текущими значениями таймеров
*/
Timers FrameCalculations::getTimers() const
{
    return timers;
}

/*!
Функция обнуляющая значения счетчиков
*/
void FrameCalculations::clearCounters()
{
    counters.clear();
}

int FrameCalculations::getMatrixSize()
{
    return matrixSize;
}

/*!
Функция обнуляющая значения сработавших ограничений
*/
void FrameCalculations::clearRestrictions()
{
    restrictions.clear();
}

/*!
Функция обнуляющая значения сработавших таймеров
*/
void FrameCalculations::clearTimers()
{
    timers.clear();
}

/*!
Функция обнуляющая значения сработавших счетчиков, сработавших ограничений, таймеров
*/
void FrameCalculations::clear()
{
    clearTimers();
    clearRestrictions();
    clearCounters();
}

bool FrameCalculations::checkDirection(TriangleFrame &a, TriangleFrame &b)
{
    if (a.getR0()==b.getR0() && a.getR2()==b.getR2())
        return true;
    if (a.getR0()==b.getR1() && a.getR2()==b.getR0())
        return true;
    if (a.getR0()==b.getR2() && b.getR2()==b.getR1())
        return true;

    if (a.getR0()==b.getR0() && a.getR1()==b.getR1())
        return true;
    if (a.getR0()==b.getR1() && a.getR1()==b.getR2())
        return true;
    if (a.getR0()==b.getR2() && a.getR1()==b.getR0())
        return true;

    if (a.getR1()==b.getR0() && a.getR2()==b.getR1())
        return true;
    if (a.getR1()==b.getR1() && a.getR2()==b.getR2())
        return true;
    if (a.getR1()==b.getR2() && a.getR2()==b.getR0())
        return  true;

    return false;
}

void FrameCalculations::correctDirectionFrames(QVector<TriangleFrame> &frames)
{
    for (int i=0; i<frames.size()-1;i++)
    {
        for (int j=i+1;j<frames.size();j++)
        {
             if(checkDirection(frames[i],frames[j]))
             {
                 Vector3D middle=frames[j].getR0();
                 frames[j].setR0(frames[j].getR2());
                 frames[j].setR2(middle);
                 frames[j].setNormal(-1.0*frames[j].getNormal());
             }
        }
    }
}

QVector<std::pair<double,double>> FrameCalculations::makeParalllepiped(QVector<Vorton> newVortons)
{
    QVector<std::pair<double,double>> boundaries(3);




    std::pair<Vorton*,Vorton*> xBoundariesIter = std::minmax_element(newVortons.begin(),newVortons.end(),xCompare);
    std::pair<double, double> xBoundaries(xBoundariesIter.first->getTail().x(),xBoundariesIter.second->getTail().x());
    boundaries[0]=xBoundaries;
    std::pair<Vorton*,Vorton*> yBoundariesIter = std::minmax_element(newVortons.begin(),newVortons.end(),yCompare);
    std::pair<double, double> yBoundaries(yBoundariesIter.first->getTail().y(),yBoundariesIter.second->getTail().y());
    boundaries[1]=yBoundaries;
    std::pair<Vorton*,Vorton*> zBoundariesIter = std::minmax_element(newVortons.begin(),newVortons.end(),zCompare);
    std::pair<double, double> zBoundaries(zBoundariesIter.first->getTail().z(),zBoundariesIter.second->getTail().z());
    boundaries[2]=zBoundaries;
    return boundaries;

}

QVector<std::pair<double, double> > FrameCalculations::makeParallepiped(QVector<std::shared_ptr<MultiFrame> > &frames)
{
    QVector<std::pair<double,double>> boundaries(3);
    std::pair<double,double> x=std::pair<double,double>(frames[0]->at(0).getMid().x(),frames[0]->at(0).getMid().x());
    std::pair<double,double> y=std::pair<double,double>(frames[0]->at(0).getMid().y(),frames[0]->at(0).getMid().y());
    std::pair<double,double> z=std::pair<double,double>(frames[0]->at(0).getMid().z(),frames[0]->at(0).getMid().z());
    for (int i=0; i<frames.size();i++)
    {
        for (int j=0; j<frames[i]->getAnglesNum();j++)
        {
            if (frames[i]->at(j).getMid().x()<x.first)
                x.first=frames[i]->at(j).getMid().x();
            if (frames[i]->at(j).getMid().x()>x.second)
                x.second=frames[i]->at(j).getMid().x();
            if (frames[i]->at(j).getTail().x()<x.first)
                x.first=frames[i]->at(j).getTail().x();
            if (frames[i]->at(j).getTail().x()>x.second)
                x.second=frames[i]->at(j).getTail().x();

            if (frames[i]->at(j).getMid().y()<y.first)
                y.first=frames[i]->at(j).getMid().y();
            if (frames[i]->at(j).getMid().y()>y.second)
                y.second=frames[i]->at(j).getMid().y();
            if (frames[i]->at(j).getTail().y()<y.first)
                y.first=frames[i]->at(j).getTail().y();
            if (frames[i]->at(j).getTail().y()>y.second)
                y.second=frames[i]->at(j).getTail().y();

            if (frames[i]->at(j).getMid().z()<z.first)
                z.first=frames[i]->at(j).getMid().z();
            if (frames[i]->at(j).getMid().z()>z.second)
                z.second=frames[i]->at(j).getMid().z();
            if (frames[i]->at(j).getTail().z()<z.first)
                z.first=frames[i]->at(j).getTail().z();
            if (frames[i]->at(j).getTail().z()>z.second)
                z.second=frames[i]->at(j).getTail().z();
        }
    }
    boundaries[0]=x;
    boundaries[1]=y;
    boundaries[2]=z;
    return boundaries;
}

bool FrameCalculations::xCompare(const Vorton a,const Vorton b)
{
    return (a.getTail().x() < b.getTail().x());
}

bool FrameCalculations::yCompare(Vorton a, Vorton b)
{
    return (a.getTail().y() < b.getTail().y());
}

bool FrameCalculations::zCompare(Vorton a, Vorton b)
{
    return (a.getTail().z() < b.getTail().z());
}





Eigen::Vector3d toEigenVector(Vector3D vec)
{
    Eigen::Vector3d eigenVec(vec.x(),vec.y(),vec.z());
    return eigenVec;
}

Vector3D fromEigenVector(Eigen::Vector3d vec)
{
    Vector3D vec3d(vec.x(),vec.y(),vec.z());
    return vec3d;
}

