#include "solver.h"

bool Solver::interrupted=false;
bool Solver::explosion=false;
bool Solver::getBackGA=false;
/*!
Создает экземпляр класса с нулевыми параметрами
*/

bool Solver::checkFinishing()
{
    if (interrupted)
    {
        emit finishSolver();
        return true;
    }
    return false;
}

Solver::Solver()
{

}

void Solver::setMotionType(const MotionType type)
{
    motion=type;
}

/*!
Создает экземпляр класса для расчета неподвижного тела с заданными параметрами
\param parameters Параметры расчета
*/
Solver::Solver(const SolverParameters& parameters)
{
    solvPar=parameters;
    cAerodynamics.resize(solvPar.stepsNum);
    forces.resize(solvPar.stepsNum);
    torques.resize(solvPar.stepsNum);
}

/*!
Создает экземпляр класса для расчета движущегося тела с заданными параметрами
\param parameters Параметры расчета
\param motionParameters Параметры свободного движения
*/
Solver::Solver(const SolverParameters &parameters, const FreeMotionParameters &motionParameters)
{
    solvPar=parameters;
    freeMotionPar=motionParameters;
    cAerodynamics.resize(solvPar.stepsNum);
    forces.resize(solvPar.stepsNum);
    torques.resize(solvPar.stepsNum);
}

/*!
Осуществляет расчет неподвижной сферы с заданными параметрами
\param fragPar Параметры разбиения сферы
*/
void Solver::sphereSolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Solver::explosion=false;
    Logger* logger=createLog(SPHERE);
    BodyFragmentation fragmentation(BodyType::SPHERE, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    emit sendNormalsVis(controlPoints,normals);
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    Vector3D center;
    QVector<double> tetas=functions.calcTetas(fragPar.sphereTetaFragNum);
    QVector<double> cp(fragPar.sphereTetaFragNum+3);
    emit updateSphereMaximum(solvPar.stepsNum-1);

    logger->writePassport(solvPar,fragPar);

    QVector<double> pressures;
    QVector<Vector3D> velocities;
    QVector<double> tangentialVelocities;
    QVector<double> normalVelocitiesBeforeIntegr;
    QVector<double> normalVelocitiesAfterIntegr;
    QVector<double> normalVelocitiesCenterIntegr;
    QVector<double> normalVelocitiesDeltaIntegr;
    QVector<double> normalVelocitiesEndIntegr;

    QVector<Vector3D> centerSection;

     QVector<std::shared_ptr<MultiFrame>> framesSection;

    pressures.resize(controlPointsRaised.size()+centerSection.size());
    velocities.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesBeforeIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesEndIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesCenterIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesDeltaIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesAfterIntegr.resize(controlPoints.size()+centerSection.size());
    tangentialVelocities.resize(controlPoints.size()+centerSection.size());
    QVector<double> oldGammas(functions.getMatrixSize());
    Vector3D currentSpeed;
    if (motion==NOACCELERATE)
        currentSpeed=solvPar.streamVel;
    else
        currentSpeed=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);

    functions.epsZero(frames);
    for (int i=0; i<solvPar.stepsNum; i++)
    {

        QTime stepTime=QTime::currentTime();
        if(checkFinishing())
            return;
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(currentSpeed,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);

        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesAfterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames),normals[i]);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);



        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,currentSpeed,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(currentSpeed, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        functions.cpSum(i,solvPar.stepsNum, cp, fragPar.sphereFiFragNum, fragPar.sphereRad, fragPar.pointsRaising, tetas,solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, center);
        cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));
        forces[i]=force;

//        Vector3D force;
//        if (i!=0)
//        {
//        force=functions.forceSetuha(currentSpeed, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals,oldGammas);

//        forces[i]=force;
//        cAerodynamics[i] = force*2.0/(solvPar.density*solvPar.streamVel.lengthSquared()*M_PI*pow(fragPar.sphereRad,2));
//        }
//        else {
//            force=Vector3D(100000,0.0,0.0);
//            cAerodynamics[i]=force;
//            forces[i]=force;
//        }
//        for (int i=0;i<vorticities.size();i++)
//            oldGammas[i]=vorticities(i);
        for (int i=0; i<controlPointsRaised.size(); i++)
            pressures[i]=(FrameCalculations::pressureCalc(controlPointsRaised[i], solvPar.streamVel, solvPar.streamPres,solvPar.density, frames, freeVortons, solvPar.tau)-solvPar.streamPres)/
                    (solvPar.density*Vector3D::dotProduct(solvPar.streamVel,solvPar.streamVel)*0.5);

        functions.epsNormal(newVortons,fragPar.vortonsRad);
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesBeforeIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        freeVortons.append(newVortons);



        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();
        QVector<Vorton> oldVortons=freeVortons;
        functions.displace(freeVortons);
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesCenterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        functions.getBackAndRotate(Solver::getBackGA,freeVortons,center,fragPar.sphereRad,solvPar.layerHeight,controlPoints,normals,oldVortons,frames);
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesDeltaIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);

        //functions.getBackAndRotateSphere(freeVortons, center,fragPar.sphereRad,solvPar.layerHeight, controlPoints, normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarSphere(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesEndIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        QPair<int,int> boundaries=fragmentation.getStreamLinesSizes();
        double stepStr=0.2;
        QVector<Vector3D> velocitiesStream;
        functions.velForStreamLines(velocitiesStream,solvPar.streamVel,stepStr,freeVortons,boundaries);
        logger->createParaviewStreamlinesFile(velocitiesStream,boundaries,stepStr,i);
        logger->createParaviewTraceVerticesFile(freeVortons,i);
        logger->createParaviewFile(frames,pressures,velocities,tangentialVelocities,normalVelocitiesAfterIntegr,normalVelocitiesBeforeIntegr,normalVelocitiesCenterIntegr, normalVelocitiesDeltaIntegr,normalVelocitiesEndIntegr,framesSection,i);

        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());
        logger->writeVortons(frames,freeVortons,i);
        emit sendProgressSphere(i);
        emit repaintGUI(freeVortons, frames);
        if (motion==ACCELERATED && currentSpeed.length()<=solvPar.streamVel.length())
        {
            if ((currentSpeed+solvPar.streamVel/(solvPar.acceleratedStepsNum+2)).length()<solvPar.streamVel.length())
                currentSpeed+=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
            else
                currentSpeed=solvPar.streamVel;
        }
        qDebug()<<currentSpeed.x()<<" "<<currentSpeed.y()<<" "<<currentSpeed.z()<<"\n";
//        if (FrameCalculations::exploseSphere(freeVortons))
//        {
//            qDebug()<<"Explose";
//            Solver::explosion=true;
//            return;
//        }
    }
    for (int i=0; i<tetas.size();i++)
        tetas[i]=M_PI-tetas[i];
    functions.cpAverage(cp,solvPar.stepsNum);
    logger->writeCpFile(cp,tetas);
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;

}

/*!
Осуществляет расчет движущейся сферы с заданными параметрами
\param fragPar Параметры разбиения сферы
*/
void Solver::sphereFreeMotionSolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Solver::explosion=false;
    Logger* logger=createLog(SPHERE);
    BodyFragmentation fragmentation(BodyType::SPHERE, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;
    Vector3D center;
    Vector3D translation=freeMotionPar.bodyVel*solvPar.tau;
    QVector<double> tetas=functions.calcTetas(fragPar.sphereTetaFragNum);
    QVector<double> cp(fragPar.sphereTetaFragNum+3);
    emit updateSphereMaximum(solvPar.stepsNum-1);

    logger->writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
            return;
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,relVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(relVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        cAerodynamics[i] = force/(solvPar.density*relVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));
        forces[i]=force;
        freeVortons.append(newVortons);

        functions.cpSum(i, solvPar.stepsNum,cp, fragPar.sphereFiFragNum, fragPar.sphereRad, fragPar.pointsRaising, tetas,solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, center);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateSphere(freeVortons, center,fragPar.sphereRad,solvPar.layerHeight, controlPoints, normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarSphere(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();
        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center/*, fragPar.rotationBodyXBeg, fragPar.rotationBodyXEnd*/);
        FrameCalculations::translateVortons(translation, freeVortons);
        emit sendNormalsVis(controlPoints,normals);
        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());
        emit sendProgressSphere(i);
        emit repaintGUI(freeVortons, frames);

    }
    functions.cpAverage(cp,solvPar.stepsNum);
    logger->writeCpFile(cp,tetas);
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;
}

/*!
Осуществляет расчет неподвижного цилиндра с заданными параметрами
\param fragPar Параметры разбиения цилиндра
*/
void Solver::cylinderSolver(const FragmentationParameters& fragPar)
{
    QTime start=QTime::currentTime();
    Logger* logger=createLog(CYLINDER);
    BodyFragmentation fragmentation(BodyType::CYLINDER, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    emit sendNormalsVis(controlPoints,normals);
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();
    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    QVector<double> cp(fragPar.cylinderFiFragNum);
    Vector3D center (0.0,fragPar.cylinderHeight*0.5,0.0);

    emit updateCylinderMaximum(solvPar.stepsNum-1);

    logger->writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
            return;
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(solvPar.streamVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        cAerodynamics[i].x(2.0*force.x()/(solvPar.density*solvPar.streamVel.lengthSquared()*M_PI*pow(fragPar.cylinderDiameter/2,2)));
        cAerodynamics[i].y(2.0*force.y()/(solvPar.density*solvPar.streamVel.lengthSquared()*fragPar.cylinderDiameter*fragPar.cylinderHeight));
        cAerodynamics[i].z(2.0*force.z()/(solvPar.density*solvPar.streamVel.lengthSquared()*fragPar.cylinderDiameter*fragPar.cylinderHeight));
        forces[i]=force;
        functions.cpSumCylinder(i,solvPar.stepsNum, cp, fragPar.cylinderFiFragNum, fragPar.cylinderDiameter,fragPar.cylinderHeight, fragPar.pointsRaising,solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau);
        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateCylinder(freeVortons, fragPar.cylinderHeight, fragPar.cylinderDiameter, solvPar.layerHeight, controlPoints, normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarCylinder(freeVortons,solvPar.farDistance,fragPar.cylinderHeight);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());

        emit sendProgressCylinder(i);
        emit repaintGUI(freeVortons, frames);


    }
    QVector<double> fis;
    double fi0=2*M_PI/fragPar.cylinderFiFragNum;
    for (int i=0; i<fragPar.cylinderFiFragNum;i++)
        fis.push_back(fi0*(i+0.5));
    functions.cpAverage(cp,solvPar.stepsNum);
    logger->writeCpFile(cp,fis);
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;
}

/*!
Осуществляет расчет неподвижного тела вращения с заданными параметрами
\param fragPar Параметры разбиения тела вращения
*/
void Solver::rotationBodySolver(const FragmentationParameters &fragPar)
{

    QTime start=QTime::currentTime();
    Logger* logger=createLog(ROTATIONBODY);
    BodyFragmentation fragmentation(BodyType::ROTATIONBODY, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    emit sendNormalsVis(controlPoints,normals);
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    Vector3D center((fragPar.rotationBodyXBeg+fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength+fragmentation.getForming().diameter*0.5/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);
    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg,0.0,0.0);
    double xend;
    Vector3D currentSpeed;
    if (motion==NOACCELERATE)
        currentSpeed=solvPar.streamVel;
    else
        currentSpeed=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
    FormingParameters forming=fragmentation.getForming();
    switch (forming.typeNum)
    {
    case 0:
    {
    center=Vector3D((fragPar.rotationBodyXBeg+forming.sectorOneLength+forming.sectorTwoLength+forming.diameter*0.5/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);
    xend=fragPar.rotationBodyXBeg+forming.sectorOneLength+forming.sectorTwoLength+forming.diameter*0.5;
    break;
    }
    case 1:
    {
    center=Vector3D((fragPar.rotationBodyXBeg+forming.sectorOneLength)*0.5,0.0,0.0);
    xend=fragPar.rotationBodyXBeg+forming.sectorOneLength;
    break;
    }
    case 2:
    {
    center=Vector3D((fragPar.rotationBodyXBeg+forming.sectorOneLength)*0.5,0.0,0.0);
    xend=fragPar.rotationBodyXBeg+forming.sectorOneLength;
    break;
    }
    };

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    QVector<Vorton> oldVortons;
    QVector<double> cp(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp0(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp90(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp180(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp270(fragPar.rotationBodyPartFragNum+1);
    emit updateRotationBodyMaximum(solvPar.stepsNum-1);
    QVector<double> pressures;
    QVector<Vector3D> velocities;
    QVector<double> tangentialVelocities;
    QVector<double> normalVelocitiesBeforeIntegr;
    QVector<double> normalVelocitiesAfterIntegr;
    QVector<double> normalVelocitiesCenterIntegr;
    QVector<double> normalVelocitiesDeltaIntegr;
    QVector<double> normalVelocitiesEndIntegr;

    QVector<std::shared_ptr<MultiFrame>> framesSection;
    QVector<Vector3D> centerSection;
    QVector<Vector3D> sectionNormals;

    //logger->createParaviewStreamlinesFile(velocities,0);
    //functions.calcSection(fragmentation.getForming().diameter,center.x(),fragPar.rotationBodyFiFragNum,framesSection,centerSection,sectionNormals);

    pressures.resize(controlPointsRaised.size()+centerSection.size());
    velocities.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesBeforeIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesAfterIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesCenterIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesDeltaIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesEndIntegr.resize(controlPoints.size()+centerSection.size());
    tangentialVelocities.resize(controlPoints.size()+centerSection.size());

    logger->writePassport(solvPar,fragPar,fragmentation.getForming(),functions.calcFrameSizes(frames));
    QVector<Vector3D> sectionVelocities(centerSection.size());
    QVector<double> oldGammas(functions.getMatrixSize());
    functions.epsZero(frames);
    QVector<std::shared_ptr<MultiFrame>> zFrames;
    QVector<std::shared_ptr<MultiFrame>> yFrames;
    QVector<std::shared_ptr<MultiFrame>> xFrames;
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
            return;
        QTime stepTime=QTime::currentTime();
        newVortons.clear();

        Eigen::VectorXd column=functions.columnCalc(currentSpeed,freeVortons,normals,controlPoints);

        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);

        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);

//        Vector3D force;
//        if (i!=0)
//        {
//        force=functions.forceSetuha(currentSpeed, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals,oldGammas);

//        forces[i]=force;
//        cAerodynamics[i] = force*2.0/(solvPar.density*solvPar.streamVel.lengthSquared()*M_PI*pow(fragmentation.getForming().diameter*0.5,2));
//        }
//        else {
//            force=Vector3D(100000,0.0,0.0);
//            cAerodynamics[i]=force;
//            forces[i]=force;
//        }
//        for (int i=0;i<vorticities.size();i++)
//            oldGammas[i]=vorticities(i);


        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesAfterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames),normals[i]);
        //for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
        //    normalVelocitiesAfterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(centerSection[i-controlPoints.size()],solvPar.streamVel,freeVortons,frames),sectionNormals[i-controlPoints.size()]);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        QVector<std::pair<double, double>> boundariesParallelipeped=functions.makeParalllepiped(newVortons);


        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);





        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);
//        QVector<Vorton> tempV1;
//        QVector<Vorton> tempV2;
//        tempV1=freeVortons;
//        tempV2=newVortons;

        //ЭЙЛЕР
        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,currentSpeed,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);

        //ЭЙЛЕР 2-го порядка
//        functions.displacementCalcGauss3(tempV1,tempV2,solvPar.tau,currentSpeed,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
//        for (int i=0; i<freeVortons.size(); i++) {
//            freeVortons[i].setMove(tempV1[i].getMove());
//            freeVortons[i].setElongation(tempV1[i].getElongation());
//        }
//        for (int i=0; i<newVortons.size(); i++) {
//            newVortons[i].setMove(tempV2[i].getMove());
//            newVortons[i].setElongation(tempV2[i].getElongation());
//        }
//        functions.displace(tempV1);
//        functions.displace(tempV2);

//        functions.displacementCalcGauss3(tempV1,tempV2,solvPar.tau,currentSpeed,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
//        for (int i=0; i<freeVortons.size(); i++) {
//            freeVortons[i].setMove(tempV1[i].getMove());
//            freeVortons[i].setElongation(tempV1[i].getElongation());
//        }
//        for (int i=0; i<newVortons.size(); i++) {
//            newVortons[i].setMove(tempV2[i].getMove());
//            newVortons[i].setElongation(tempV2[i].getElongation());
//        }
        //

        QVector<Vorton> copyVort=freeVortons;
        functions.displace(copyVort);
        functions.displace(newVortons);
        //functions.getBackRotationBody(copyVort,bodyNose,xend,controlPoints,normals,forming);
        //functions.getBackRotationBody(newVortons,bodyNose,xend,controlPoints,normals,forming);
        //for (int i=0; i<freeVortons.size();i++)
        //    freeVortons[i].setMove(copyVort[i].getMid()-freeVortons[i].getMid());

        Vector3D force=functions.forceCalc(currentSpeed, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        forces[i]=force;
        cAerodynamics[i] = force*2.0/(solvPar.density*solvPar.streamVel.lengthSquared()*M_PI*pow(fragmentation.getForming().diameter*0.5,2));


        for (int i=0; i<controlPointsRaised.size(); i++)
            pressures[i]=(FrameCalculations::pressureCalc(controlPointsRaised[i], solvPar.streamVel, solvPar.streamPres,solvPar.density, frames, freeVortons, solvPar.tau)-solvPar.streamPres)/
                    (solvPar.density*Vector3D::dotProduct(solvPar.streamVel,solvPar.streamVel)*0.5);
        for (int i=controlPointsRaised.size(); i<controlPointsRaised.size()+centerSection.size(); i++)
            pressures[i]=(FrameCalculations::pressureCalc(centerSection[i-controlPoints.size()], solvPar.streamVel, solvPar.streamPres,solvPar.density, frames, freeVortons, solvPar.tau)-solvPar.streamPres)/
                    (solvPar.density*Vector3D::dotProduct(solvPar.streamVel,solvPar.streamVel)*0.5);





        for (int i=0; i<controlPoints.size();i++)
            velocities[i]=FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames);
        //for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
        //    velocities[i]=FrameCalculations::velocity(centerSection[i-controlPoints.size()],solvPar.streamVel,freeVortons,frames);



        //for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
        //    normalVelocitiesBeforeIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(centerSection[i-controlPoints.size()],solvPar.streamVel,freeVortons),sectionNormals[i-controlPoints.size()]);

        for (int i=0; i<controlPoints.size();i++)
            tangentialVelocities[i]=(velocities[i]-normalVelocitiesBeforeIntegr[i]*normals[i]).length();
        //for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
        //    tangentialVelocities[i]=(velocities[i]-normalVelocitiesBeforeIntegr[i]*sectionNormals[i-controlPoints.size()]).length();



        functions.cpSumRotationBody(i,solvPar.stepsNum, cp, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp0, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,0);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp90, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,90);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp180, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,180);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp270, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,270);
        functions.epsNormal(newVortons,fragPar.vortonsRad);
        //functions.epsNormal(frames,fragPar.vortonsRad);

        freeVortons=copyVort+newVortons;

        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesBeforeIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);

        oldVortons=freeVortons;

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();


        //functions.displace(freeVortons);

        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesCenterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        functions.universalGetBack(freeVortons,boundariesParallelipeped,solvPar.layerHeight,controlPoints,normals,frames);
        //functions.getBackAndRotate(Solver::getBackGA,freeVortons,bodyNose,xend,solvPar.layerHeight,controlPoints,normals,oldVortons,frames,forming);
        //functions.rotateRotationBody(freeVortons,bodyNose,xend,solvPar.layerHeight,controlPoints,normals,forming);
        //functions.getBackAndRotateRotationBody(freeVortons, bodyNose, xend, solvPar.layerHeight, controlPoints,normals,fragmentation.getForming());
        //functions.getBackAndRotateRotationBodyGA(freeVortons,oldVortons,frames,xend,bodyNose,forming,solvPar.layerHeight,controlPoints,normals);

        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesDeltaIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);

        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationBody(freeVortons,solvPar.farDistance,center);

//        for (int i=0; i<freeVortons.size(); i++)
//        {
//            if ((freeVortons[i].getMid()-center).length()<0.5)
//                qDebug()<<(freeVortons[i].getMid()-center).length();
//        }
        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();
        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());



        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesEndIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        qDebug()<<"vel.start";
        QPair<int,int> boundaries=fragmentation.getStreamLinesSizes();
        double stepStr=0.2;
        QVector<Vector3D> velocitiesStream;
        functions.velForStreamLines(velocitiesStream,solvPar.streamVel,stepStr,freeVortons,boundaries);
        logger->createParaviewStreamlinesFile(velocitiesStream,boundaries,stepStr,i);
        logger->createParaviewTraceVerticesFile(freeVortons,i);
        if (i==0)
        {
            fragmentation.prepareGraphsX0(xFrames, forming);
            fragmentation.prepareGraphsZ0(zFrames, forming);
            fragmentation.prepareGraphsY0(yFrames, forming);
        }
        logger->createCenterGraphs(forming,solvPar.tau,i,freeVortons,solvPar.streamVel,xFrames,yFrames,zFrames);
        //logger->createCenterGraphs(forming,0.2,i);
        //logger->createParaviewTraceFile(freeVortons,i);
        logger->createParaviewFile(frames,pressures,velocities,tangentialVelocities,normalVelocitiesAfterIntegr,normalVelocitiesBeforeIntegr, normalVelocitiesCenterIntegr,normalVelocitiesDeltaIntegr, normalVelocitiesEndIntegr,framesSection,i);
        emit sendProgressRotationBody(i);
        emit repaintGUI(freeVortons, frames);

        if (motion==ACCELERATED && currentSpeed.length()<= solvPar.streamVel.length())
        {
            if ((currentSpeed+solvPar.streamVel/(solvPar.acceleratedStepsNum+2)).length()<solvPar.streamVel.length())
                currentSpeed+=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
            else
                currentSpeed=solvPar.streamVel;
        }
        qDebug()<<currentSpeed.x()<<" "<<currentSpeed.y()<<" "<<currentSpeed.z()<<"\n";
    }
    QVector<double> fis;
    fis.push_back(controlPoints[controlPoints.size()-2].y()/*BodyFragmentation::presetFunctionF(controlPoints[controlPoints.size()-2].x(),fragmentation.getForming())*/);
    for (int i=0; i<controlPoints.size()-2;i+=fragPar.rotationBodyFiFragNum)
        fis.push_back(controlPoints[i].y()/*BodyFragmentation::presetFunctionF(controlPoints[i].x(),fragmentation.getForming())*/);
    fis.push_back(controlPoints[controlPoints.size()-1].y()/*BodyFragmentation::presetFunctionF(controlPoints[controlPoints.size()-1].x(),fragmentation.getForming())*/);
    functions.cpAverage(cp,solvPar.stepsNum);
    functions.cpAverage(cp0,solvPar.stepsNum);
    functions.cpAverage(cp90,solvPar.stepsNum);
    functions.cpAverage(cp180,solvPar.stepsNum);
    functions.cpAverage(cp270,solvPar.stepsNum);
    qDebug()<<"File Created";
    logger->writeCpFile(cp,fis);
    logger->writeCpDegreeFile(cp,fis, 0);
    logger->writeCpDegreeFile(cp,fis, 90);
    logger->writeCpDegreeFile(cp,fis, 180);
    logger->writeCpDegreeFile(cp,fis, 270);
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;
}

void Solver::ovalSolver()
{
    Vorton vort;
    QVector <Vorton> v;
    vort.setMid(Vector3D(2.500000,10.000000,7.000000));
    vort.setTail(vort.getMid()+Vector3D(5.000000,8.000000,7.000000));
    vort.setVorticity(0.500000);
    vort.setRadius(0.200000);
    v.push_back(vort);
    Vorton v2;
    v2.setMid(Vector3D(2.400000,9.900000,6.950000));
    v2.setTail(v2.getMid()+Vector3D(1.000000,2.100000,3.200000));
    v2.setVorticity(0.500000);
    v2.setRadius(0.200000);
    v.push_back(v2);

//    Vorton vort;
//    QVector <Vorton> v;
//    vort.setMid(Vector3D(2.500000,10.000000,7.000000));
//    vort.setTail(vort.getMid()+Vector3D(5.000000,8.000000,7.000000));
//    vort.setVorticity(0.500000);
//    vort.setRadius(0.200000);
//    v.push_back(vort);
//    Vorton v2;
//    v2.setMid(Vector3D(3.500000,7.000000,8.000000));
//    v2.setTail(v2.getMid()+Vector3D(1.000000,2.100000,3.200000));
//    v2.setVorticity(0.500000);
//    v2.setRadius(0.200000);
//    v.push_back(v2);


    Vector3D deltar=v2.getTail()-v2.getMid();


    VelBsym res=FrameCalculations::velocityAndBsymmGauss3(v2.getMid(),deltar,Vector3D(0.0,0.0,0.0),v);



    QTime start = QTime::currentTime();
    const double gamma0 = 1.0;
    const double epsilon = 0.15;
    QVector <Vorton> V;
    QFile sourceGA("Kadr02500.txt");
    if (sourceGA.open(QIODevice::ReadOnly))
    {
        QString sizeStr=sourceGA.readLine();
        int size=sizeStr.toInt();
        int j=0;
        while (j<size) {
            sourceGA.readLine();
            sourceGA.readLine();
            QString r0Str=sourceGA.readLine();
            QStringList r0StrL=r0Str.split(" ",QString::SkipEmptyParts);
            Vorton newVort;
            newVort.setVorticity(gamma0);
            newVort.setRadius(epsilon);
            newVort.setMid(Vector3D(r0StrL[0].toDouble(),r0StrL[1].toDouble(),r0StrL[2].toDouble()));
            QString r1Str=sourceGA.readLine();
            QStringList r1StrL=r1Str.split(" ",QString::SkipEmptyParts);
            newVort.setTail(newVort.getMid()+Vector3D(r1StrL[0].toDouble(),r1StrL[1].toDouble(),r1StrL[2].toDouble()));
            V.append(newVort);
            j++;
        }
    }
    sourceGA.close();
    qDebug()<<V.size();
    QVector <Vorton> V1;
    QVector <Vorton> V2;

    for (int i=0; i<V.size()*0.5; i++)
        V1.append(V[i]);
    for (int i=V.size()*0.5; i<V.size(); i++)
        V2.append(V[i]);

    QString path = QCoreApplication::applicationDirPath();
    QDir folder(path);
    folder.cdUp();
    path=folder.absolutePath();
    QString kadrPath=path+"/KADR";
    path.append("/Oval_");
    path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
    folder.mkdir(path);
    folder.cd(path);
    folder.mkdir("traces");

    const Vector3D Vinf = Vector3D(0,0,0);
    double D = 1;
    const double tau = 0.01;

//    const int Nfi = 35;
//    const int N = 92; //всего отрезков в овале

    const int Nfi = 37;
    const int N = 194; //всего отрезков в овале
    const int Nl = (N - Nfi)*0.5; //по вертикальной части

    double dfi = 2*M_PI/Nfi;
    //double L = Nl*M_PI*D/Nfi;

    double dh = (V[1].getTail()-V[1].getMid()).length();
    //qDebug()<<(L+D)/D;

    const int N_steps = 10000;
    const double eDelta = 2*dh;
    const double fi_max = M_PI/12;




    double R = D*0.5;
//    for (int i=0; i<Nfi*0.5; i++)
//    {
//        double fi = dfi*i;
//        Vector3D a = Vector3D (R*cos(fi),L*0.5+R*sin(fi),0);
//        Vector3D r1 = Vector3D (R*cos(fi+dfi),L*0.5+R*sin(fi+dfi),0);
//        Vorton Vort = Vorton(0.5*(r1+a),r1,gamma0,epsilon);
//        V.append(Vort);
//    }

//    for (int i=0; i<Nfi*0.5; i++)
//    {
//        double fi = dfi*i+M_PI;
//        Vector3D a = Vector3D (R*cos(fi),-L*0.5+R*sin(fi),0);
//        Vector3D r1 = Vector3D (R*cos(fi+dfi),-L*0.5+R*sin(fi+dfi),0);
//        Vorton Vort = Vorton(0.5*(r1+a),r1,gamma0,epsilon);
//        V.append(Vort);
//    }

//    for (int i=0; i<Nl; i++)
//    {
//        double y = -L*0.5+dh*i;
//        Vector3D r1 = Vector3D (-R,y,0);
//        Vector3D a = Vector3D (-R,y+dh,0);
//        Vorton Vort = Vorton(0.5*(r1+a),r1,gamma0,epsilon);
//        V.append(Vort);
//    }

//    for (int i=0; i<Nl; i++)
//    {
//        double y = -L*0.5+dh*i;
//        Vector3D a = Vector3D (R,y,0);
//        Vector3D r1 = Vector3D (R,y+dh,0);
//        Vorton Vort = Vorton(0.5*(r1+a),r1,gamma0,epsilon);
//        V.append(Vort);
//    }



    int TurnRestr = 0, ElongationRestr = 0, MoveRestr = 0;
    double MaxMove = 100000;
    double MoveTime;

    FrameCalculations functions;
    QVector<std::shared_ptr<MultiFrame>> frams;
    QVector<Vorton> tempV1;

    QVector<Vorton> tempV2;
    QVector<Vorton> gaVortons;

    long double Re = 0.5;
    long double L  = 7.0;  // 7 - "длинное" кольцо,  3 - "короткое" кольцо

    // long double gam = 1.0;
    // long double dfi = dpi/(2.0*NR);

    int NR = 20;  //Число вортонов на полуокружности
    long double dpi = 2*M_PI;
    long double DLa = 0.5 * dpi * Re / NR;
    int NL = (int)((L - 2.0*Re)/DLa) + 1;
    long double DL = (L - 2.0*Re)/NL;
    long double dl_max_etalon = 2.0*(0.5*DL);
    long double dl_min_etalon = (0.5*DL)*0;


    for (int ii=2500; ii<N_steps; ii++)
    {
        //qDebug()<< "Step number: "<<i;
        //MoveElongationCalcStrictForTwo(V1,V2, tau, Vinf,eDelta,fi_max, MaxMove,MoveTime, TurnRestr, ElongationRestr, MoveRestr);
        //MoveElongationCalcStrictForTwoParallel(V1,V2, tau, Vinf,eDelta,fi_max, MaxMove,MoveTime, TurnRestr, ElongationRestr, MoveRestr);

        tempV1=V1;
        tempV2=V2;
        Logger log;
        QFile traceFile(path+"/traces/trace.vtk."+QString::number(ii));
        if (traceFile.open(QIODevice::WriteOnly))
        {
            QTextStream traceStream(&traceFile);
            traceStream<<"# vtk DataFile Version 3.0\n";
            traceStream<<"vtk output\n";
            traceStream<<"ASCII\n";
            traceStream<<"DATASET POLYDATA\n";
            traceStream<<"POINTS "+QString::number(V1.size()*2+V2.size()*2)+" float\n";
            for (int i=0; i<V1.size();i++)
            {
                Vector3D mid=V1[i].getMid();
                Vector3D tail=V1[i].getTail();
                Vector3D end=tail-mid;
                traceStream<<QString::number(i)+"\n";
                traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
                traceStream<<QString::number(end.x())+" "+QString::number(end.y())+" "+QString::number(end.z())+"\n";
            }
            traceStream.flush();
            for (int i=0; i<V2.size();i++)
            {
                Vector3D mid=V2[i].getMid();
                Vector3D tail=V2[i].getTail();
                Vector3D end=tail-mid;
                traceStream<<QString::number(i+V1.size())+"\n";
                traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
                traceStream<<QString::number(end.x())+" "+QString::number(end.y())+" "+QString::number(end.z())+"\n";
            }
            traceStream.flush();
            traceStream<<"LINES "+QString::number(V1.size()+V2.size())+" "+QString::number(V1.size()*3+V2.size()*3)+"\n";
            for (int i=0; i<(V1.size()+V2.size())*2;i=i+2)
            {
                traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
            }
            traceStream.flush();
            traceStream<<"POINT_DATA "+QString::number((V1.size()+V2.size())*2)+"\n";
            traceStream<<"SCALARS gamma double 1\n";
            traceStream<<"LOOKUP_TABLE default\n";
            for (int i=0; i<V1.size();i++)
            {
                traceStream<<QString::number(V1[i].getVorticity())+"\n";
                traceStream<<QString::number(V1[i].getVorticity())+"\n";
            }
            for (int i=0; i<V2.size();i++)
            {
                traceStream<<QString::number(V2[i].getVorticity())+"\n";
                traceStream<<QString::number(V2[i].getVorticity())+"\n";
            }
            traceStream.flush();

        }
        traceFile.close();

         // эйлер с перес;четом
        functions.displacementCalcGauss3(tempV1,tempV2,0.5*tau,Vinf,eDelta,fi_max,MaxMove,dl_max_etalon,dl_min_etalon);

        functions.displace(tempV1);
        functions.displace(tempV2);

        functions.displacementCalcGauss3(tempV1,tempV2,0.5*tau,Vinf,eDelta,fi_max,MaxMove,dl_max_etalon,dl_min_etalon);

        for (int i=0; i<V1.size(); i++) {
            V1[i].setMove(tempV1[i].getMove()*2.0);
            V1[i].setElongation(tempV1[i].getElongation()*2.0);
        }
        for (int i=0; i<V2.size(); i++) {
            V2[i].setMove(tempV2[i].getMove()*2.0);
            V2[i].setElongation(tempV2[i].getElongation()*2.0);
        }


        //эйлер
//                functions.displacementCalcGauss3(V1,V2,tau,Vinf,eDelta,fi_max,MaxMove);



        functions.displace(V1);
        functions.displace(V2);


        qDebug()<< "Step number: "<<ii;
        qDebug()<<"Time: "<<tau*ii;

        qDebug()<<"Turn restrictions: "<<TurnRestr;
        qDebug()<<"Length change restrictions: "<<ElongationRestr;


        if (ii%10==0)
        {
            gaVortons=log.gaVortons(kadrPath,ii/10);

        emit repaintGUI(V1+V2,gaVortons);
        emit makeScreenShot(path);
        }


//        QFile traceFile(path+"/traces/trace.vtk."+QString::number(ii));
//        if (traceFile.open(QIODevice::WriteOnly))
//        {
//            QTextStream traceStream(&traceFile);
//            traceStream<<"# vtk DataFile Version 3.0\n";
//            traceStream<<"vtk output\n";
//            traceStream<<"ASCII\n";
//            traceStream<<"DATASET POLYDATA\n";
//            traceStream<<"POINTS "+QString::number(V1.size()*2+V2.size()*2)+" float\n";
//            for (int i=0; i<V1.size();i++)
//            {
//                Vector3D mid=V1[i].getMid();
//                Vector3D tail=V1[i].getTail();
//                Vector3D begin=2.0*mid-tail;
//                traceStream<<QString::number(begin.x())+" "+QString::number(begin.y())+" "+QString::number(begin.z())+"\n";
//                traceStream<<QString::number(tail.x())+" "+QString::number(tail.y())+" "+QString::number(tail.z())+"\n";
//            }
//            traceStream.flush();
//            for (int i=0; i<V2.size();i++)
//            {
//                Vector3D mid=V2[i].getMid();
//                Vector3D tail=V2[i].getTail();
//                Vector3D begin=2.0*mid-tail;
//                traceStream<<QString::number(begin.x())+" "+QString::number(begin.y())+" "+QString::number(begin.z())+"\n";
//                traceStream<<QString::number(tail.x())+" "+QString::number(tail.y())+" "+QString::number(tail.z())+"\n";
//            }
//            traceStream.flush();
//            traceStream<<"LINES "+QString::number(V1.size()+V2.size())+" "+QString::number(V1.size()*3+V2.size()*3)+"\n";
//            for (int i=0; i<(V1.size()+V2.size())*2;i=i+2)
//            {
//                traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
//            }
//            traceStream.flush();
//            traceStream<<"POINT_DATA "+QString::number((V1.size()+V2.size())*2)+"\n";
//            traceStream<<"SCALARS gamma double 1\n";
//            traceStream<<"LOOKUP_TABLE default\n";
//            for (int i=0; i<V1.size();i++)
//            {
//                traceStream<<QString::number(V1[i].getVorticity())+"\n";
//                traceStream<<QString::number(V1[i].getVorticity())+"\n";
//            }
//            for (int i=0; i<V2.size();i++)
//            {
//                traceStream<<QString::number(V2[i].getVorticity())+"\n";
//                traceStream<<QString::number(V2[i].getVorticity())+"\n";
//            }
//            traceStream.flush();

//        }
//        traceFile.close();

        }
    qDebug()<<"Time is: " <<N_steps*tau;
    qDebug()<<"Vortons quantity: "<<V.size();
    qDebug()<<"Dlina otrezka ravna: "<<dh;
    //qDebug()<<"Udlinenie: "<<(L+D)/D;
    qDebug()<<"Program time is: "<<start.elapsed()*0.001<<" s";
}

void Solver::ringsSolver()
{
    QTime start = QTime::currentTime();

    QString path = QCoreApplication::applicationDirPath();
    QDir folder(path);
    folder.cdUp();
    path=folder.absolutePath();
    path.append("/Rings_");
    path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
    folder.mkdir(path);
    folder.cd(path);
    folder.mkdir("traces");

    const Vector3D Vinf = Vector3D(0,0,0);
    const double tau = 0.01;
    const int Nc = 60;
    const double R = 1;
    double L = M_PI*R/Nc;
    const double epsilon = 0.1;
    double dfi = 2*M_PI/Nc;
    const double gamma0 = 1;
    const double H = 1.2;
    const int N_steps = 20000;
    const double eDelta = 2.0 *L;
    const double fi_max = M_PI/12;

    QVector <Vorton> V1;
    V1.resize(Nc);
    QVector <Vorton> V2;
    V2.resize(Nc);

    for (int i=0; i<Nc; i++)
    {
        double fi = dfi*i;
        Vector3D a = Vector3D (R*cos(fi),R*sin(fi),0);
        Vector3D r1 = Vector3D (R*cos(fi+dfi),R*sin(fi+dfi),0);
        Vorton Vort = Vorton(0.5*(r1+a),r1,gamma0,epsilon);
        V1[i]= (Vort);
    }

    for (int i=0; i<Nc; i++)
    {
        double fi = dfi*i;
        Vector3D a = Vector3D (R*cos(fi),R*sin(fi),H);
        Vector3D r1 = Vector3D (R*cos(fi+dfi),R*sin(fi+dfi),H);
        Vorton Vort = Vorton(0.5*(r1+a),r1,gamma0,epsilon);
        V2[i]= (Vort);
    }

    QVector <double> distance;
    distance.resize(N_steps);
    QVector <double> R1;
    R1.resize(N_steps);
    QVector <double>  R2;
    R2.resize(N_steps);
    QVector <double> D1;
    D1.resize(N_steps);
    QVector <double> D2;
    D2.resize(N_steps);
    QVector <double> time;
    time.resize(N_steps);

    for (int i=0; i<N_steps; i++)
        time[i] = i*tau;

    int TurnRestr = 0, ElongationRestr = 0, MoveRestr = 0;
    double MaxMove = 100000;
    double MoveTime;

    FrameCalculations functions;
    QVector<std::shared_ptr<MultiFrame>> frams;
    QVector<Vorton> tempV1;

    QVector<Vorton> tempV2;

    double Re = 0.5;
    //long double L  = 7.0;  // 7 - "длинное" кольцо,  3 - "короткое" кольцо

    // long double gam = 1.0;
    // long double dfi = dpi/(2.0*NR);

    int NR = 20;  //Число вортонов на полуокружности
    double dpi = 2*M_PI;
    double DLa = 0.5 * dpi * Re / NR;
    int NL = static_cast<int>((L - 2.0*Re)/DLa) + 1;
    double DL = (L - 2.0*Re)/NL;
    double dl_max_etalon = 2.0*(0.5*DL);
    double dl_min_etalon = (0.5*DL)*0;


    for (int ii=0; ii<N_steps; ii++)
    {
        //MoveElongationCalcStrictForTwo(V1,V2, tau, Vinf,eDelta,fi_max, MaxMove,MoveTime, TurnRestr, ElongationRestr, MoveRestr);
        //MoveElongationCalcStrictForTwoParallel(V1,V2, tau, Vinf,eDelta,fi_max, MaxMove,MoveTime, TurnRestr, ElongationRestr, MoveRestr);
        tempV1=V1;
        tempV2=V2;
        functions.displacementCalcGauss3(tempV1,tempV2,tau,Vinf,eDelta,fi_max,MaxMove,dl_max_etalon,dl_min_etalon);
        for (int i=0; i<V1.size(); i++) {
            V1[i].setMove(tempV1[i].getMove());
            V1[i].setElongation(tempV1[i].getElongation());
        }
        for (int i=0; i<V2.size(); i++) {
            V2[i].setMove(tempV2[i].getMove());
            V2[i].setElongation(tempV2[i].getElongation());
        }
        functions.displace(tempV1);
        functions.displace(tempV2);

        functions.displacementCalcGauss3(tempV1,tempV2,tau,Vinf,eDelta,fi_max,MaxMove,dl_max_etalon,dl_min_etalon);

        for (int i=0; i<V1.size(); i++) {
            V1[i].setMove((tempV1[i].getMove()+V1[i].getMove())*0.5);
            V1[i].setElongation((tempV1[i].getElongation()+V1[i].getElongation())*0.5);
        }
        for (int i=0; i<V2.size(); i++) {
            V2[i].setMove((tempV2[i].getMove()+V2[i].getMove())*0.5);
            V2[i].setElongation((tempV2[i].getElongation()+V2[i].getElongation())*0.5);
        }
        functions.displace(V1);
        functions.displace(V2);


        if (ii%100==0) {
        qDebug()<< "Step number: "<<ii;
        qDebug()<<"Time: "<<tau*ii;
        qDebug()<<"Turn restrictions: "<<TurnRestr;
        qDebug()<<"Length change restrictions: "<<ElongationRestr;
        emit repaintGUI(V1+V2,frams);


        QFile traceFile(path+"/traces/trace.vtk."+QString::number(ii));
        if (traceFile.open(QIODevice::WriteOnly))
        {
            QTextStream traceStream(&traceFile);
            traceStream<<"# vtk DataFile Version 3.0\n";
            traceStream<<"vtk output\n";
            traceStream<<"ASCII\n";
            traceStream<<"DATASET POLYDATA\n";
            traceStream<<"POINTS "+QString::number(V1.size()*2+V2.size()*2)+" float\n";
            for (int i=0; i<V1.size();i++)
            {
                Vector3D mid=V1[i].getMid();
                Vector3D tail=V1[i].getTail();
                Vector3D begin=2.0*mid-tail;
                traceStream<<QString::number(begin.x())+" "+QString::number(begin.y())+" "+QString::number(begin.z())+"\n";
                traceStream<<QString::number(tail.x())+" "+QString::number(tail.y())+" "+QString::number(tail.z())+"\n";
            }
            traceStream.flush();
            for (int i=0; i<V2.size();i++)
            {
                Vector3D mid=V2[i].getMid();
                Vector3D tail=V2[i].getTail();
                Vector3D begin=2.0*mid-tail;
                traceStream<<QString::number(begin.x())+" "+QString::number(begin.y())+" "+QString::number(begin.z())+"\n";
                traceStream<<QString::number(tail.x())+" "+QString::number(tail.y())+" "+QString::number(tail.z())+"\n";
            }
            traceStream.flush();
            traceStream<<"LINES "+QString::number(V1.size()+V2.size())+" "+QString::number(V1.size()*3+V2.size()*3)+"\n";
            for (int i=0; i<(V1.size()+V2.size())*2;i=i+2)
            {
                traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
            }
            traceStream.flush();
            traceStream<<"POINT_DATA "+QString::number((V1.size()+V2.size())*2)+"\n";
            traceStream<<"SCALARS gamma double 1\n";
            traceStream<<"LOOKUP_TABLE default\n";
            for (int i=0; i<V1.size();i++)
            {
                traceStream<<QString::number(V1[i].getVorticity())+"\n";
                traceStream<<QString::number(V1[i].getVorticity())+"\n";
            }
            for (int i=0; i<V2.size();i++)
            {
                traceStream<<QString::number(V2[i].getVorticity())+"\n";
                traceStream<<QString::number(V2[i].getVorticity())+"\n";
            }
            traceStream.flush();

        }
        traceFile.close();
        }
    }
    qDebug()<<"Program time is: "<<start.elapsed()*0.001<<" s";
}

void Solver::rotationBodyFreeMotionSolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Logger* logger=createLog(ROTATIONBODY);
    BodyFragmentation fragmentation(BodyType::ROTATIONBODY, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();

    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();
    QVector<std::shared_ptr<MultiFrame>> oldFrames=FrameCalculations::copyFrames(frames);
    QVector<Vector3D> oldControlPoints=controlPoints;
    QVector<Vector3D> oldNormals=normals;
    QVector<Vector3D> oldControlPointsRaised=controlPointsRaised;
    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    //Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
    Vector3D translation=freeMotionPar.bodyVel*solvPar.tau;

    //double xEnd=fragPar.rotationBodyXEnd;
//    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg,0.0,0.0);
    Vector3D center((fragPar.rotationBodyXBeg+fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength+fragmentation.getForming().diameter*0.5/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);

    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg,0.0,0.0);
    double xend;
    Vector3D currentSpeed;
    if (motion==NOACCELERATE)
        currentSpeed=solvPar.streamVel;
    else
        currentSpeed=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
    FormingParameters forming=fragmentation.getForming();
    switch (forming.typeNum)
    {
    case 0:
    {
    center=Vector3D((fragPar.rotationBodyXBeg+forming.sectorOneLength+forming.sectorTwoLength+forming.diameter*0.5/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);
    xend=fragPar.rotationBodyXBeg+forming.sectorOneLength+forming.sectorTwoLength+forming.diameter*0.5;
    break;
    }
    case 1:
    {
    center=Vector3D((fragPar.rotationBodyXBeg+forming.sectorOneLength)*0.5,0.0,0.0);
    xend=fragPar.rotationBodyXBeg+forming.sectorOneLength;
    break;
    }
    case 2:
    {
    center=Vector3D((fragPar.rotationBodyXBeg+forming.sectorOneLength)*0.5,0.0,0.0);
    xend=fragPar.rotationBodyXBeg+forming.sectorOneLength;
    break;
    }
    };
    Vector3D nullCenter=center;
    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    QVector<double> cp(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp0(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp90(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp180(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp270(fragPar.rotationBodyPartFragNum+1);
    emit updateRotationBodyMaximum(solvPar.stepsNum-1);
    QVector<double> pressures;
    QVector<Vector3D> velocities;
    QVector<double> tangentialVelocities;
    QVector<double> normalVelocities;

    QVector<std::shared_ptr<MultiFrame>> framesSection;
    QVector<Vector3D> centerSection;
    QVector<Vector3D> sectionNormals;

    logger->writePassport(solvPar,fragPar);
    pressures.resize(controlPointsRaised.size());
    velocities.resize(controlPoints.size());
    normalVelocities.resize(controlPoints.size());
    tangentialVelocities.resize(controlPoints.size());
    dMatrix3 R;
    Eigen::Matrix3d rotation;
    Vector3D angularVel;
    IntegrationResults results=functions.integrateParameters(xend-bodyNose.x(), 1700, fragmentation.getForming());
    functions.epsZero(frames);
    Vector3D oldCenter=center;
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        Vector3D relVel=currentSpeed-freeMotionPar.bodyVel;
        if(checkFinishing())
            return;
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        //Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,angularVel,controlPoints,center);
        Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,relVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force;
        Vector3D torque;
        functions.forceAndTorqueCalc(relVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals,center,force,torque);
        forces[i]=force;
        torques[i]=torque;
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        for (int i=0; i<controlPointsRaised.size(); i++)
            pressures[i]=(FrameCalculations::pressureCalc(controlPointsRaised[i], solvPar.streamVel, solvPar.streamPres,solvPar.density, frames, freeVortons, solvPar.tau)-solvPar.streamPres)/
                    (solvPar.density*Vector3D::dotProduct(solvPar.streamVel,solvPar.streamVel)*0.5);


        for (int i=0; i<controlPoints.size();i++)
            velocities[i]=FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames);


        for (int i=0; i<controlPoints.size();i++)
            tangentialVelocities[i]=(velocities[i]-normalVelocities[i]*normals[i]).length();
        for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
            normalVelocities[i]=(velocities[i]-normalVelocities[i]*sectionNormals[i-controlPoints.size()]).length();

        functions.cpSumRotationBody(i,solvPar.stepsNum, cp, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp0, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,0);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp90, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,90);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp180, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,180);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp270, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,270);

        functions.epsNormal(newVortons,fragPar.vortonsRad);
        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();
        if (i==0)
        {
            dRFromEulerAngles(R,0.0,0.0,0.0);
            for (int i=0; i<3; i++)
                for (int j=0; j<3; j++)
                    rotation(i,j)=R[i*4+j];

        }

        functions.displace(freeVortons);
        QVector<Vorton> oldVortons;

        //functions.getBackAndRotate(Solver::getBackGA,freeVortons,bodyNose,xend,solvPar.layerHeight,controlPoints,normals,oldVortons,frames,forming);
        functions.getBackAndRotateMovingRotationBody(freeVortons,center, nullCenter,bodyNose,xend, solvPar.layerHeight, controlPoints,normals,rotation,fragmentation.getForming());
        //functions.universalGetBack(freeVortons,solvPar.layerHeight,controlPoints,normals,frames);
        oldCenter=center;
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
               //     qDebug()<<center.x()<<" "<<center.y()<<" "<<center.z();
        functions.removeFarRotationBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        //        Eigen::Matrix3d inertia;
        //        inertia<<1,0,0,0,1,0,0,0,1;


//        qDebug()<<"Center is "<<center.x()<<" "<< center.y()<<" "<<center.z();

        functions.translateAndRotatev2(frames,freeVortons,results.mass,results.inertiaTensor,torque,rotation,force,center,nullCenter, results.massCenter,/*(i+1)**/solvPar.tau,freeMotionPar.bodyVel,controlPoints,normals,controlPointsRaised,oldFrames,oldControlPoints,oldNormals,oldControlPointsRaised, angularVel,bodyNose,xend);
        functions.clear();
//        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center,bodyNose, xend, fragPar);
 //       FrameCalculations::translateVortons(translation, freeVortons);
        emit sendNormalsVis(controlPoints,normals);
        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());

        logger->createParaviewTraceFile(freeVortons,i);
        logger->createParaviewFile(frames,pressures,velocities,tangentialVelocities,normalVelocities,framesSection,i);
        emit sendProgressRotationBody(i);
        emit repaintGUI(freeVortons, frames);
        if (motion==ACCELERATED && currentSpeed.length()<= solvPar.streamVel.length())
        {
            if ((currentSpeed+solvPar.streamVel/(solvPar.acceleratedStepsNum+2)).length()<solvPar.streamVel.length())
                currentSpeed+=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
            else
                currentSpeed=solvPar.streamVel;
        }


    }
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;
}

/*!
Осуществляет расчет неподвижного тела вращения со срезом с заданными параметрами
\param fragPar Параметры разбиения тела вращения со срезом
*/
void Solver::rotationCutBodySolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Logger* logger=createLog(ROTATIONBOTTOMCUT);
    BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    emit sendNormalsVis(controlPoints,normals);
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);
    FormingParametersRBC forming=fragmentation.getFormingRBC();
    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg/*-2.5+fragPar.rotationBodyXEnd*/,0.0,0.0);
    Vector3D center;
    double xend;
//    Vector3D center((bodyNose.x()+fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);
//    double xend=fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength+bodyNose.x();
    switch (forming.type)
    {
    case ELLIPSOID_CONE:
    {
        center=Vector3D((bodyNose.x()+forming.ellipsoidLength+forming.coneLength)*0.5,0.0,0.0);
        xend=bodyNose.x()+forming.ellipsoidLength+forming.coneLength;
        break;
    }
    case ELLIPSOID_CYLINDER:
    {
        center=Vector3D((bodyNose.x()+forming.fullLength)*0.5,0.0,0.0);
        xend=bodyNose.x()+forming.fullLength;
        break;
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        center=Vector3D((bodyNose.x()+forming.fullLength)*0.5,0.0,0.0);
        xend=bodyNose.x()+forming.fullLength;
        break;
    }
    };

    Vector3D currentSpeed;
    if (motion==NOACCELERATE)
        currentSpeed=solvPar.streamVel;
    else
        currentSpeed=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

    QVector<double> cp(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp0(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp90(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp180(fragPar.rotationBodyPartFragNum+1);
    QVector<double> cp270(fragPar.rotationBodyPartFragNum+1);
    emit updateRotationBodyMaximum(solvPar.stepsNum-1);
    QVector<double> pressures;
    QVector<Vector3D> velocities;
    QVector<double> tangentialVelocities;
    QVector<double> normalVelocitiesBeforeIntegr;
    QVector<double> normalVelocitiesCenterIntegr;
    QVector<double> normalVelocitiesDeltaIntegr;
    QVector<double> normalVelocitiesAfterIntegr;
    QVector<double> normalVelocitiesEndIntegr;


    QVector<std::shared_ptr<MultiFrame>> framesSection;
    QVector<Vector3D> centerSection;
    QVector<Vector3D> sectionNormals;
    logger->writePassport(solvPar,fragPar,fragmentation.getForming(),functions.calcFrameSizes(frames));
    //functions.calcSection(fragmentation.getForming().diameter,center.x(),fragPar.rotationBodyFiFragNum,framesSection,centerSection,sectionNormals);

    pressures.resize(controlPointsRaised.size()+centerSection.size());
    velocities.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesBeforeIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesCenterIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesDeltaIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesEndIntegr.resize(controlPoints.size()+centerSection.size());
    normalVelocitiesAfterIntegr.resize(controlPoints.size()+centerSection.size());
    tangentialVelocities.resize(controlPoints.size()+centerSection.size());
    functions.epsZero(frames);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
            return;
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(currentSpeed,freeVortons,normals,controlPoints);
        if (i==0)
        {
            QFile bCol("Col.txt");
            bCol.open(QIODevice::WriteOnly);
            QTextStream bcolstr(&bCol);
            for (int i=0; i<controlPoints.size();i++)
            {
                bcolstr<<QString::number(controlPoints[i].x())+" "+QString::number(controlPoints[i].y())+" "+QString::number(controlPoints[i].z())+"\n";
            }
            bcolstr.flush();
            bCol.close();
        }
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);





        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesAfterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames),normals[i]);
        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        QVector<std::pair<double,double>> boundariesParallelipeped=functions.makeParalllepiped(newVortons);
        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);

        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,currentSpeed,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);

        Vector3D force;
        Vector3D torque;
        functions.forceAndTorqueCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals,center,force,torque);
        forces[i]=force;
        torques[i]=torque;
        cAerodynamics[i] = 2.0*force/(solvPar.density*solvPar.streamVel.lengthSquared()*M_PI*pow(fragmentation.getFormingRBC().ellipsoidDiameter*0.5,2));

        for (int i=0; i<controlPointsRaised.size(); i++)
            pressures[i]=(FrameCalculations::pressureCalc(controlPointsRaised[i], solvPar.streamVel, solvPar.streamPres,solvPar.density, frames, freeVortons, solvPar.tau)-solvPar.streamPres)/
                    (solvPar.density*Vector3D::dotProduct(solvPar.streamVel,solvPar.streamVel)*0.5);
//        for (int i=controlPointsRaised.size(); i<controlPointsRaised.size()+centerSection.size(); i++)
//            pressures[i]=(FrameCalculations::pressureCalc(centerSection[i-controlPoints.size()], solvPar.streamVel, solvPar.streamPres,solvPar.density, frames, freeVortons, solvPar.tau)-solvPar.streamPres)/
//                    (solvPar.density*Vector3D::dotProduct(solvPar.streamVel,solvPar.streamVel)*0.5);

        for (int i=0; i<controlPoints.size();i++)
            velocities[i]=FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames);
//        for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
//            velocities[i]=FrameCalculations::velocity(centerSection[i-controlPoints.size()],solvPar.streamVel,freeVortons,frames);

        functions.cpSumRotationCutBody(i,solvPar.stepsNum, cp, fragPar.rotationBodyFiFragNum, fragPar.rotationBodyRFragNum,solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised);
        functions.cpRotationCutBodyDegree(i,solvPar.stepsNum, cp0, fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,0);
        functions.cpRotationCutBodyDegree(i,solvPar.stepsNum, cp90, fragPar.rotationBodyFiFragNum, fragPar.rotationBodyRFragNum,solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,90);
        functions.cpRotationCutBodyDegree(i,solvPar.stepsNum, cp180, fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,180);
        functions.cpRotationCutBodyDegree(i,solvPar.stepsNum, cp270, fragPar.rotationBodyFiFragNum,fragPar.rotationBodyRFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,270);

        for (int i=0; i<controlPoints.size();i++)
            tangentialVelocities[i]=(velocities[i]-normalVelocitiesBeforeIntegr[i]*normals[i]).length();
        functions.epsNormal(newVortons,fragPar.vortonsRad);
        freeVortons.append(newVortons);
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesBeforeIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();
        QVector<Vorton> oldVortons=freeVortons;
        functions.displace(freeVortons);
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesCenterIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        //functions.getBackAndRotate(Solver::getBackGA,freeVortons,oldVortons,frames,xend,solvPar.layerHeight,controlPoints,normals,bodyNose,forming);
        functions.universalGetBack(freeVortons,boundariesParallelipeped,solvPar.layerHeight,controlPoints,normals,frames);
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesDeltaIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);
        //functions.getBackAndRotateRotationCutBody(freeVortons, xend, solvPar.layerHeight, controlPoints,normals, bodyNose,forming);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();
        for (int i=0; i<controlPoints.size();i++)
            normalVelocitiesEndIntegr[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons),normals[i]);

        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());
        QPair<int,int> boundaries=fragmentation.getStreamLinesSizes();
        double stepStr=0.2;
        QVector<Vector3D> velocitiesStream;
        functions.velForStreamLines(velocitiesStream,solvPar.streamVel,stepStr,freeVortons,boundaries);
        logger->createParaviewStreamlinesFile(velocitiesStream,boundaries,stepStr,i);
        logger->createParaviewTraceVerticesFile(freeVortons,i);
        //logger->createParaviewTraceFile(freeVortons,i);
        logger->createParaviewFile(frames,pressures,velocities,tangentialVelocities,normalVelocitiesAfterIntegr,normalVelocitiesBeforeIntegr,normalVelocitiesCenterIntegr,normalVelocitiesDeltaIntegr,normalVelocitiesEndIntegr,framesSection,i);
        emit sendProgressRotationCutBody(i);
        emit repaintGUI(freeVortons, frames);

        if (motion==ACCELERATED && currentSpeed.length()<= solvPar.streamVel.length())
        {
            if ((currentSpeed+solvPar.streamVel/(solvPar.acceleratedStepsNum+2)).length()<solvPar.streamVel.length())
                currentSpeed+=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
            else
                currentSpeed=solvPar.streamVel;
        }
        qDebug()<<currentSpeed.x()<<" "<<currentSpeed.y()<<" "<<currentSpeed.z()<<"\n";

    }
    QVector<double> fis;
    fis.push_back(controlPoints[controlPoints.size()-2].y()/*BodyFragmentation::presetFunctionF(controlPoints[controlPoints.size()-2].x(),fragmentation.getForming())*/);
    for (int i=0; i<controlPoints.size()-fragPar.rotationBodyRFragNum*fragPar.rotationBodyFiFragNum;i+=fragPar.rotationBodyFiFragNum)
        fis.push_back(controlPoints[i].y()/*BodyFragmentation::presetFunctionF(controlPoints[i].x(),fragmentation.getForming())*/);
    fis.push_back(controlPoints[controlPoints.size()-1].y()/*BodyFragmentation::presetFunctionF(controlPoints[controlPoints.size()-1].x(),fragmentation.getForming())*/);
    functions.cpAverage(cp,solvPar.stepsNum);
    functions.cpAverage(cp0,solvPar.stepsNum);
    functions.cpAverage(cp90,solvPar.stepsNum);
    functions.cpAverage(cp180,solvPar.stepsNum);
    functions.cpAverage(cp270,solvPar.stepsNum);
    qDebug()<<"File Created";
    logger->writeCpFile(cp,fis);
    logger->writeCpDegreeFile(cp,fis, 0);
    logger->writeCpDegreeFile(cp,fis, 90);
    logger->writeCpDegreeFile(cp,fis, 180);
    logger->writeCpDegreeFile(cp,fis, 270);
    logger->writeSolverTime(start.elapsed()*0.001);

    logger->closeFiles();
    delete logger;
}

/*!
Осуществляет расчет неподвижного тела вращения со срезом с заданными параметрами вблизи экрана
\param fragPar Параметры разбиения тела вращения со срезом
\param screenDistance Расстояние между дном тела и экраном
*/
void Solver::rotationCutBodySolverNearScreen(const FragmentationParameters &fragPar, const double screenDistance)
{
    QTime start=QTime::currentTime();
    Logger* logger=createLog(ROTATIONBOTTOMCUT);
    BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    emit sendNormalsVis(controlPoints,normals);
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    FormingParametersRBC forming=fragmentation.getFormingRBC();
    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg/*-2.5+fragPar.rotationBodyXEnd*/,0.0,0.0);
    Vector3D center;
    double xend;
//    Vector3D center((bodyNose.x()+fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);
//    double xend=fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength+bodyNose.x();
    switch (forming.type)
    {
    case ELLIPSOID_CONE:
    {
        center=Vector3D((bodyNose.x()+forming.ellipsoidLength+forming.coneLength)*0.5,0.0,0.0);
        xend=bodyNose.x()+forming.ellipsoidLength+forming.coneLength;
        break;
    }
    case ELLIPSOID_CYLINDER:
    {
        center=Vector3D((bodyNose.x()+forming.fullLength)*0.5,0.0,0.0);
        xend=bodyNose.x()+forming.fullLength;
        break;
    }
    };

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);


    //FrameCalculations::translateBody(Vector3D(-(fragPar.rotationBodyXEnd+screenDistance),0.0,0.0),frames,controlPoints,controlPointsRaised,center);
    //double xEnd=-screenDistance;
    logger->writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
           return;
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(solvPar.streamVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);


        QVector<Vorton> symNewVortons=newVortons;
        QVector<Vorton> symFreeVortons=freeVortons;
        QVector<std::shared_ptr<MultiFrame>> symFrames=FrameCalculations::copyFrames(frames);
        FrameCalculations::reflect(symFreeVortons,symNewVortons,symFrames);

        functions.displacementLaunchCalc(freeVortons,newVortons,symFreeVortons,symNewVortons, solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        FrameCalculations::reflectMove(symFreeVortons,freeVortons);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames+symFrames,freeVortons+symFreeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        forces[i]=force;
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));



        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationCutLaunchedBody(freeVortons, bodyNose, -screenDistance, solvPar.layerHeight, controlPoints,normals,forming);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());

        emit sendProgressRotationCutBody(i);
        emit repaintGUI(freeVortons, frames);
    }
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;
}

/*!
Осуществляет расчет движущегося тела вращения со срезом с заданными параметрами
\param fragPar Параметры разбиения тела вращения со срезом
*/
void Solver::rotationCutBodyFreeMotionSolver(const FragmentationParameters &fragPar)
{
        QTime start=QTime::currentTime();
        Logger* logger=createLog(ROTATIONBOTTOMCUT);
        BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar);
        QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
        QVector<Vector3D> normals=fragmentation.getNormals();

        QVector<double> squares=fragmentation.getSquares();
        QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
        QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();
        QVector<std::shared_ptr<MultiFrame>> oldFrames=FrameCalculations::copyFrames(frames);
        QVector<Vector3D> oldControlPoints=controlPoints;
        QVector<Vector3D> oldNormals=normals;
        QVector<Vector3D> oldControlPointsRaised=controlPointsRaised;
        FrameCalculations functions;
        functions.matrixCalc(frames,controlPoints,normals);

//        Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
        Vector3D translation=freeMotionPar.bodyVel*solvPar.tau;
        QVector<Vorton> freeVortons;
        QVector<Vorton> newVortons;
        emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

        FormingParametersRBC forming=fragmentation.getFormingRBC();
        Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg/*-2.5+fragPar.rotationBodyXEnd*/,0.0,0.0);
        Vector3D center;
        double xend;
    //    Vector3D center((bodyNose.x()+fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength/*fragPar.rotationBodyXEnd*/)*0.5,0.0,0.0);
    //    double xend=fragmentation.getForming().sectorOneLength+fragmentation.getForming().sectorTwoLength+bodyNose.x();
        Vector3D currentSpeed;
        if (motion==NOACCELERATE)
            currentSpeed=solvPar.streamVel;
        else
            currentSpeed=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
        switch (forming.type)
        {
        case ELLIPSOID_CONE:
        {
            center=Vector3D((bodyNose.x()+forming.ellipsoidLength+forming.coneLength)*0.5,0.0,0.0);
            xend=bodyNose.x()+forming.ellipsoidLength+forming.coneLength;
            break;
        }
        case ELLIPSOID_CYLINDER:
        {
            center=Vector3D((bodyNose.x()+forming.fullLength)*0.5,0.0,0.0);
            xend=bodyNose.x()+forming.fullLength;
            break;
        }
        case ELLIPSOID_CYLINDER_CONE:
        {
            center=Vector3D((bodyNose.x()+forming.fullLength)*0.5,0.0,0.0);
            xend=bodyNose.x()+forming.fullLength;
        }
        };
        Vector3D nullCenter=center;
        dMatrix3 R;
        Eigen::Matrix3d rotation;
        Eigen::Matrix3d rotationNull;
        Vector3D angularVel;
        IntegrationResults results=functions.integrateParameters(xend-bodyNose.x(), 1700, fragmentation.getFormingRBC());
        functions.epsZero(frames);
        logger->writePassport(solvPar,fragPar);
        for (int i=0; i<solvPar.stepsNum; i++)
        {
            Vector3D relVel=currentSpeed-freeMotionPar.bodyVel;
            if (i==0)
            {
                QFile bCol("Col.txt");
                bCol.open(QIODevice::WriteOnly);
                QTextStream bcolstr(&bCol);
                for (int i=0; i<controlPoints.size();i++)
                {
                    bcolstr<<QString::number(controlPoints[i].x())+" "+QString::number(controlPoints[i].y())+" "+QString::number(controlPoints[i].z())+"\n";
                }
                bcolstr.flush();
                bCol.close();
            }
            if(checkFinishing())
               return;
            QTime stepTime=QTime::currentTime();
            newVortons.clear();
            Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,controlPoints);
            Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
            emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
            emit sendReguliser(vorticities[vorticities.size()-1]);
            FrameCalculations::setVorticity(frames,vorticities);

            newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
            int generatedNum=newVortons.size();
            functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
            functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

            functions.displacementCalc(freeVortons,newVortons,solvPar.tau,relVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
            Vector3D force;
            Vector3D torque;
            functions.forceAndTorqueCalc(relVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals,center,force,torque);
            //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));
            forces[i]=force;
            torques[i]=torque;

            functions.epsNormal(newVortons,fragPar.vortonsRad);
            freeVortons.append(newVortons);

            Counters countersBeforeIntegration=functions.getCounters();
            Timers timersBeforeIntegration=functions.getTimers();
            Restrictions restrictions=functions.getRestrictions();
            functions.clear();

            if (i==0)
            {
                dRFromEulerAngles(R,0.0,0.0,0.0);
                for (int i=0; i<3; i++)
                    for (int j=0; j<3; j++)
                        rotation(i,j)=R[i*4+j];
                rotationNull=rotation;

            }

            functions.displace(freeVortons);
            functions.getBackAndRotateMovingRotationCutBody(freeVortons, center,nullCenter,bodyNose,xend, solvPar.layerHeight, controlPoints,normals,rotation,forming);
            functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
            functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);

            functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

            Counters countersAfterIntegration=functions.getCounters();
            Timers timersAfterIntegration=functions.getTimers();

            functions.clear();
            Vector3D oldCenter=center;
            functions.translateAndRotatev3(frames,freeVortons,results.mass,results.inertiaTensor,torque,rotation,rotationNull,force,center,nullCenter, results.massCenter,/*(i+1)**/solvPar.tau,freeMotionPar.bodyVel,controlPoints,normals,controlPointsRaised,oldFrames,oldControlPoints,oldNormals,oldControlPointsRaised, angularVel,bodyNose,xend);
//            bodyNose+=center-oldCenter;

//            FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center,bodyNose, xend, fragPar);
//            FrameCalculations::translateVortons(translation, freeVortons);
            emit sendNormalsVis(controlPoints,normals);
            logger->writeForces(force,Vector3D(0.0,0.0,0.0));
            logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
            logger->writeVortons(frames,freeVortons,i);
            logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                               FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());

            emit sendProgressRotationCutBody(i);
            emit repaintGUI(freeVortons, frames);

            if (motion==ACCELERATED && currentSpeed.length()<= solvPar.streamVel.length())
            {
                if ((currentSpeed+solvPar.streamVel/(solvPar.acceleratedStepsNum+2)).length()<solvPar.streamVel.length())
                    currentSpeed+=solvPar.streamVel/(solvPar.acceleratedStepsNum+2);
                else
                    currentSpeed=solvPar.streamVel;
            }


        }
        logger->writeSolverTime(start.elapsed()*0.001);
        logger->closeFiles();
        delete logger;
}

/*!
Осуществляет расчет задачи старта для тела вращения со срезом с заданными параметрами
\param fragPar Параметры разбиения тела вращения со срезом
*/
void Solver::rotationCutBodyLaunchSolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
   Logger* logger=createLog(ROTATIONBOTTOMCUT);



    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);


//    double xBeg=fragPar.rotationBodyXBeg;
//    double xEnd=fragPar.rotationBodyXEnd;
//    double xBeg=0.0;
    Vector3D bodyNose=freeMotionPar.bodyVel*solvPar.tau;
    Vector3D center=bodyNose/2;

    Vector3D nullCenter=center;
    double xEnd=0.0;
    logger->writePassport(solvPar,fragPar);
    BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar, true);
    int exitStep=0;
    FormingParametersRBC form= fragmentation.getFormingRBC();
    double fullLength;
        switch (form.type)
        {
        case ELLIPSOID_CONE:
        {
            nullCenter=Vector3D((bodyNose.x()+form.ellipsoidLength+form.coneLength)*0.5,0.0,0.0);
            fullLength=form.ellipsoidLength+form.coneLength;
            break;
        }
        case ELLIPSOID_CYLINDER:
        {
            nullCenter=Vector3D((bodyNose.x()+form.fullLength)*0.5,0.0,0.0);
            fullLength=form.fullLength;
            break;
        }
        case ELLIPSOID_CYLINDER_CONE:
        {
            nullCenter=Vector3D((bodyNose.x()+form.fullLength)*0.5,0.0,0.0);
            fullLength=form.fullLength;
            break;
        }
        }
         FrameCalculations functions;
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
            return;


        fragmentation.rotationCutBodyLaunchFragmentation(i,freeMotionPar.bodyVel,solvPar.tau,fullLength);


        QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
        QVector<Vector3D> normals=fragmentation.getNormals();

        QVector<double> squares=fragmentation.getSquares();
        QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
        QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

        functions.epsZero(frames);

        QVector<std::shared_ptr<MultiFrame>> oldFrames=FrameCalculations::copyFrames(frames);
        QVector<Vector3D> oldControlPoints=controlPoints;
        QVector<Vector3D> oldNormals=normals;
        QVector<Vector3D> oldControlPointsRaised=controlPointsRaised;

        double ledge=freeMotionPar.bodyVel.length()*solvPar.tau*(i+1);
        Vector3D translation;

//             xEnd=0.0;
//        else
//            xEnd+=freeMotionPar.bodyVel.length()*solvPar.tau;
//        if (xEnd-bodyNose.x()>fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)
//            xEnd+=freeMotionPar.bodyVel.length()*solvPar.tau;

//        double ledge=freeMotionPar.bodyVel.length()*solvPar.tau*(i+1);
//        Vector3D translation;
//        if (ledge<fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)
//        {
//            translation=freeMotionPar.bodyVel*solvPar.tau*i;
//            FrameCalculations::translateBody(translation,frames,controlPoints,controlPointsRaised);
//        }
        translation=freeMotionPar.bodyVel*solvPar.tau;
//        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center, bodyNose, xEnd, fragPar);


        Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;


        dMatrix3 R;
        Eigen::Matrix3d rotation;
        Eigen::Matrix3d rotationNull;
        Vector3D angularVel;
        double xend;

        if (form.fullLength+bodyNose.x()>=0.0)
            xend=0.0;
        else
            xend=form.fullLength+bodyNose.x();
//        qDebug()<<xend;
//        qDebug()<<bodyNose.x()<<" "<<bodyNose.y()<<" "<<bodyNose.z();
        IntegrationResults results=functions.integrateParameters(xend-bodyNose.x(), 1700, fragmentation.getFormingRBC());
        if (fabs(ledge)<fullLength)
            functions.matrixCalc(frames,controlPoints,normals);


        QTime stepTime=QTime::currentTime();

        Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        emit sendMaxGamma(*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare));
        emit sendReguliser(vorticities[vorticities.size()-1]);

        newVortons.clear();
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        int generatedNum=newVortons.size();
        //FrameCalculations::translateVortons(translation,newVortons);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        QVector<Vorton> symNewVortons=newVortons;
        QVector<Vorton> symFreeVortons=freeVortons;
        QVector<std::shared_ptr<MultiFrame>> symFrames=FrameCalculations::copyFrames(frames);
        FrameCalculations::reflect(symFreeVortons,symNewVortons,symFrames);
//        Vector3D vel=FrameCalculations::velocity(Vector3D(0.0,0.2,0.1),solvPar.streamVel,freeVortons+newVortons+symFreeVortons+symNewVortons);
//        qDebug()<<vel.x()<<" "<<vel.y()<<" "<<vel.z();
//        Vector3D sumVel;

//        for (int i=0; i<freeVortons.size(); i++)
//        {
//            sumVel+=freeVortons[i].velocity(Vector3D(0.0,1.0,1.0));
//        }
//        for (int i=0; i<symFreeVortons.size(); i++)
//        {
//            sumVel+=symFreeVortons[i].velocity(Vector3D(0.0,1.0,1.0));
//        }
//        for (int i=0; i<frames.size(); i++)
//        {
//            sumVel+=frames[i]->velocity(Vector3D(0.0,1.0,1.0));
//        }
//        for (int i=0; i<symFrames.size(); i++)
//        {
//            sumVel+=symFrames[i]->velocity(Vector3D(0.0,1.0,1.0));
//        }

//        qDebug()<<sumVel.x()<<" "<<sumVel.y()<<" "<<sumVel.z();
        if (i==0)
        {
            dRFromEulerAngles(R,M_PI,0.0,0.0);
            for (int i=0; i<3; i++)
            {
                for (int j=0; j<3; j++)
                {
                    rotation(i,j)=R[i*4+j];
                    rotationNull(i,j)=rotation(i,j);
                }
            }
//            rotationNull.inverse();

        }
        functions.displacementLaunchCalc(freeVortons,newVortons,symFreeVortons, symNewVortons, solvPar.tau,relVel/*solvPar.streamVel*/,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        FrameCalculations::reflectMove(symFreeVortons,freeVortons);
        Vector3D torque;
        Vector3D force;
        functions.forceAndTorqueCalc(relVel, solvPar.streamPres,solvPar.density,frames+symFrames,freeVortons+symFreeVortons, solvPar.tau, squares, controlPointsRaised, normals,center,force,torque);
        forces[i]=force;
        torques[i]=torque;
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));


        functions.epsNormal(newVortons,fragPar.vortonsRad);
        functions.epsNormal(symNewVortons,fragPar.vortonsRad);
        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

//        for (int i=0; i<freeVortons.size();i++)
//        {
//            if (freeVortons[i].getMove().length()>3.0)
//                qDebug()<<freeVortons[i].getVorticity();
//        }
        functions.displace(freeVortons);
        functions.getBackAndRotateMovingLaunchedRotationCutBody(freeVortons,center,nullCenter, bodyNose, xend, solvPar.layerHeight, controlPoints,normals,rotation,rotationNull,fragmentation.getFormingRBC());
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);


        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        if (fabs(ledge)<fullLength)
        {
//            qDebug()<<fullLength;
//            qDebug()<<fabs(ledge);
            FrameCalculations::updateBoundaries(bodyNose,translation,center);

            exitStep++;
            center=bodyNose/2;
            nullCenter=center;
        }
        else {
//            qDebug()<<exitStep;
            Vector3D oldCenter=center;
            functions.translateAndRotatev3(frames,freeVortons,results.mass,results.inertiaTensor,torque,rotation,rotationNull,force,center,nullCenter, results.massCenter,solvPar.tau,freeMotionPar.bodyVel,controlPoints,normals,controlPointsRaised,oldFrames,oldControlPoints,oldNormals,oldControlPointsRaised, angularVel,bodyNose,xend);
//            bodyNose+=center-oldCenter;
            for (int i=0; i<freeVortons.size();i++)
            {
                if (freeVortons[i].getMid().x()<0.1)
                {
                            Vector3D vel=FrameCalculations::velocity(freeVortons[i].getMid(),solvPar.streamVel,freeVortons+newVortons+symFreeVortons+symNewVortons);
                            qDebug()<<vel.x()<<" "<<vel.y()<<" "<<vel.z();
                }
                }

        }
            //FrameCalculations::translateBody(translation,frames,controlPoints,controlPointsRaised,center,bodyNose,xEnd,fragPar);

//        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center, xBeg, xEnd, fragPar);
//        FrameCalculations::translateVortons(translation,freeVortons);
//        bodyNose+=translation;
        emit sendNormalsVis(controlPoints,normals);
        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration,functions.getConditionalNum());

        emit sendProgressRotationCutBody(i);
        emit repaintGUI(freeVortons/*+symFreeVortons*/, frames/*+symFrames*/);



    }
    logger->writeSolverTime(start.elapsed()*0.001);
    logger->closeFiles();
    delete logger;
}

/*!
Осуществляет вариацию для определения наилучших параметров сферы
\param fragPar Параметры разбиения сферы
\param variateEps Необходимость варьирования радиуса вортона
*/
void Solver::variateSphereParameters(FragmentationParameters fragPar, bool variateEps)
{

    FragmentationParameters resultFrag=fragPar;
    SolverParameters resultSolv=solvPar;

    FragmentationParameters initialFrag=fragPar;
    SolverParameters initiaLSolv=solvPar;


    double oldDispersion;
    double dispersion;

    Logger variateLogger(SPHERE,OPTIMIZATION);
    logPath=variateLogger.getPath();

    sphereSolver(fragPar);


    double frameAvLength=2*M_PI/std::max(fragPar.sphereFiFragNum,fragPar.sphereTetaFragNum);

    oldDispersion=FrameCalculations::calcDispersion(cAerodynamics);

    if (variateEps)
    {
        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps<1.5*frameAvLength; eps+=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            sphereSolver(fragPar);
            if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }

        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps>0.0; eps-=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            sphereSolver(fragPar);
            if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }
    }
    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau<2.0*frameAvLength/solvPar.streamVel.length(); tau+=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau>0.0; tau-=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es>0.0; es-=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es<1.5*resultFrag.vortonsRad; es+=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h<3.0*frameAvLength; h+=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h>0.0; h-=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup<2.0*frameAvLength; deltup+=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup>0.0; deltup-=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }


    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np<4.0*frameAvLength; np+=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np>0.0; np-=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        sphereSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    variateLogger.writePassport(resultSolv,resultFrag);
    variateLogger.closeFiles();
    logPath.clear();
    emit variatingFinished();
}

/*!
Осуществляет вариацию для определения наилучших параметров цилиндра
\param fragPar Параметры разбиения цилиндра
\param variateEps Необходимость варьирования радиуса вортона
*/
void Solver::variateCylinderParameters(FragmentationParameters fragPar, bool variateEps)
{
    FragmentationParameters resultFrag=fragPar;
    SolverParameters resultSolv=solvPar;

    FragmentationParameters initialFrag=fragPar;
    SolverParameters initiaLSolv=solvPar;


    double oldDispersion;
    double dispersion;

    Logger variateLogger(CYLINDER,OPTIMIZATION);
    logPath=variateLogger.getPath();

    cylinderSolver(fragPar);


    double frameAvLength=std::max({2*M_PI/fragPar.cylinderFiFragNum,2*M_PI/fragPar.cylinderRadFragNum,fragPar.cylinderHeight/fragPar.cylinderHeightFragNum});

    oldDispersion=FrameCalculations::calcDispersion(cAerodynamics);

    if (variateEps)
    {
        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps<1.5*frameAvLength; eps+=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            cylinderSolver(fragPar);
            if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }

        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps>0.0; eps-=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            cylinderSolver(fragPar);
            if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }
    }
    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau<2.0*frameAvLength/solvPar.streamVel.length(); tau+=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau>0.0; tau-=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es>0.0; es-=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es<1.5*resultFrag.vortonsRad; es+=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h<3.0*frameAvLength; h+=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h>0.0; h-=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup<2.0*frameAvLength; deltup+=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup>0.0; deltup-=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }


    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np<4.0*frameAvLength; np+=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np>0.0; np-=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        cylinderSolver(fragPar);
        if (!checkingVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    variateLogger.writePassport(resultSolv,resultFrag);
    variateLogger.closeFiles();
    logPath.clear();
    emit variatingFinished();
}

/*!
Осуществляет вариацию для определения наилучших параметров тела вращения
\param fragPar Параметры разбиения тела вращения
\param variateEps Необходимость варьирования радиуса вортона
*/
void Solver::variateRotationBodyParameters(FragmentationParameters fragPar, bool variateEps)
{
    FragmentationParameters resultFrag=fragPar;
    SolverParameters resultSolv=solvPar;

    FragmentationParameters initialFrag=fragPar;
    SolverParameters initiaLSolv=solvPar;


    double oldDispersion;
    double dispersion;

    Logger variateLogger(ROTATIONBODY,OPTIMIZATION);
    logPath=variateLogger.getPath();

    rotationBodySolver(fragPar);


    double frameAvLength=(fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)/fragPar.rotationBodyPartFragNum;
    oldDispersion=FrameCalculations::calcDispersion(cAerodynamics);

    if (variateEps)
    {
        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps<1.5*frameAvLength; eps+=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            rotationBodySolver(fragPar);
            if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }

        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps>0.0; eps-=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            rotationBodySolver(fragPar);
            if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }
    }
    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau<2.0*frameAvLength/solvPar.streamVel.length(); tau+=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau>0.0; tau-=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es>0.0; es-=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es<1.5*resultFrag.vortonsRad; es+=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h<3.0*frameAvLength; h+=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h>0.0; h-=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup<2.0*frameAvLength; deltup+=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup>0.0; deltup-=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }


    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np<4.0*frameAvLength; np+=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np>0.0; np-=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        rotationBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    variateLogger.writePassport(resultSolv,resultFrag);
    variateLogger.closeFiles();
    logPath.clear();
    emit variatingFinished();
}

/*!
Осуществляет вариацию для определения наилучших параметров тела вращения со срезом
\param fragPar Параметры разбиения тела вращения со срезом
\param variateEps Необходимость варьирования радиуса вортона
*/
void Solver::variateRotationCutBodyParameters(FragmentationParameters fragPar, bool variateEps)
{
    FragmentationParameters resultFrag=fragPar;
    SolverParameters resultSolv=solvPar;

    FragmentationParameters initialFrag=fragPar;
    SolverParameters initiaLSolv=solvPar;


    double oldDispersion;
    double dispersion;

    Logger variateLogger(ROTATIONBOTTOMCUT,OPTIMIZATION);
    logPath=variateLogger.getPath();

    rotationCutBodySolver(fragPar);


    double frameAvLength=(fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)/fragPar.rotationBodyPartFragNum;
    oldDispersion=FrameCalculations::calcDispersion(cAerodynamics);

    if (variateEps)
    {
        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps<1.5*frameAvLength; eps+=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            rotationCutBodySolver(fragPar);
            if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }

        for (double eps=initialFrag.vortonsRad+0.1*frameAvLength; eps>0.0; eps-=0.1*frameAvLength)
        {
            fragPar.vortonsRad=eps;
            rotationCutBodySolver(fragPar);
            if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
            {
                break;
            }
        }
    }
    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau<2.0*frameAvLength/solvPar.streamVel.length(); tau+=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double tau=initiaLSolv.tau+0.1*frameAvLength; tau>0.0; tau-=0.1*frameAvLength)
    {
        solvPar.tau=tau;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es>0.0; es-=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double es=initiaLSolv.eStar+0.1*resultFrag.vortonsRad; es<1.5*resultFrag.vortonsRad; es+=0.1*resultFrag.vortonsRad)
    {
        solvPar.eStar=es;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h<3.0*frameAvLength; h+=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double h=initiaLSolv.layerHeight+0.1*frameAvLength; h>0.0; h-=0.1*frameAvLength)
    {
        solvPar.layerHeight=h;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup<2.0*frameAvLength; deltup+=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double deltup=initiaLSolv.deltaUp+0.1*frameAvLength; deltup>0.0; deltup-=0.1*frameAvLength)
    {
        solvPar.deltaUp=deltup;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }


    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np<4.0*frameAvLength; np+=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    for (double np=initialFrag.pointsRaising+0.1*frameAvLength; np>0.0; np-=0.1*frameAvLength)
    {
        fragPar.pointsRaising=np;
        rotationCutBodySolver(fragPar);
        if (!checkingForceVariate(dispersion, oldDispersion, fragPar, resultFrag, resultSolv))
        {
            break;
        }
    }

    variateLogger.writePassport(resultSolv,resultFrag);
    variateLogger.closeFiles();
    logPath.clear();
    emit variatingFinished();
}

/*!
Оператор копирования для класса
\param solver Новое значение
*/
void Solver::operator =(const Solver &solver)
{
    solvPar=solver.solvPar;
    freeMotionPar=solver.freeMotionPar;
    cAerodynamics=solver.cAerodynamics;
    forces=solver.forces;
    torques=solver.torques;
}

Logger* Solver::createLog(BodyType type)
{
    if (logPath.isEmpty())
    {
        Logger* logger = new Logger(type);
        return  logger;
    }
    else
    {
        Logger* logger = new Logger(type,logPath);
        return logger;
    }
}

/*!
Функция проверки улучшения параметров для вариации (нахождение наименьшей дисперсии С)
\param dispersion Новая дисперсия С
\param oldDispersion Старая дисперсия С
\param fragPar Текущие значения параметров разбиения
\param resultFrag Лучшие значения параметров разбиения
\param resultSolv Лучшие значения параметров расчета
*/
bool Solver::checkingVariate(double& dispersion, double& oldDispersion, FragmentationParameters& fragPar, FragmentationParameters& resultFrag, SolverParameters& resultSolv)
{
    if (Solver::explosion==false)
    {
        dispersion=FrameCalculations::calcDispersion(cAerodynamics);
        if (dispersion<oldDispersion)
        {
            resultFrag=fragPar;
            resultSolv=solvPar;
            oldDispersion=dispersion;
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

/*!
Функция проверки улучшения параметров для вариации (нахождение наименьшей дисперсии силы)
\param dispersion Новая дисперсия силы
\param oldDispersion Старая дисперсия силы
\param fragPar Текущие значения параметров разбиения
\param resultFrag Лучшие значения параметров разбиения
\param resultSolv Лучшие значения параметров расчета
*/
bool Solver::checkingForceVariate(double &dispersion, double &oldDispersion, FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters &resultSolv)
{
    if (Solver::explosion==false)
    {
        dispersion=FrameCalculations::calcDispersion(forces);
        if (dispersion<oldDispersion)
        {
            resultFrag=fragPar;
            resultSolv=solvPar;
            oldDispersion=dispersion;
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}
