#include "solver.h"

bool Solver::interrupted=false;
bool Solver::explosion=false;

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

    for (int i=0; i<solvPar.stepsNum; i++)
    {

        QTime stepTime=QTime::currentTime();
        if(checkFinishing())
            return;
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
        functions.cpSum(i,solvPar.stepsNum, cp, fragPar.sphereFiFragNum, fragPar.sphereRad, fragPar.pointsRaising, tetas,solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, center);
        cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));
        forces[i]=force;

        freeVortons.append(newVortons);



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
        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);
        logger->writeVortons(frames,freeVortons,i);
        emit sendProgressSphere(i);
        emit repaintGUI(freeVortons, frames);

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
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);
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
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);

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


    functions.calcSection(fragmentation.getForming().diameter,center.x(),fragPar.rotationBodyFiFragNum,framesSection,centerSection,sectionNormals);

    pressures.resize(controlPointsRaised.size()+centerSection.size());
    velocities.resize(controlPoints.size()+centerSection.size());
    normalVelocities.resize(controlPoints.size()+centerSection.size());
    tangentialVelocities.resize(controlPoints.size()+centerSection.size());

    logger->writePassport(solvPar,fragPar,fragmentation.getForming(),functions.calcFrameSizes(frames));
    QVector<Vector3D> sectionVelocities(centerSection.size());
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
        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);

        int generatedNum=newVortons.size();
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,currentSpeed,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
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
        for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
            velocities[i]=FrameCalculations::velocity(centerSection[i-controlPoints.size()],solvPar.streamVel,freeVortons,frames);


        for (int i=0; i<controlPoints.size();i++)
            normalVelocities[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames),normals[i]);
        for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
            normalVelocities[i]=Vector3D::dotProduct(FrameCalculations::velocity(centerSection[i-controlPoints.size()],solvPar.streamVel,freeVortons,frames),sectionNormals[i-controlPoints.size()]);

        for (int i=0; i<controlPoints.size();i++)
            tangentialVelocities[i]=(velocities[i]-normalVelocities[i]*normals[i]).length();
        for (int i=controlPoints.size(); i<controlPoints.size()+centerSection.size();i++)
            tangentialVelocities[i]=(velocities[i]-normalVelocities[i]*sectionNormals[i-controlPoints.size()]).length();



        functions.cpSumRotationBody(i,solvPar.stepsNum, cp, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp0, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,0);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp90, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,90);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp180, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,180);
        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp270, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,270);
        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationBody(freeVortons, bodyNose, xend, solvPar.layerHeight, controlPoints,normals,fragmentation.getForming());
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
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);





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
    Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;
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

    for (int i=0; i<solvPar.stepsNum; i++)
    {
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


        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();
        if (i==0)
        {
            dRFromEulerAngles(R,M_PI,0.0,0.0);
            for (int i=0; i<3; i++)
                for (int j=0; j<3; j++)
                    rotation(i,j)=R[i*4+j];
        }

        functions.displace(freeVortons);
        functions.getBackAndRotateMovingRotationBody(freeVortons,center+results.massCenter, bodyNose,xend, solvPar.layerHeight, controlPoints,normals,rotation,fragmentation.getForming());
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();
        Eigen::Matrix3d inertia;
        inertia<<1,0,0,0,1,0,0,0,1;


//        qDebug()<<"Center is "<<center.x()<<" "<< center.y()<<" "<<center.z();
        functions.translateAndRotate(frames,freeVortons,results.mass,results.inertiaTensor,torque,rotation,force,center,results.massCenter,(i+1)*solvPar.tau,freeMotionPar.bodyVel,controlPoints,normals,controlPointsRaised,oldFrames,oldControlPoints,oldNormals,oldControlPointsRaised, angularVel);
        functions.clear();
//        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center,bodyNose, xend, fragPar);
 //       FrameCalculations::translateVortons(translation, freeVortons);
        emit sendNormalsVis(controlPoints,normals);
        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);

        logger->createParaviewTraceFile(freeVortons,i);
        logger->createParaviewFile(frames,pressures,velocities,tangentialVelocities,normalVelocities,framesSection,i);
        emit sendProgressRotationBody(i);
        emit repaintGUI(freeVortons, frames);


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
    QVector<double> normalVelocities;

    QVector<std::shared_ptr<MultiFrame>> framesSection;
    QVector<Vector3D> centerSection;
    QVector<Vector3D> sectionNormals;
    logger->writePassport(solvPar,fragPar,fragmentation.getForming(),functions.calcFrameSizes(frames));
    //functions.calcSection(fragmentation.getForming().diameter,center.x(),fragPar.rotationBodyFiFragNum,framesSection,centerSection,sectionNormals);

    pressures.resize(controlPointsRaised.size()+centerSection.size());
    velocities.resize(controlPoints.size()+centerSection.size());
    normalVelocities.resize(controlPoints.size()+centerSection.size());
    tangentialVelocities.resize(controlPoints.size()+centerSection.size());
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

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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

        for (int i=0; i<controlPoints.size();i++)
            normalVelocities[i]=Vector3D::dotProduct(FrameCalculations::velocity(controlPoints[i],solvPar.streamVel,freeVortons,frames),normals[i]);

        for (int i=0; i<controlPoints.size();i++)
            tangentialVelocities[i]=(velocities[i]-normalVelocities[i]*normals[i]).length();


//        functions.cpSumRotationBody(i,solvPar.stepsNum, cp, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised);
//        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp0, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,0);
//        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp90, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,90);
//        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp180, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,180);
//        functions.cpRotationBodyDegree(i,solvPar.stepsNum, cp270, fragPar.rotationBodyFiFragNum, solvPar.streamVel, solvPar.streamPres,solvPar.density,frames, freeVortons, solvPar.tau, controlPointsRaised,270);


        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationCutBody(freeVortons, xend, solvPar.layerHeight, controlPoints,normals, bodyNose,forming);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);

        logger->createParaviewTraceFile(freeVortons,i);
        logger->createParaviewFile(frames,pressures,velocities,tangentialVelocities,normalVelocities,/*framesSection,*/i);
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
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);

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

        FrameCalculations functions;
        functions.matrixCalc(frames,controlPoints,normals);
        Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;
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
            //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

            freeVortons.append(newVortons);

            Counters countersBeforeIntegration=functions.getCounters();
            Timers timersBeforeIntegration=functions.getTimers();
            Restrictions restrictions=functions.getRestrictions();
            functions.clear();

            functions.displace(freeVortons);
            functions.getBackAndRotateRotationCutBody(freeVortons, xend, solvPar.layerHeight, controlPoints,normals, bodyNose,forming);
            functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
            functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
            functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

            Counters countersAfterIntegration=functions.getCounters();
            Timers timersAfterIntegration=functions.getTimers();

            functions.clear();
            FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center,bodyNose, xend, fragPar);
            FrameCalculations::translateVortons(translation, freeVortons);
            emit sendNormalsVis(controlPoints,normals);
            logger->writeForces(force,Vector3D(0.0,0.0,0.0));
            logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
            logger->writeVortons(frames,freeVortons,i);
            logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                               FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);

            emit sendProgressRotationCutBody(i);
            emit repaintGUI(freeVortons, frames);


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
    double xEnd=0.0;
    logger->writePassport(solvPar,fragPar);
    BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar, true);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        if(checkFinishing())
            return;
        fragmentation.rotationCutBodyLaunchFragmentation(i,freeMotionPar.bodyVel,solvPar.tau);
        QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
        QVector<Vector3D> normals=fragmentation.getNormals();

        QVector<double> squares=fragmentation.getSquares();
        QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
        QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();
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
        //center=bodyNose/2;

        Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;

        FrameCalculations functions;
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
        //FzrameCalculations::translateVortons(translation,newVortons);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        QVector<Vorton> symNewVortons=newVortons;
        QVector<Vorton> symFreeVortons=freeVortons;
        QVector<std::shared_ptr<MultiFrame>> symFrames=FrameCalculations::copyFrames(frames);
        FrameCalculations::reflect(symFreeVortons,symNewVortons,symFrames);

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
        functions.displacementLaunchCalc(freeVortons,newVortons,symFreeVortons, symNewVortons, solvPar.tau,relVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        FrameCalculations::reflectMove(symFreeVortons,freeVortons);
        Vector3D force=functions.forceCalc(relVel, solvPar.streamPres,solvPar.density,frames+symFrames,freeVortons+symFreeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        forces[i]=force;

        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        for (int i=0; i<freeVortons.size();i++)
        {
            if (freeVortons[i].getMove().length()>3.0)
                qDebug()<<freeVortons[i].getVorticity();
        }
        functions.displace(freeVortons);
        functions.getBackAndRotateRotationCutLaunchedBody(freeVortons, bodyNose, xEnd, solvPar.layerHeight, controlPoints,normals,fragmentation.getFormingRBC());
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);


        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        if (ledge<fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)
            FrameCalculations::updateBoundaries(bodyNose,translation,center);
        else
            FrameCalculations::translateBody(translation,frames,controlPoints,controlPointsRaised,center,bodyNose,xEnd,fragPar);

//        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center, xBeg, xEnd, fragPar);
        FrameCalculations::translateVortons(translation,freeVortons);
//        bodyNose+=translation;
        emit sendNormalsVis(controlPoints,normals);
        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
        logger->writeTable(i,stepTime.elapsed()*0.001,generatedNum,*std::max_element(vorticities.data(),vorticities.data()+vorticities.size(),Vector3D::fabsCompare),
                           FrameCalculations::velocity(center,solvPar.streamVel,freeVortons,frames),vorticities[vorticities.size()-1],freeVortons.size(),countersBeforeIntegration,countersAfterIntegration);

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
