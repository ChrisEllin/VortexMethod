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

/*!
Создает экземпляр класса для расчета неподвижного тела с заданными параметрами
\param parameters Параметры расчета
*/
Solver::Solver(const SolverParameters& parameters)
{
    solvPar=parameters;
    cAerodynamics.resize(solvPar.stepsNum);
    forces.resize(solvPar.stepsNum);
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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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
        logger->writeVortons(frames,freeVortons,i);
        emit sendProgressSphere(i);
        emit repaintGUI(freeVortons, frames);

        if (FrameCalculations::exploseSphere(freeVortons))
        {
            qDebug()<<"Explose";
            Solver::explosion=true;
            return;
        }
    }
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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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

        logger->writeForces(force,cAerodynamics[i]);
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);
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
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();
    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    QVector<double> cp(fragPar.cylinderFiFragNum);

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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg,0.0,0.0);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationBodyMaximum(solvPar.stepsNum-1);

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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        forces[i]=force;
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationBody(freeVortons, bodyNose, fragPar.rotationBodyXEnd, solvPar.layerHeight, controlPoints,normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();
        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);

        emit sendProgressRotationBody(i);
        emit repaintGUI(freeVortons, frames);


    }
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

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);
    Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;
    Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
    Vector3D translation=freeMotionPar.bodyVel*solvPar.tau;

    double xEnd=fragPar.rotationBodyXEnd;
    Vector3D bodyNose=Vector3D(fragPar.rotationBodyXBeg,0.0,0.0);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationBodyMaximum(solvPar.stepsNum-1);

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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,relVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(relVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        forces[i]=force;
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationBody(freeVortons, bodyNose, fragPar.rotationBodyXEnd, solvPar.layerHeight, controlPoints,normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();
        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center,bodyNose, xEnd, fragPar);
        FrameCalculations::translateVortons(translation, freeVortons);
        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001, freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);

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
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);


    Vector3D bodyNose=Vector3D(-2.5+fragPar.rotationBodyXEnd,0.0,0.0);
    Vector3D center((bodyNose.x()+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));
        forces[i]=force;

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationCutBody(freeVortons, 0.0, solvPar.layerHeight, controlPoints,normals, bodyNose);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(), countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);

        emit sendProgressRotationCutBody(i);
        emit repaintGUI(freeVortons, frames);


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
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    Vector3D center((fragPar.rotationBodyXEnd+fragPar.rotationBodyXBeg)*0.5,0.0,0.0);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

    Vector3D bodyNose(-(/*fragPar.rotationBodyXEnd+*/screenDistance),0.0,0.0);
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
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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
        functions.getBackAndRotateRotationCutLaunchedBody(freeVortons, bodyNose, -screenDistance, solvPar.layerHeight, controlPoints,normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);

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
        Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
        Vector3D translation=freeMotionPar.bodyVel*solvPar.tau;
        QVector<Vorton> freeVortons;
        QVector<Vorton> newVortons;
        emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

        double xBeg=fragPar.rotationBodyXBeg;
        double xEnd=0.0;
        Vector3D bodyNose=Vector3D(-2.5,0.0,0.0);
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
            FrameCalculations::setVorticity(frames,vorticities);

            newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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
            functions.getBackAndRotateRotationCutBody(freeVortons, xEnd, solvPar.layerHeight, controlPoints,normals, bodyNose);
            functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
            functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
            functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

            Counters countersAfterIntegration=functions.getCounters();
            Timers timersAfterIntegration=functions.getTimers();

            functions.clear();
            FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center,bodyNose, xEnd, fragPar);
            FrameCalculations::translateVortons(translation, freeVortons);

            logger->writeForces(force,Vector3D(0.0,0.0,0.0));
            logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
            logger->writeVortons(frames,freeVortons,i);

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

        newVortons.clear();
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
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
        functions.getBackAndRotateRotationCutLaunchedBody(freeVortons, bodyNose, xEnd, solvPar.layerHeight, controlPoints,normals);
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

        logger->writeForces(force,Vector3D(0.0,0.0,0.0));
        logger->writeLogs(i,stepTime.elapsed()*0.001,freeVortons.size(),countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);
        logger->writeVortons(frames,freeVortons,i);

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
