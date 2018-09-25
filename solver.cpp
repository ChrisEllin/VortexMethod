#include "solver.h"
bool Solver::explosion=false;

Solver::Solver()
{

}

Solver::Solver(const SolverParameters& parameters)
{
    solvPar=parameters;
    cAerodynamics.resize(solvPar.stepsNum);
}

Solver::Solver(const SolverParameters &parameters, const FreeMotionParameters &motionParameters)
{
    solvPar=parameters;
    freeMotionPar=motionParameters;
    cAerodynamics.resize(solvPar.stepsNum);
}

void Solver::sphereSolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Solver::explosion=false;
    Logger logger(BodyType::SPHERE);
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
    emit updateSphereMaximum(solvPar.stepsNum-1);

    logger.writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(solvPar.streamVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

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
        logger.writeForces(force,cAerodynamics[i]);
        logger.writeLogs(i,stepTime.elapsed()*0.001,countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);

        emit sendProgressSphere(i);
        emit repaintGUI(freeVortons, frames);

        if (FrameCalculations::exploseSphere(freeVortons))
        {
            qDebug()<<"Explose";
            Solver::explosion=true;
            return;
        }
    }
    logger.writeSolverTime(start.elapsed()*0.001);
    logger.closeFiles();
}

void Solver::cylinderSolver(const FragmentationParameters& fragPar)
{
    QTime start=QTime::currentTime();
    Logger logger(BodyType::CYLINDER);
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
    emit updateCylinderMaximum(solvPar.stepsNum-1);

    logger.writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(solvPar.streamVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

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

        logger.writeLogs(i,stepTime.elapsed()*0.001,countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);

        emit sendProgressCylinder(i);
        emit repaintGUI(freeVortons, frames);


    }
    logger.writeSolverTime(start.elapsed()*0.001);
    logger.closeFiles();
}

void Solver::rotationBodySolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Logger logger(BodyType::ROTATIONBODY);
    BodyFragmentation fragmentation(BodyType::ROTATIONBODY, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationBodyMaximum(solvPar.stepsNum-1);

    logger.writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(solvPar.streamVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationBody(freeVortons, fragPar.rotationBodyXBeg, fragPar.rotationBodyXEnd, solvPar.layerHeight, controlPoints,normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger.writeLogs(i,stepTime.elapsed()*0.001,countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);

        emit sendProgressRotationBody(i);
        emit repaintGUI(freeVortons, frames);


    }
    logger.writeSolverTime(start.elapsed()*0.001);
    logger.closeFiles();
}

void Solver::rotationCutBodySolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Logger logger(BodyType::ROTATIONBOTTOMCUT);
    BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar);
    QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
    QVector<Vector3D> normals=fragmentation.getNormals();
    QVector<double> squares=fragmentation.getSquares();
    QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
    QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

    FrameCalculations functions;
    functions.matrixCalc(frames,controlPoints,normals);

    Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);

    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

    logger.writePassport(solvPar,fragPar);
    for (int i=0; i<solvPar.stepsNum; i++)
    {
        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(solvPar.streamVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        functions.displacementCalc(freeVortons,newVortons,solvPar.tau,solvPar.streamVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(solvPar.streamVel, solvPar.streamPres,solvPar.density,frames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationCutBody(freeVortons, fragPar.rotationBodyXBeg, fragPar.rotationBodyXEnd, solvPar.layerHeight, controlPoints,normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        logger.writeLogs(i,stepTime.elapsed()*0.001,countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);

        emit sendProgressRotationCutBody(i);
        emit repaintGUI(freeVortons, frames);


    }
    logger.writeSolverTime(start.elapsed()*0.001);
    logger.closeFiles();
}

void Solver::rotationCutBodyFreeMotionSolver(const FragmentationParameters &fragPar)
{
        QTime start=QTime::currentTime();
        Logger logger(BodyType::ROTATIONBOTTOMCUT);
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

        logger.writePassport(solvPar,fragPar);
        for (int i=0; i<solvPar.stepsNum; i++)
        {
            QTime stepTime=QTime::currentTime();
            newVortons.clear();
            Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,controlPoints);
            Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
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
            functions.getBackAndRotateRotationCutBody(freeVortons, fragPar.rotationBodyXBeg, fragPar.rotationBodyXEnd, solvPar.layerHeight, controlPoints,normals);
            functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
            functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
            functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

            Counters countersAfterIntegration=functions.getCounters();
            Timers timersAfterIntegration=functions.getTimers();

            functions.clear();

            FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center);
            FrameCalculations::translateVortons(translation, freeVortons);

            logger.writeLogs(i,stepTime.elapsed()*0.001,countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);

            emit sendProgressRotationCutBody(i);
            emit repaintGUI(freeVortons, frames);


        }
        logger.writeSolverTime(start.elapsed()*0.001);
        logger.closeFiles();
}

void Solver::rotationCutBodyLaunchSolver(const FragmentationParameters &fragPar)
{
    QTime start=QTime::currentTime();
    Logger logger(BodyType::ROTATIONBOTTOMCUT);


    Vector3D center((fragPar.rotationBodyXBeg+fragPar.rotationBodyXEnd)*0.5,0.0,0.0);
    QVector<Vorton> freeVortons;
    QVector<Vorton> newVortons;
    emit updateRotationCutBodyMaximum(solvPar.stepsNum-1);

    logger.writePassport(solvPar,fragPar);
    BodyFragmentation fragmentation(BodyType::ROTATIONBOTTOMCUT, fragPar, true);
    for (int i=0; i<solvPar.stepsNum; i++)
    {

        fragmentation.rotationCutBodyLaunchFragmentation(i,freeMotionPar.bodyVel,solvPar.tau);
        QVector<Vector3D> controlPoints=fragmentation.getControlPoints();
        QVector<Vector3D> normals=fragmentation.getNormals();
        QVector<double> squares=fragmentation.getSquares();
        QVector<Vector3D> controlPointsRaised=fragmentation.getControlPointsRaised();
        QVector<std::shared_ptr<MultiFrame>> frames=fragmentation.getFrames();

        Vector3D translation=freeMotionPar.bodyVel*solvPar.tau*(i+1);
        Vector3D relVel=solvPar.streamVel-freeMotionPar.bodyVel;
        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center);

        FrameCalculations functions;
        functions.matrixCalc(frames,controlPoints,normals);


        QTime stepTime=QTime::currentTime();
        newVortons.clear();
        Eigen::VectorXd column=functions.columnCalc(relVel,freeVortons,normals,controlPoints);
        Eigen::VectorXd vorticities=functions.vorticitiesCalc(column);
        FrameCalculations::setVorticity(frames,vorticities);

        newVortons=FrameCalculations::getLiftedFrameVortons(frames,normals,solvPar.deltaUp);
        functions.unionVortons(newVortons,solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(newVortons,solvPar.minVorticity);

        QVector<Vorton> symNewVortons=newVortons;
        QVector<Vorton> symFreeVortons=freeVortons;
        QVector<std::shared_ptr<MultiFrame>> symFrames=FrameCalculations::copyFrames(frames);
        reflect(symFreeVortons,symNewVortons,symFrames);

        functions.displacementLaunchCalc(freeVortons,newVortons,symFreeVortons, symNewVortons, solvPar.tau,relVel,solvPar.eDelta,solvPar.fiMax,solvPar.maxMove);
        Vector3D force=functions.forceCalc(relVel, solvPar.streamPres,solvPar.density,frames+symFrames,freeVortons, solvPar.tau, squares, controlPointsRaised, normals);
        //cAerodynamics[i] = force/(solvPar.density*solvPar.streamVel.lengthSquared()*0.5*M_PI*pow(fragPar.sphereRad,2));

        freeVortons.append(newVortons);

        Counters countersBeforeIntegration=functions.getCounters();
        Timers timersBeforeIntegration=functions.getTimers();
        Restrictions restrictions=functions.getRestrictions();
        functions.clear();

        functions.displace(freeVortons);
        functions.getBackAndRotateRotationCutBody(freeVortons, fragPar.rotationBodyXBeg, fragPar.rotationBodyXEnd, solvPar.layerHeight, controlPoints,normals);
        functions.unionVortons(freeVortons, solvPar.eStar,solvPar.eDoubleStar,fragPar.vortonsRad);
        functions.removeSmallVorticity(freeVortons,solvPar.minVorticity);
        functions.removeFarRotationCutBody(freeVortons,solvPar.farDistance,center);

        Counters countersAfterIntegration=functions.getCounters();
        Timers timersAfterIntegration=functions.getTimers();

        functions.clear();

        FrameCalculations::translateBody(translation, frames, controlPoints, controlPointsRaised, center);
        FrameCalculations::translateVortons(translation, freeVortons);

        logger.writeLogs(i,stepTime.elapsed()*0.001,countersBeforeIntegration,countersAfterIntegration, timersBeforeIntegration, timersAfterIntegration, restrictions);

        emit sendProgressRotationCutBody(i);
        emit repaintGUI(freeVortons+symFreeVortons, frames+symFrames);


    }
    logger.writeSolverTime(start.elapsed()*0.001);
    logger.closeFiles();
}

void Solver::variateSphereParameters(FragmentationParameters fragPar)
{

    FragmentationParameters resultFrag=fragPar;
    SolverParameters resultSolv=solvPar;

    FragmentationParameters initialFrag=fragPar;
    SolverParameters initiaLSolv=solvPar;


    double oldDispersion;
    double dispersion;


    sphereSolver(fragPar);
    Logger variateLogger;


    double frameAvLength=2*M_PI/std::max(fragPar.sphereFiFragNum,fragPar.sphereTetaFragNum);

    oldDispersion=FrameCalculations::calcDispersion(cAerodynamics);

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
    emit variatingFinished();
}

void Solver::reflect(QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, QVector<std::shared_ptr<MultiFrame> > symFrames)
{
    for (int i=0; i<symFreeVortons.size(); i++)
    {
        symFreeVortons[i].setMid(Vector3D(-symFreeVortons[i].getMid().x(),symFreeVortons[i].getMid().y(),symFreeVortons[i].getMid().z()));
        symFreeVortons[i].setTail(Vector3D(-symFreeVortons[i].getTail().x(),symFreeVortons[i].getTail().y(),symFreeVortons[i].getTail().z()));
    }

    for (int i=0; i<symNewVortons.size(); i++)
    {
        symNewVortons[i].setMid(Vector3D(-symNewVortons[i].getMid().x(),symNewVortons[i].getMid().y(),symNewVortons[i].getMid().z()));
        symNewVortons[i].setTail(Vector3D(-symNewVortons[i].getTail().x(),symNewVortons[i].getTail().y(),symNewVortons[i].getTail().z()));
    }

    for (int i=0; i<symFrames.size(); i++)
    {
        for (int j=0; j<symFrames[i]->getAnglesNum(); j++)
        {
            symFrames[i]->at(j).setMid(Vector3D(-symFrames[i]->at(j).getMid().x(),symFrames[i]->at(j).getMid().y(),symFrames[i]->at(j).getMid().z()));
            symFrames[i]->at(j).setTail(Vector3D(-symFrames[i]->at(j).getTail().x(),symFrames[i]->at(j).getTail().y(),symFrames[i]->at(j).getTail().z()));
        }
    }
}



void Solver::operator =(const Solver &solver)
{
    solvPar=solver.solvPar;
    freeMotionPar=solver.freeMotionPar;
    cAerodynamics=solver.cAerodynamics;
}

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
