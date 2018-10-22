#ifndef SOLVER_H
#define SOLVER_H
#include <QObject>
#include "logger.h"

class Solver: public QObject
{
    Q_OBJECT
private:
    SolverParameters solvPar;
    FreeMotionParameters freeMotionPar;
    QVector<Vector3D> cAerodynamics;
    QVector<Vector3D> forces;
    QString logPath;
public:
    Solver();
    Solver(const SolverParameters& parameters);
    Solver(const SolverParameters& parameters, const FreeMotionParameters& motionParameters);
    void sphereSolver(const FragmentationParameters& fragPar);
    void sphereFreeMotionSolver(const FragmentationParameters& fragPar);
    void cylinderSolver(const FragmentationParameters& fragPar);
    void rotationBodySolver(const FragmentationParameters& fragPar);
    void rotationCutBodySolver(const FragmentationParameters& fragPar);
    void rotationCutBodySolverNearScreen(const FragmentationParameters& fragPar, const double screenDistance);
    void rotationCutBodyFreeMotionSolver(const FragmentationParameters &fragPar);
    void rotationCutBodyLaunchSolver(const FragmentationParameters& fragPar);
    void variateSphereParameters(FragmentationParameters fragPar, bool variateEps);
    void variateCylinderParameters(FragmentationParameters fragPar, bool variateEps);
    void variateRotationBodyParameters(FragmentationParameters fragPar, bool variateEps);
    void variateRotationCutBodyParameters(FragmentationParameters fragPar, bool variateEps);
    void reflect(QVector<Vorton>& symFreeVortons, QVector<Vorton>& symNewVortons, QVector<std::shared_ptr<MultiFrame>> symFrames);
    void operator = (const Solver &solver);
    static bool explosion;
    bool checkingVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);
    bool checkingForceVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);
signals:
    void repaintGUI(const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>>& frames);
    void variatingFinished();
    void sendProgressSphere(const int percent);
    void updateSphereMaximum(const int max);
    void sendProgressCylinder(const int percent);
    void updateCylinderMaximum(const int max);
    void sendProgressRotationBody(const int percent);
    void updateRotationBodyMaximum(const int max);
    void sendProgressRotationCutBody(const int percent);
    void updateRotationCutBodyMaximum(const int max);
};

#endif // SOLVER_H
