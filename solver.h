#ifndef SOLVER_H
#define SOLVER_H
#include <QObject>
#include "logger.h"

class Solver: public QObject
{
    Q_OBJECT
private:
    SolverParameters solvPar;
    QVector<Vector3D> cAerodynamics;

public:
    Solver();
    Solver(const SolverParameters parameters);
    void sphereSolver(const FragmentationParameters& fragPar);
    void cylinderSolver(const FragmentationParameters& fragPar);
    void rotationBodySolver(const FragmentationParameters& fragPar);
    void variateSphereParameters(FragmentationParameters& fragPar);

    void operator = (const Solver &solver);
    static bool explosion;
    bool checkingVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);
signals:
    void repaintGUI(const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>>& frames);
    void sendProgressSphere(const int percent);
    void updateSphereMaximum(const int max);
    void sendProgressCylinder(const int percent);
    void updateCylinderMaximum(const int max);
    void sendProgressRotationBody(const int percent);
    void updateRotationBodyMaximum(const int max);
};

#endif // SOLVER_H
