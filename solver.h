#ifndef SOLVER_H
#define SOLVER_H
#include <QObject>
#include "logger.h"

/*!
    \brief Класс, реализующий расчеты для различных типов тел

    Главный класс, содержащий все расчетчики.
*/
class Solver: public QObject
{
    Q_OBJECT
private:
    SolverParameters solvPar; ///<Параметры расчета
    FreeMotionParameters freeMotionPar; ///<Параметры свободного движения
    QVector<Vector3D> cAerodynamics; ///<Вектор значений С
    QVector<Vector3D> forces; ///<Вектор значений силы
    QString logPath; ///<Путь к каталогу для записи результатов расчета
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
    void operator = (const Solver &solver);
    static bool explosion;///<Существование "взрыва"
    bool checkingVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);
    bool checkingForceVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);
signals:
    void repaintGUI(const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>>& frames); ///<Сигнал о перерисовке интерфейса
    void variatingFinished(); ///<Сигнал об окончании  варьирования
    void sendProgressSphere(const int percent); ///< Сигнал об текущем прогрессе расчета сферы
    void updateSphereMaximum(const int max); ///<Сигнал о количестве шагов расчета сферы
    void sendProgressCylinder(const int percent); ///< Сигнал об текущем прогрессе расчета цилиндра
    void updateCylinderMaximum(const int max); ///<Сигнал о количестве шагов расчета цилиндра
    void sendProgressRotationBody(const int percent); ///< Сигнал об текущем прогрессе расчета тела вращения
    void updateRotationBodyMaximum(const int max); ///<Сигнал о количестве шагов расчета тела вращения
    void sendProgressRotationCutBody(const int percent); ///< Сигнал об текущем прогрессе расчета тела вращения со срезом
    void updateRotationCutBodyMaximum(const int max); ///<Сигнал о количестве шагов расчета тела вращения со срезом
};

#endif // SOLVER_H
