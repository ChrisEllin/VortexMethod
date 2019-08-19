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
    QVector<Vector3D> torques;
    QString logPath; ///<Путь к каталогу для записи результатов расчета
    MotionType motion;
    bool concentration=false;
    bool checkFinishing();
public:
    static bool interrupted;
    static bool getBackGA;
    Solver();
    void setConcentration(bool conc);
    BodyFragmentation fragmentate(const FragmentationParameters parameters, const BodyType type);
    void setMotionType(const MotionType type);
    Solver(const SolverParameters& parameters);
    Solver(const SolverParameters& parameters, const FreeMotionParameters& motionParameters);
    void sphereSolver(const FragmentationParameters& fragPar);
    void sphereFreeMotionSolver(const FragmentationParameters& fragPar);
    void cylinderSolver(const FragmentationParameters& fragPar);
    void rotationBodySolver(const FragmentationParameters& fragPar);
    void ovalSolver();
    void ringsSolver();
    void rotationBodyFreeMotionSolver(const FragmentationParameters &fragPar);
    void rotationCutBodySolver(const FragmentationParameters& fragPar);
    void rotationCutBodySolverNearScreen(const FragmentationParameters& fragPar, const double screenDistance);
    void rotationCutBodyFreeMotionSolver(const FragmentationParameters &fragPar);
    void rotationCutBodyLaunchSolver(const FragmentationParameters& fragPar);
    void variateSphereParameters(FragmentationParameters fragPar, bool variateEps);
    void variateCylinderParameters(FragmentationParameters fragPar, bool variateEps);
    void variateRotationBodyParameters(FragmentationParameters fragPar, bool variateEps);
    void variateRotationCutBodyParameters(FragmentationParameters fragPar, bool variateEps);
    void operator = (const Solver &solver);
    Logger *createLog(BodyType type);
    static bool explosion;///<Существование "взрыва"
    bool checkingVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);
    bool checkingForceVariate(double& dispersion, double& oldDispersion,FragmentationParameters &fragPar, FragmentationParameters &resultFrag, SolverParameters& resultSolv);

    void updateMaximum(int steps, BodyType type);
    void updateProgress(int step, BodyType type);
    void unifiedSolver(const FragmentationParameters &fragPar, const BodyType type);
signals:
    void sendReguliser(double reguliser);
    void finishSolver();
    void makeScreenShot(QString screenDir);
    void sendNormalsVis(QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals);
    void sendMaxGamma(double maxGamma);
    void repaintGUI(const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>>& frames);
    void repaintGUI(const QVector<Vorton>& vortons, const QVector<Vorton>& gaVortons);///<Сигнал о перерисовке интерфейса
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
