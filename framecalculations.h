#ifndef FRAMECALCULATIONS_H
#define FRAMECALCULATIONS_H
#include <QtConcurrent/QtConcurrent>
#include <QTime>
#include <eigen-eigen-5a0156e40feb/Eigen/Dense>
#include <eigen-eigen-5a0156e40feb/Eigen/LU>
#include "bodyfragmentation.h"

/** \file framecalculations.h
    \brief Заголовочный файл для описания классов, структур и перечислений для работы с рамками
*/

/*!
    \brief Контейнер, представляющий собой структуру для параллельных вычислений

    Для дальнейшего распараллеливания с использованием QtConcurrent создается структура, хранящая в себе указатель на вектор вортонов, указатель на вектор свободных вортонов, указатель на вектор рамок, номер текущей рамки для вычислений, скорость потока и размер шага
*/
struct Parallel
{
    QVector<Vorton> *Vortons; ///< Указатель на вектор вортонов
    QVector<Vorton> *freeVortons;  ///< Указатель на вектор вортонов в потоке
    QVector<std::shared_ptr<MultiFrame>> *frames; ///< Указатель на вектор рамок
    int num; ///< Номер текущей рамки
    Vector3D streamVel; ///< Скорость потока
    double tau; ///< Размер шага
};

//struct Lengths
//{
//    double minLength;
//    double maxLength;
//    double averLength;
//    double lengthDeviation;
//};


/*!
    \brief Контейнер, представляющий собой структуру для хранения счетчиков
*/
struct Counters
{
    int unitedNum; ///<Количество объединенных вортонов
    int vorticityEliminated; ///<Количество удаленных вортонов по причине малой завихренности
    int tooFarNum; ///<Количество удаенных вортонов по причине большой дальности
    int rotatedNum; ///<Количество вортонов развернутых относительно поверзности тела в слое
    int underScreenNum; ///<Количество вортонов попавших под экран
    int gotBackNum; ///<Количество вортонов, возвращенных из тела в поток
    void clear();
};

/*!
    \brief Контейнер, представляющий собой структуру для хранения сработавших ограничений
*/
struct Restrictions
{
    int moveRestr; ///<Количество сработавших ограничений на поворот
    int elongationRestr; ///<Количество сработавших ограничений на удлинение
    int turnRestr; ///<Количество сработавших ограничений на поворот
    void clear();
};

/*!
    \brief Контейнер, представляющий собой структуру для хранения таймеров
*/
struct Timers
{
    double integrationTimer; ///<Время, затраченное на расчет перемещений и удлинений
    double unionTimer; ///<Время, затраченное на объединение вортонов
    double removeVorticityTimer; ///<Время, затраченное на удаление вортонов по причине малой завихренности
    double forceTimer; ///<Время, затраченное на расчет величин действующих на тело сил
    double farTimer; ///<Время, затраченное на удаление вортонов по причине большой дальности
    double getBackAndRotateTimer; ///<Время, затраченное на возвращение вортонов из тела в поток и на разворот вортонов относительно поверхности в слое
    void clear();
};

/*!
    \brief Класс, содержащий функции для работы с рамками

    Класс, в котором определены все функции, описывающие работу с рамками при проведении расчета
*/

class FrameCalculations
{
private:

    Eigen::MatrixXd matrix; ///<Матрица, составленная из произведения единичных интенсивностей от каждой из рамок на соответствующую нормаль. Используется для дальнейшего решения СЛАУ и вычисления завихренностей рамок.
    int matrixSize;  ///<Размер матрицы

    Counters counters;  ///<Количество удаленных вортонов в результате действия различных функций

    Restrictions restrictions;  ///<Количество сработанных ограничений

    Timers timers;  ///<Таймеры на функции

public:
    FrameCalculations();

    QVector<double> calcTetas(const int tetaFragNum);
    void matrixCalc(QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    Eigen::VectorXd columnCalc(const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<Vector3D> &normals, const QVector<Vector3D> controlPoints);
    Eigen::VectorXd vorticitiesCalc(const Eigen::VectorXd& column);
    void unionVortons(QVector<Vorton> &vortons, const double eStar, const double eDoubleStar,const  double vortonRad);
    void removeSmallVorticity(QVector<Vorton> &vortons,const double minVorticity);
    void removeFarSphere(QVector <Vorton> &vortons, const double farDistance, const Vector3D bodyCenter);
    void removeFarCylinder(QVector<Vorton> &vortons, const double farDistance, const double height);
    void removeFarRotationBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter);
    void removeFarRotationCutBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter);
    void displacementCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove);
    void displacementLaunchCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons,QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove);
    void setMatrixSize(int size);
    Vector3D forceCalc(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vorton>& freeVortons, const double tau,
                       const QVector<double>& squares , const QVector <Vector3D>& controlPointsRaised, const QVector <Vector3D>& normals);
    void cpSum(const int stepNum, QVector<double>& cp, const int fiFragNum, const double radius, const double pointsRaising, const QVector<double> &tetas, const Vector3D streamVel,
                    const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const Vector3D center);
    void cpAverage(QVector<double>& cp, const int stepsNum);
    void getBackAndRotateSphere(QVector<Vorton>& vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateCylinder(QVector<Vorton>& vortons, const double height, const double diameter, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateRotationBody(QVector<Vorton> &vortons, const double xBeg, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateRotationCutBody(QVector<Vorton> &vortons, const double xBeg, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateRotationCutLaunchedBody(QVector<Vorton> &vortons, const double xBeg, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    static void setVorticity(QVector<std::shared_ptr<MultiFrame>> frames, const Eigen::VectorXd vorticities);
    static QVector<Vorton> getFrameVortons(QVector<std::shared_ptr<MultiFrame>> frames);
    static QVector<Vorton> getLiftedFrameVortons (QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vector3D>& normals, const double deltaUp);
    static double pressureCalc(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density,QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau);
    static void addToVortonsVec(QVector<Vorton> &vortons, const Vorton vort);
    static VelBsym velocityAndBsymm(const Vector3D point,const Vector3D streamVel, const QVector<Vorton>& vortons);
    static Vector3D velocity(const Vector3D point ,const Vector3D streamVel, const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>> frames);
    static Vector3D velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons);
    static Vorton parallelDisplacement(const Parallel el);
    //static Lengths lengthsCalc(QVector<std::shared_ptr<MultiFrame>> frames);
    static void displace(QVector<Vorton> &vortons);
    static void reflect(QVector<Vorton>& symFreeVortons, QVector<Vorton>& symNewVortons, QVector<std::shared_ptr<MultiFrame>> symFrames);
    static bool insideSphere(const Vorton& vort, const Vector3D& center,const double radius);
    static bool insideSphereLayer(const Vorton& vort, const Vector3D& center,const double radius, const double layerHeight);
    static bool insideCylinder(const Vorton& vort, const double height, const double diameter);
    static bool insideCylinderLayer(const Vorton& vort, const double height, const double diameter, const double layerHeight);
    static bool insideRotationBody(const Vorton& vort, const double xBeg, const double xEnd);
    static bool insideRotationBodyLayer(const Vorton& vort, const double xBeg, const double xEnd, const double layerHeight);
    static bool insideRotationCutBody(const Vorton& vort, const double xBeg, const double xEnd);
    static bool insideRotationCutBodyLayer(const Vorton& vort, const double xBeg, const double xEnd, const double layerHeight);
    static bool insideScreen(Vorton &vort);
    static bool exploseSphere(const QVector<Vorton>& vortons);
    static double calcDispersion(const QVector<Vector3D> &cAerodynamics);

    static QVector<std::shared_ptr<MultiFrame>> copyFrames(QVector<std::shared_ptr<MultiFrame>> frames);
    static void translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vector3D>& controlPoints, QVector<Vector3D>& controlPointsRaised, Vector3D& center, double &xbeg, double &xend, const FragmentationParameters &fragPar);
    static void translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vector3D>& controlPoints, QVector<Vector3D>& controlPointsRaised, Vector3D& center);
    static void translateVortons(const Vector3D &translation, QVector<Vorton>& vortons);
    Counters getCounters() const;
    Restrictions getRestrictions() const;
    Timers getTimers() const;
    void clearCounters();
    void clearRestrictions();
    void clearTimers();
    void clear();
};

#endif // FRAMECALCULATIONS_H
