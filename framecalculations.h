#ifndef FRAMECALCULATIONS_H
#define FRAMECALCULATIONS_H
#include <QtConcurrent/QtConcurrent>
#include <QTime>
#include <ode/ode.h>
#include "simpsonintegration.h"

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
    double tau; ///< � азмер шага
};

struct ParallelPassive
{
    QVector<Vorton> *vortons;
    Vorton centralVort;
    Vector3D streamVel;
    double tau;
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

struct Inside
{
    int res;
    Vector3D rtilda;
    int num;
};

/*!
    \brief Класс, содержащий функции для работы с рамками

    Класс, в котором определены все функции, описывающие работу с рамками при проведении расчета
*/

class FrameCalculations
{
private:

    Eigen::MatrixXd matrix; ///<Матрица, составленная из произведения единичных интенсивностей от каждой из рамок на соответствующую нормаль. �?спользуется для дальнейшего решения СЛАУ и вычисления завихренностей рамок.
    int matrixSize;  ///<� азмер матрицы
    double conditionalNum;
    Counters counters;  ///<Количество удаленных вортонов в результате действия различных функций

    Restrictions restrictions;  ///<Количество сработанных ограничений

    Timers timers;  ///<Таймеры на функции

public:
    FrameCalculations();
    double getConditionalNum();
    QVector<double> calcTetas(const int tetaFragNum);
    void epsZero(QVector<std::shared_ptr<MultiFrame>> &frames);
    void epsNormal(QVector<Vorton>& newVortons, double eps);
    void epsNormal(QVector<std::shared_ptr<MultiFrame>> &frames, double eps);
    void matrixCalc(QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    Eigen::VectorXd columnCalc(const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<Vector3D> &normals, const QVector<Vector3D> controlPoints);
    Eigen::VectorXd columnCalc(const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<Vector3D> &normals, const Vector3D angularVel, const QVector<Vector3D>& controlPoints, const Vector3D center);
    Eigen::VectorXd vorticitiesCalc(const Eigen::VectorXd& column);
    static int universalInside(const Vorton vort, const QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame>>& frames);
    static Inside universalInsideCorrect(const Vorton vort, const QVector<std::pair<double, double> > boundaries, QVector<std::shared_ptr<MultiFrame>>& frames);
    QVector<int> universalGetBack(QVector<Vorton> &vortons, QVector<std::pair<double, double> > boundaries, const double layerHeight,  const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame>>& frames,bool screen = false);
    QVector<int> universalGetBackTriangle(QVector<Vorton> &vortons, QVector<std::pair<double, double> > boundaries, const double layerHeight,  const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame>>& frames,bool screen = false);

    void correctMove(QVector<Vorton>& freeVortons, QVector<Vorton>& copyVort);
    void universalRotate(QVector<Vorton> vortons, QVector<int> res, const double layerHeight,  const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void universalRotateTriangle(QVector<Vorton> vortons, QVector<int> res, const double layerHeight,  const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<std::shared_ptr<MultiFrame> > &frames);

    void unionVortons(QVector<Vorton> &vortons, const double eStar, const double eDoubleStar,const  double vortonRad);
    void removeSmallVorticity(QVector<Vorton> &vortons,const double minVorticity);
    void removeFarSphere(QVector <Vorton> &vortons, const double farDistance, const Vector3D bodyCenter);
    void removeFarCylinder(QVector<Vorton> &vortons, const double farDistance, const double height);
    void removeFarRotationBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter);
    void removeFarRotationCutBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter);
    void displacementCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove);
    void displacementPassiveCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons,QVector<Vorton> &frameVortons,double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove);
    void displacementCalcGauss3(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove, double dlMax, double dlMin);
    void displacementLaunchCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons,QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove);
    void setMatrixSize(int size);
    void getBackAndRotate(bool getBackGA, QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames, FormingParameters forming);
     void getBackAndRotate(bool getBackGA, QVector<Vorton> &vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames);
    void getBackAndRotate(bool getBackGA,QVector<Vorton> &vortons, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParametersRBC forming);
    IntegrationResults integrateParameters(double length, double bodyDensity, FormingParameters parameters);
    IntegrationResults integrateParameters(double length, double bodyDensity, FormingParametersRBC parameters);
    void calcSection(double diameter, double xCenter, int fiFragNum, QVector<std::shared_ptr<MultiFrame> >& sectionFrames, QVector<Vector3D> &centerFrames, QVector<Vector3D>& sectionNormals);
    FramesSizes calcFrameSizes(QVector<std::shared_ptr<MultiFrame>> frames);
    Vector3D forceCalc(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vorton>& freeVortons, const double tau,
                       const QVector<double>& squares , const QVector <Vector3D>& controlPointsRaised, const QVector <Vector3D>& normals);
    Vector3D forceSetuha(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vorton>& freeVortons, const double tau,
                       const QVector<double>& squares , const QVector <Vector3D>& controlPointsRaised, const QVector <Vector3D>& normals,QVector<double>&gammas);
    void forceAndTorqueCalc(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vorton>& freeVortons, const double tau,
                            const QVector<double>& squares , const QVector <Vector3D>& controlPointsRaised, const QVector <Vector3D>& normals, Vector3D center, Vector3D& force, Vector3D& tongue);
    void cpSum(const int stepNum, int stepsQuant, QVector<double>& cp, const int fiFragNum, const double radius, const double pointsRaising, const QVector<double> &tetas, const Vector3D streamVel,
                    const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const Vector3D center);
    void cpSumCylinder(const int stepNum, int stepsQuant, QVector<double>& cp, const int fiFragNum, const double diameter, const double height, const double pointsRaising, const Vector3D streamVel,
                       const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau);
    void cpSumRotationBody(const int stepNum, int stepsQuant, QVector<double>& cp, const int fiFragNum, const Vector3D streamVel,
                       const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised);
    void cpRotationBodyDegree(const int stepNum, int stepsQuant, QVector<double>& cp, const int fiFragNum, const Vector3D streamVel,
                              const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised, int degree);
    void cpSumRotationCutBody(const int stepNum, int stepsQuant, QVector<double>& cp, const int fiFragNum, const int rFragNum,const Vector3D streamVel,
                       const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised);
    void cpRotationCutBodyDegree(const int stepNum, int stepsQuant, QVector<double>& cp, const int fiFragNum, const int rFragNum, const Vector3D streamVel,
                              const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D> &controlPointsRaised, int degree);
    void cpAverage(QVector<double>& cp, const int stepsNum);
    void getBackAndRotateSphere(QVector<Vorton>& vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateSphereGA(QVector<Vorton>& vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames);
    void getBackAndRotateCylinder(QVector<Vorton>& vortons, const double height, const double diameter, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateRotationBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming);
    void getBackAndRotateRotationBodyv2(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming);
    void getBackAndRotateMovingRotationBody(QVector<Vorton> &vortons, Vector3D centerMassWorld,Vector3D oldCenterWorld, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, Eigen::Matrix3d rotationMatrix, FormingParameters forming);
    void getBackAndRotateMovingRotationCutBody(QVector<Vorton> &vortons, Vector3D centerMassWorld,Vector3D oldCenterWorld, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, Eigen::Matrix3d rotationMatrix, FormingParametersRBC forming);
    void getBackAndRotateMovingLaunchedRotationCutBody(QVector<Vorton> &vortons, Vector3D centerMassWorld, Vector3D oldCenterWorld, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, Eigen::Matrix3d rotationMatrix, Eigen::Matrix3d rotationNullMatrix, FormingParametersRBC forming);
    void getBackAndRotateRotationCutBody(QVector<Vorton> &vortons, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParametersRBC forming);
    void getBackAndRotateRotationCutBodyGA(QVector<Vorton> &vortons, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParametersRBC forming);
    void getBackAndRotateRotationBodyGA(QVector<Vorton> &vortons, QVector<Vorton> &oldvortons, QVector<std::shared_ptr<MultiFrame> > &frames, const double xEnd, const Vector3D bodyNose, FormingParameters forming, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals);
    void getBackAndRotateRotationCutLaunchedBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParametersRBC forming);
    void translateAndRotate(QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vorton>& vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel,
                            QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals, QVector<Vector3D>& controlPointsRaised, QVector<std::shared_ptr<MultiFrame>>& oldFrames, QVector<Vector3D>& oldControlPoints, QVector<Vector3D>& oldNormals, QVector<Vector3D>& oldControlPointsRaised , Vector3D &angVel);
    void translateAndRotatev2(QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vorton>& vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel,
                            QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals, QVector<Vector3D>& controlPointsRaised, QVector<std::shared_ptr<MultiFrame>>& oldFrames, QVector<Vector3D>& oldControlPoints, QVector<Vector3D>& oldNormals, QVector<Vector3D>& oldControlPointsRaised , Vector3D &angVel, Vector3D &bodyNose, double &xend);
    void translateAndRotatev3(QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vorton>& vortons, double mass, Eigen::Matrix3d inertiaTensor, Vector3D tongue, Eigen::Matrix3d &rotationMatrix, Eigen::Matrix3d &rotationNullMatrix,Vector3D force, Vector3D &center, Vector3D nullCenter, Vector3D massCenter, double time, Vector3D linearVel,
                            QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals, QVector<Vector3D>& controlPointsRaised, QVector<std::shared_ptr<MultiFrame>>& oldFrames, QVector<Vector3D>& oldControlPoints, QVector<Vector3D>& oldNormals, QVector<Vector3D>& oldControlPointsRaised , Vector3D &angVel, Vector3D &bodyNose, double &xend);
    void displace(QVector<Vorton>& vortons, double dlMax, double dlMin);
    void getBackRotationBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming);
    void rotateRotationBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd,const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming);
    void velForStreamLines(QVector<Vector3D>& velocities, Vector3D streamVel, double step, QVector<Vorton>& freeVortons, QPair<int, int> boundaries);
    static bool coDirectionallyCheck(const Vector3D a, const Vector3D b, const Vector3D c);

    static void setVorticity(QVector<std::shared_ptr<MultiFrame>> frames, const Eigen::VectorXd vorticities);
    static QVector<Vorton> getFrameVortons(QVector<std::shared_ptr<MultiFrame>> frames);
    static QVector<Vorton> getLiftedFrameVortons (QVector<std::shared_ptr<MultiFrame>> frames, const QVector<Vector3D>& normals, const double deltaUp);
    static double pressureCalc(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density,QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau);
    static double pressureSetuha(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density, QVector<std::shared_ptr<MultiFrame>> frames, QVector<double> oldGammas, QVector<Vorton> freeVortons, double tau);
    static void addToVortonsVec(QVector<Vorton> &vortons, const Vorton vort);
    static VelBsym velocityAndBsymm(const Vector3D point,const Vector3D streamVel, const QVector<Vorton>& vortons);
    static VelBsym velocityAndBsymmGauss3(const Vector3D point,const Vector3D deltar, const Vector3D streamVel, const QVector<Vorton>& vortons);
    static Vector3D velocity(const Vector3D point ,const Vector3D streamVel, const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>> frames);
    static Vector3D velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons);
    static Vorton parallelDisplacement(const Parallel el);
    static Vorton parallelDisplacementPassive(const ParallelPassive el);
    static Vorton parallelDisplacementGauss(const Parallel el);
    //static Lengths lengthsCalc(QVector<std::shared_ptr<MultiFrame>> frames);
    static void displace(QVector<Vorton> &vortons);
    static void reflect(QVector<Vorton>& symFreeVortons, QVector<Vorton>& symNewVortons, QVector<std::shared_ptr<MultiFrame> > &symFrames);
    static void reflectMove(QVector<Vorton>& symFreeVortons, QVector<Vorton> &freeVortons);
    static bool insideSphere(const Vorton& vort, const Vector3D& center,const double radius);
    static bool insideSphereLayer(const Vorton& vort, const Vector3D& center,const double radius, const double layerHeight);
    static bool insideCylinder(const Vorton& vort, const double height, const double diameter);
    static bool insideCylinderLayer(const Vorton& vort, const double height, const double diameter, const double layerHeight);
    static bool insideRotationBody(const Vorton& vort, const Vector3D bodyNose, const double xEnd, FormingParameters forming);
    static bool insideRotationBodyLayer(const Vorton& vort, const Vector3D bodyNose, const double xEnd, const double layerHeight, FormingParameters forming);
    static int insideRotationBodyv2(const Vorton& vort, const Vector3D bodyNose, const double xEnd, FormingParameters forming);
    static int insideRotationBodyLayerv2(const Vorton& vort, const Vector3D bodyNose, const double xEnd, const double layerHeight, FormingParameters forming);
    static bool insideRotationCutBody(const Vorton& vort, const double xBeg, const double xEnd, FormingParametersRBC forming);
    static bool insideRotationCutBody(const Vorton& vort, const double xEnd, const Vector3D bodyNose, FormingParametersRBC forming);
    static bool insideRotationCutBodyLayer(const Vorton& vort, const double xEnd, const double layerHeight, const Vector3D bodyNose, FormingParametersRBC forming);
    static bool insideScreen(Vorton &vort);
    static bool exploseSphere(const QVector<Vorton>& vortons);
    static double calcDispersion(const QVector<Vector3D> &cAerodynamics);
    static QVector<std::shared_ptr<MultiFrame>> copyFrames(QVector<std::shared_ptr<MultiFrame>> frames);
    static void translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vector3D>& controlPoints, QVector<Vector3D>& controlPointsRaised, Vector3D& center, Vector3D &bodyNose, double &xend, const FragmentationParameters &fragPar);
    static void translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vector3D>& controlPoints, QVector<Vector3D>& controlPointsRaised, Vector3D& center);
    static void translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>>& frames, QVector<Vector3D>& controlPoints, QVector<Vector3D>& controlPointsRaised);
    static void updateBoundaries(Vector3D& bodynose, Vector3D& translation, Vector3D& center);
    static void translateVortons(const Vector3D &translation, QVector<Vorton>& vortons);
    static int findMaxSolidAngle(Vector3D point, QVector<std::shared_ptr<MultiFrame>>& frames);
    Counters getCounters() const;
    Restrictions getRestrictions() const;
    Timers getTimers() const;
    void clearCounters();
    int getMatrixSize();
    void clearRestrictions();
    void clearTimers();
    void clear();
    QVector<std::pair<double, double> > makeParalllepiped(QVector<Vorton> newVortons);
    static bool xCompare(const Vorton a, const Vorton b);
    static bool yCompare(Vorton a,Vorton b);
    static bool zCompare(Vorton a, Vorton b);

};

Eigen::Vector3d toEigenVector(Vector3D vec);
Vector3D fromEigenVector(Eigen::Vector3d vec);
#endif // FRAMECALCULATIONS_H
