#ifndef FRAMECALCULATIONS_H
#define FRAMECALCULATIONS_H
#include <QtConcurrent/QtConcurrent>
#include <QTime>
#include <eigen-eigen-5a0156e40feb/Eigen/Dense>
#include <eigen-eigen-5a0156e40feb/Eigen/LU>
#include "bodyfragmentation.h"

struct Parallel
{
    QVector<Vorton> *Vortons;
    QVector<Vorton> *freeVortons;
    QVector<std::shared_ptr<MultiFrame>> *frames;
    int num;
    Vector3D streamVel;
    double tau;
};

struct Lengths
{
    double minLength;
    double maxLength;
    double averLength;
    double lengthDeviation;
};

struct Counters
{
    int unitedNum;
    int vorticityEliminated;
    int tooFarNum;
    int rotatedNum;
    int underScreenNum;
    int gotBackNum;
    void clear();
};

struct Restrictions
{
    int moveRestr;
    int elongationRestr;
    int turnRestr;
    void clear();
};

struct Timers
{
    double integrationTimer;
    double unionTimer;
    double removeVorticityTimer;
    double forceTimer;
    double farTimer;
    double getBackAndRotateTimer;
    void clear();
};

class FrameCalculations
{
private:

    Eigen::MatrixXd matrix;
    int matrixSize;

    //количество удаленных вортонов в результате действия различных функций
    Counters counters;

    //количество сработанных ограничений
    Restrictions restrictions;

    //таймеры на функции
    Timers timers;

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
    static Lengths lengthsCalc(QVector<std::shared_ptr<MultiFrame>> frames);
    static void displace(QVector<Vorton> &vortons);
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
