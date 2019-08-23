#ifndef LOGGER_H
#define LOGGER_H
#include <memory>
#include <QString>
#include <QFile>
#include "framecalculations.h"
#include <QMessageBox>
#include "solversettings.h"
#include "QFileDialog"
#include <dos.h>
#include "qcustomplot.h"

/** \file logger.h
    \brief Заголовочный файл для описания классов и перечистлений для работы с файлами 
*/

/**
 * Виды расчета.
*/
enum SolvType {
    OPTIMIZATION, ///< Вариация переменных
    NOOPTIMIZATION ///<Обычный расчет
};

/*!
    \brief Класс, созданный для записи результатов расчетов

    Создает файлы для записи начальных параметров расчета(паспорт-файл), логов, сил и распределения Ср
*/

struct TriangleVectors
{
    Vector3D a;
    Vector3D b;
    Vector3D c;
    Vector3D center;
    Vector3D normal;
    double square;

    void calcParameters();
};

class Logger:public QObject
{
    Q_OBJECT
private:
    std::shared_ptr<QFile> logFile; ///<Файл содержащий записи логов расчета
    std::shared_ptr<QTextStream> logTextStream;  ///<Поток для записи логов в файл
    std::shared_ptr<QFile> passportFile;///<Файл, содержащий начальные параметры расчета
    std::shared_ptr<QTextStream> passportTextStream; ///<Поток для записи начальных параметров в файл
    std::shared_ptr<QFile> forcesFile; ///<Файл содержащий рассчитанные силы и аэродинамические характеристики
    std::shared_ptr<QTextStream> forcesTextStream; ///<Поток для записи рассчитанных силы и аэродинамических характеристик
    std::shared_ptr<QFile> cpFile; ///<Файл содержащий распределение Ср
    std::shared_ptr<QTextStream> cpTextStream; ///<Поток для записи распределения Ср
    std::shared_ptr<QFile> tableFile;
    std::shared_ptr<QTextStream> tableTextStream;
    std::shared_ptr<QFile> gammaFile;
    std::shared_ptr<QTextStream> gammaTextStream;

    QString path; ///<Текущий путь записи каталога; если пуст - каталог сборки
    BodyType type; ///<Вид рассчитываемого тела
public:
    Logger();
    Logger(BodyType _type, SolvType _stype=NOOPTIMIZATION);
    Logger(BodyType _type, QString _path, SolvType _stype=NOOPTIMIZATION);
    std::shared_ptr<QPair<QVector<QVector<Vorton>>, QVector<QVector<Vorton>>>> loadKadrDir(const QString vortonsDir);
    void writeVortons(QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freevortons, const int stepNum);

    void createFiles();

    void parseSalomeMesh(const QString& path, double vortonsRad, QVector<TriangleFrame> &frames, QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals, QVector<double>& squares);
    void writeCpGraphs(QVector<QVector<double>>& cpArray, QVector<double>& fis);
    void writeNormals(QVector<Vector3D>& controlPoints, QVector<Vector3D>& normals);
    void writeCpFile(const QVector<double> cp, const QVector<double> tetas);
    void writeCpDegreeFile(const QVector<double> cp, const QVector<double> tetas, const int degree);
    void writeLogs(const int stepNum, const double stepTime, const int freeVortonsSize, const Counters beforeIntegrC, const Counters afterIntegrC, const Timers beforeIntegrT, const Timers afterIntegrT, const Restrictions restr);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar);
    void writePassport(const SolverParameters& solvPar, const FragmentationParameters& fragPar, const FormingParameters forming, const FramesSizes framesSizes, double maxGammFactor=0.0);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar, const FreeMotionParameters& freeMotionPar);
    void writeForces(const Vector3D forces, const Vector3D c);
    void writeSolverTime(const double solvTime);
    void writeTable(const int stepNum, const double stepTime, const double generatedNum, const double maxGamma, const Vector3D velocity, const double reguliser, const int freeVortonsSize, const Counters beforeIntegrC, const Counters afterIntegrC, const double conditionalNum, double quant2=0);
    void createParaviewFile(QVector<std::shared_ptr<MultiFrame>> &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocities, QVector<std::shared_ptr<MultiFrame> > &sectionFrames, int currentStep);
    void createParaviewFile(QVector<std::shared_ptr<MultiFrame>> &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocities, int currentStep);
    void createParaviewFile(QVector<std::shared_ptr<MultiFrame>> &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocitiesBefore, QVector<double> &normalVelocitiesAfter, QVector<double> &normalVelocitiesEnd, QVector<std::shared_ptr<MultiFrame> > &sectionFrames, int currentStep);
    void createParaviewFile(QVector<std::shared_ptr<MultiFrame>> &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocitiesAfter, QVector<double> &normalVelocitiesBefore, QVector<double> &normalVelocitiesCenter, QVector<double> &normalVelocitiesEnd, QVector<std::shared_ptr<MultiFrame> > &sectionFrames, int currentStep);

    void createParaviewFile(QVector<std::shared_ptr<MultiFrame>> &frames, QVector<double> &forces, QVector<double> &normalVelocitiesAfter, QVector<double> &normalVelocitiesBefore, QVector<double> &normalVelocitiesCenter, QVector<double> &normalVelocitiesEnd, int currentStep);

    void createParaviewStreamlinesFile(QVector<Vector3D> velocities, QPair<int, int> boundary, double step, int currentStep);
    void createParaviewTraceVerticesFile(QVector<Vorton> &vortons, int currentStep);
    void createParaviewVelocityField(QVector<Vorton> &vortons, double tau, int currentStep);
    void createParaviewVelocityField(QVector<Vorton> &vortons, QVector<Vector3D>& moveFromFrames, QVector<Vector3D>& moveFromVortons, QVector<Vector3D> &moveFromGetBack, double tau, int currentStep);

    void createParaviewTraceFile(QVector<Vorton> &vortons, int currentStep);
    void createCenterGraphs(FormingParameters pars, double step, int currentStep, QVector<Vorton> &freeVortons, Vector3D velInf, QVector<std::shared_ptr<MultiFrame> > &xFrames,QVector<std::shared_ptr<MultiFrame> > &yFrames,QVector<std::shared_ptr<MultiFrame> > &zFrames);
    void createCenterGraphs(FormingParameters pars, double step, int currentStep, QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, Vector3D velInf, QVector<std::shared_ptr<MultiFrame> > &xFrames, QVector<std::shared_ptr<MultiFrame> > &yFrames, QVector<std::shared_ptr<MultiFrame> > &zFrames, QVector<std::shared_ptr<MultiFrame> > &frames, int inter);
    void writeGammas(QVector<double> gammaMax, double initialGamma);
    void createQuadroGraphs(int currentStep, QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, Vector3D velInf, QVector<QVector<Vector3D> > &graphNodesX, QVector<QVector<Vector3D> > &graphNodesY, QVector<QVector<Vector3D> > &graphNodesZ, QVector<std::shared_ptr<MultiFrame> > &frames, int inter);

    QVector<Vorton> gaVortons(const QString vortonsDir, int currentFileNum);
    void closeFiles();
    QString getPath();
    void openVortonFiles(QString vortonsDir);

signals:
    void sendVortons(const QVector<Vorton>&, const QVector<Vorton>&);


};

#endif // LOGGER_H
