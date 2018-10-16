#ifndef LOGGER_H
#define LOGGER_H
#include <memory>
#include <QString>
#include <QFile>
#include "framecalculations.h"
#include <QMessageBox>
#include "solversettings.h"

enum SolvType {OPTIMIZATION, NOOPTIMIZATION};

class Logger:public QObject
{
    Q_OBJECT
private:
    std::shared_ptr<QFile> logFile;
    std::shared_ptr<QTextStream> logTextStream;
    std::shared_ptr<QFile> passportFile;
    std::shared_ptr<QTextStream> passportTextStream;
    std::shared_ptr<QFile> forcesFile;
    std::shared_ptr<QTextStream> forcesTextStream;
    std::shared_ptr<QFile> cpFile;
    std::shared_ptr<QTextStream> cpTextStream;
    QString path;
    BodyType type;
public:
    Logger(BodyType _type, SolvType _stype=NOOPTIMIZATION);
    Logger(BodyType _type, QString _path, SolvType _stype=NOOPTIMIZATION);
    void createFiles();
    void writeCpFile(const QVector<double> cp, const QVector<double> tetas);
    void writeLogs(const int stepNum, const double stepTime, const Counters beforeIntegrC, const Counters afterIntegrC, const Timers beforeIntegrT, const Timers afterIntegrT, const Restrictions restr);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar, const FreeMotionParameters& freeMotionPar);
    void writeForces(const Vector3D forces, const Vector3D c);
    void writeSolverTime(const double solvTime);
    void closeFiles();
    QString getPath();
};

#endif // LOGGER_H
