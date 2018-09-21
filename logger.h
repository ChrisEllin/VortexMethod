#ifndef LOGGER_H
#define LOGGER_H
#include <memory>
#include <QString>
#include <QFile>
#include "framecalculations.h"
#include <QMessageBox>
#include "solversettings.h"


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
    QString path;
    BodyType type;
public:
    Logger();
    Logger(BodyType _type);
    void createFiles();
    void writeLogs(const int stepNum, const double stepTime, const Counters beforeIntegrC, const Counters afterIntegrC, const Timers beforeIntegrT, const Timers afterIntegrT, const Restrictions restr);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar);
    void writeForces(const Vector3D forces, const Vector3D c);
    void writeSolverTime(const double solvTime);
    void closeFiles();
};

#endif // LOGGER_H
