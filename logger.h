#ifndef LOGGER_H
#define LOGGER_H
#include <memory>
#include <QString>
#include <QFile>
#include "framecalculations.h"
#include <QMessageBox>
#include "solversettings.h"

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
    QString path; ///<Текущий путь записи каталога; если пуст - каталог сборки
    BodyType type; ///<Вид рассчитываемого тела
public:
    Logger();
    Logger(BodyType _type, SolvType _stype=NOOPTIMIZATION);
    Logger(BodyType _type, QString _path, SolvType _stype=NOOPTIMIZATION);
    void createFiles();
    void writeCpFile(const QVector<double> cp, const QVector<double> tetas);
    void writeLogs(const int stepNum, const double stepTime, const int freeVortonsSize, const Counters beforeIntegrC, const Counters afterIntegrC, const Timers beforeIntegrT, const Timers afterIntegrT, const Restrictions restr);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar);
    void writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar, const FreeMotionParameters& freeMotionPar);
    void writeForces(const Vector3D forces, const Vector3D c);
    void writeSolverTime(const double solvTime);
    void closeFiles();
    QString getPath();
};

#endif // LOGGER_H
