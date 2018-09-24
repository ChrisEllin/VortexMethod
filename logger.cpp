#include "logger.h"

Logger::Logger()
{
    path = QCoreApplication::applicationDirPath();
    QDir folder(path);
    folder.cdUp();
    path=folder.absolutePath();
    path.append("/Results");
    folder.mkdir("Results");
    createFiles();
}

Logger::Logger(const BodyType _type)
{
    type=_type;
    path = QCoreApplication::applicationDirPath();
    QDir folder(path);
    folder.cdUp();
    path=folder.absolutePath();
    switch (type)
    {
    case SPHERE:
    {
        path.append("/Sphere_");
        path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
        folder.mkdir("Sphere_"+QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
        break;
    }
    case CYLINDER:
    {
        path.append("/Cylinder_");
        path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
        folder.mkdir("Cylinder_"+QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
        break;
    }
    case ROTATIONBODY:
    {
        path.append("/RotationBody_");
        path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
        folder.mkdir("RotationBody_"+QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        path.append("/RotationBottomCut_");
        path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
        folder.mkdir("RotationBottomCut_"+QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
        break;
    }
    default:
    {
        QMessageBox::critical(new QWidget(), tr("Ошибка"), tr("Попытка записи несоответствующего тела"));
        exit(1);
    }
    }
    createFiles();
}

void Logger::createFiles()
{
    logFile=std::shared_ptr<QFile>(new QFile(path+"/logs.txt"));
    passportFile=std::shared_ptr<QFile>(new QFile(path+"/passport.txt"));
    forcesFile=std::shared_ptr<QFile>(new QFile(path+"/forces.csv"));

    if (logFile->open(QIODevice::WriteOnly))
    {
        logTextStream=std::shared_ptr<QTextStream>(new QTextStream(logFile.get()));
        *logTextStream.get()<<"Расчет запущен в "+QTime::currentTime().toString("H:m:s a")+"\n\n";
    }
    else
    {
        QMessageBox::critical(new QWidget(),tr("Ошибка"), tr("Не удалось создать лог-файл"));
        exit(1);
    }

    if (passportFile->open(QIODevice::WriteOnly))
    {
        passportTextStream=std::shared_ptr<QTextStream>(new QTextStream(passportFile.get()));
        *passportTextStream.get()<<"Паспорт создан в "+QTime::currentTime().toString("H:m:s a")+"\n\n";

    }
    else
    {
        QMessageBox::critical(new QWidget(),tr("Ошибка"), tr("Не удалось создать пасспорт-файл"));
        exit(1);
    }

    if (forcesFile->open(QIODevice::WriteOnly))
    {
        forcesTextStream=std::shared_ptr<QTextStream>(new QTextStream(forcesFile.get()));
        *forcesTextStream.get()<<"Файл для сил создан в "+QTime::currentTime().toString("H:m:s a")+"\n\n";
        *forcesTextStream.get()<<QString("Cила (x) \t");
        *forcesTextStream.get()<<QString("Сила (y) \t");
        *forcesTextStream.get()<<QString("Сила (z) \t");
        *forcesTextStream.get()<<QString("Аэродинамический коэффициент (x) \t");
        *forcesTextStream.get()<<QString("Аэродинамический коэффициент (y) \t");
        *forcesTextStream.get()<<QString("Аэродинамический коэффициент (z) \n");
    }
    else
    {
        QMessageBox::critical(new QWidget(),tr("Ошибка"), tr("Не удалось создать файл для записи сил"));
        exit(1);
    }
}

void Logger::writeLogs(const int stepNum, const double stepTime, const Counters beforeIntegrC, const Counters afterIntegrC, const Timers beforeIntegrT, const Timers afterIntegrT, const Restrictions restr)
{
    *logTextStream.get()<<"Начата запись шага расчета №"+QString::number(stepNum)+" \n\n";
    *logTextStream.get()<<"С рамок объединилось "+QString::number(beforeIntegrC.unitedNum)+" вортонов \n";
    *logTextStream.get()<<"По гамме c рамок удалилось "+QString::number(beforeIntegrC.vorticityEliminated)+" вортонов \n";
    *logTextStream.get()<<"Из фигуры назад возвращено "+QString::number(afterIntegrC.gotBackNum)+" вортонов \n";
    *logTextStream.get()<<"Из слоя развернуто относительно поверхности "+QString::number(afterIntegrC.rotatedNum)+" вортонов \n";
    *logTextStream.get()<<"В слое объединилось "+QString::number(afterIntegrC.unitedNum)+" вортонов \n";
    *logTextStream.get()<<"В слое удалилось по гамме "+QString::number(afterIntegrC.vorticityEliminated)+" вортонов \n";
    *logTextStream.get()<<"По причине большой дальности удалено"+QString::number(afterIntegrC.tooFarNum)+" вортонов \n\n";

    *logTextStream.get()<<"Ограничение на перемещение сработало "+QString::number(restr.moveRestr)+" раз\n";
    *logTextStream.get()<<"Ограничение на поворот сработало "+QString::number(restr.turnRestr)+" раз\n";
    *logTextStream.get()<<"Ограничение на удлинение сработало "+QString::number(restr.elongationRestr)+" раз\n\n";

    *logTextStream.get()<<"Объединение вортонов с рамок заняло "+QString::number(beforeIntegrT.unionTimer)+" с.\n";
    *logTextStream.get()<<"Удаление по гамме вортонов с рамок заняло "+QString::number(beforeIntegrT.removeVorticityTimer)+" с.\n";
    *logTextStream.get()<<"Расчет перемещений и удлинений занял "+QString::number(beforeIntegrT.integrationTimer)+" с.\n";
    *logTextStream.get()<<"Расчет сил занял "+QString::number(beforeIntegrT.forceTimer)+" с.\n";
    *logTextStream.get()<<"Разворот и возвращение в поток заняло "+QString::number(afterIntegrT.getBackAndRotateTimer)+" с.\n";
    *logTextStream.get()<<"Объединение вортонов в слое заняло "+QString::number(afterIntegrT.unionTimer)+" с.\n";
    *logTextStream.get()<<"Удаление по гамме вортонов с рамок заняло "+QString::number(afterIntegrT.removeVorticityTimer)+" с.\n";
    *logTextStream.get()<<"Удаление вортонов по причине большой дальности заняло "+QString::number(afterIntegrT.farTimer)+" с.\n";
    *logTextStream.get()<<"Шаг №"+QString::number(stepNum)+" занял "+QString::number(stepTime)+" с.\n\n";
    *logTextStream.get()<<"Закончена запись шага расчета №"+QString::number(stepNum)+"\n\n";
    logTextStream.get()->flush();
}

void Logger::writePassport(const SolverParameters& solvPar,const FragmentationParameters& fragPar)
{
    *passportTextStream.get()<<QString("Параметры расчета: \n\n");
    switch (type)
    {
    case SPHERE:
    {
        *passportTextStream.get()<<QString("Тип тела: Сфера \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.sphereFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по тета: "+QString::number(fragPar.sphereTetaFragNum)+"\n";
        *passportTextStream.get()<<"Радиус сферы: "+QString::number(fragPar.sphereRad)+"\n";
        break;
    }
    case CYLINDER:
    {
        *passportTextStream.get()<<QString("Тип тела: Цилиндр \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.cylinderFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по радиусу: "+QString::number(fragPar.cylinderRadFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по высоте: "+QString::number(fragPar.cylinderHeightFragNum)+"\n";
        *passportTextStream.get()<<"Диаметр цилиндра: "+QString::number(fragPar.cylinderDiameter)+"\n";
        *passportTextStream.get()<<"Высота цилиндра: "+QString::number(fragPar.cylinderHeight)+"\n";
        break;
    }
    case ROTATIONBODY:
    {
        *passportTextStream.get()<<QString("Тип тела: Тело вращения \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.rotationBodyFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по частям: "+QString::number(fragPar.rotationBodyPartFragNum)+"\n";
        *passportTextStream.get()<<"Начальная точка по х: "+QString::number(fragPar.rotationBodyXBeg)+"\n";
        *passportTextStream.get()<<"Конечная точка по х: "+QString::number(fragPar.rotationBodyXEnd)+"\n";
        *passportTextStream.get()<<"Высота среза: "+QString::number(fragPar.rotationBodySectionDistance)+"\n";

        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        *passportTextStream.get()<<QString("Тип тела: Тело вращения со срезом \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.rotationBodyFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по частям: "+QString::number(fragPar.rotationBodyPartFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по радиусу на срезе: "+QString::number(fragPar.rotationBodyRFragNum)+"\n";
        *passportTextStream.get()<<"Начальная точка по х: "+QString::number(fragPar.rotationBodyXBeg)+"\n";
        *passportTextStream.get()<<"Конечная точка по х: "+QString::number(fragPar.rotationBodyXEnd)+"\n";
        *passportTextStream.get()<<"Высота среза: "+QString::number(fragPar.rotationBodySectionDistance)+"\n";
        break;
    }
    default:
    {
        *passportTextStream.get()<<QString("Тип тела: Финальные параметры после варьирования для сферы \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.sphereFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по тета: "+QString::number(fragPar.sphereTetaFragNum)+"\n";
        *passportTextStream.get()<<"Радиус сферы: "+QString::number(fragPar.sphereRad)+"\n";
        break;
    }
    }

    *passportTextStream.get()<<"Подъем вортонов при разбиении: "+QString::number(fragPar.delta)+"\n";
    *passportTextStream.get()<<"Подъем контрольных точек для вычисления давления: "+QString::number(fragPar.pointsRaising)+"\n";
    *passportTextStream.get()<<"Радиус вортонов: "+QString::number(fragPar.vortonsRad)+"\n";
    *passportTextStream.get()<<"Давление потока: "+QString::number(solvPar.streamPres)+"\n";
    *passportTextStream.get()<<"Плотность: "+QString::number(solvPar.density)+"\n";
    *passportTextStream.get()<<"Cкорость потока: "+QString::number(solvPar.streamVel.x())+" "+QString::number(solvPar.streamVel.y())+" "+QString::number(solvPar.streamVel.z())+"\n";
    *passportTextStream.get()<<"Расстояние между центрами для объединения: "+QString::number(solvPar.eStar)+"\n";
    *passportTextStream.get()<<"Минимальный косинус для объединения: "+QString::number(solvPar.eDoubleStar)+"\n";
    *passportTextStream.get()<<"Максимальный угол для поворота: "+QString::number(solvPar.fiMax)+"\n";
    *passportTextStream.get()<<"Максимальное удлинение вортона: "+QString::number(solvPar.eDelta)+"\n";
    *passportTextStream.get()<<"Минимальная завихренность: "+QString::number(solvPar.minVorticity)+"\n";
    *passportTextStream.get()<<"Высота слоя: "+QString::number(solvPar.layerHeight)+"\n";
    *passportTextStream.get()<<"Величина шага: "+QString::number(solvPar.tau)+"\n";
    *passportTextStream.get()<<"Количество шагов: "+QString::number(solvPar.stepsNum)+"\n";
    *passportTextStream.get()<<"Подъем вортонов с рамок: "+QString::number(solvPar.deltaUp)+"\n";
    *passportTextStream.get()<<"Максимальное расстояние для воротонов в следе: "+QString::number(solvPar.farDistance)+"\n";
    *passportTextStream.get()<<"Максимальное перемещение: "+QString::number(solvPar.maxMove)+"\n";
    passportTextStream.get()->flush();

}

void Logger::writePassport(const SolverParameters &solvPar, const FragmentationParameters &fragPar, const FreeMotionParameters &freeMotionPar)
{
    writePassport(solvPar,fragPar);
    *passportTextStream.get()<<"Cкорость тела: "+QString::number(freeMotionPar.bodyVel.x())+" "+QString::number(freeMotionPar.bodyVel.y())+" "+QString::number(freeMotionPar.bodyVel.z())+"\n";
    passportTextStream.get()->flush();
}

void Logger::writeForces(const Vector3D forces,const Vector3D c)
{
    *forcesTextStream.get()<<QString::number(forces.x())+"\t";
    *forcesTextStream.get()<<QString::number(forces.y())+"\t";
    *forcesTextStream.get()<<QString::number(forces.z())+"\t";
    *forcesTextStream.get()<<QString::number(c.x())+"\t";
    *forcesTextStream.get()<<QString::number(c.y())+"\t";
    *forcesTextStream.get()<<QString::number(c.z())+"\n";
}

void Logger::writeSolverTime(const double solvTime)
{
    *logTextStream.get()<<"Расчет занял "+QString::number(solvTime)+" с.\n";
    logTextStream.get()->flush();
}

void Logger::closeFiles()
{
    logFile->close();
    forcesFile->close();
    passportFile->close();
}
