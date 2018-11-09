#include "logger.h"

/*!
Создает каталог для хранения результатов расчета внутри каталога сборки
\param _type вид рассчитываемого тела
\param _stype тип расчета
*/

Logger::Logger()
{

}

Logger::Logger(const BodyType _type, const SolvType _stype)
{
    type=_type;
    QDir folder(QCoreApplication::applicationDirPath());
    folder.cdUp();
    path=folder.absolutePath();
    if (_stype==NOOPTIMIZATION)
    {

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
        }
    }
    else
    {
        path.append("/Results_");
        path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
        folder.mkdir("Results_"+QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
    }
    createFiles();
}

/*!
Создает каталог для хранения результатов расчета по заданному пути
\param _type вид рассчитываемого тела
\param _path путь для записи каталога
\param _stype тип расчета
*/

Logger::Logger(BodyType _type, QString _path, SolvType _stype)
{
    type=_type;
    path=_path;
    QDir folder(path);
    if (_stype==NOOPTIMIZATION)
    {

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
        }
    }
    else
    {
        path.append("/Results_");
        path+=QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");
        folder.mkdir("Results_"+QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
    }
    createFiles();
}

void Logger::writeVortons(QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freevortons, const int stepNum)
{
    QString currentStepStr=QString::number(stepNum);
    while(currentStepStr.size()!=5)
        currentStepStr.push_front(QString::number(0));
    QString vortonsPath=path+"/VortonFiles/Kadr"+currentStepStr+".txt";
    QFile vortonFile(vortonsPath);
    if (vortonFile.open(QIODevice::WriteOnly))
    {
        QTextStream vortonsTextStream(&vortonFile);
        QString vortonString;
        vortonString=QString::number(freevortons.size())+"\t";
        int framesSize=0;
        for (int i=0; i<frames.size(); i++)
            framesSize+=frames[i]->getAnglesNum();
        vortonString+=QString::number(framesSize)+"\n\n";
        vortonsTextStream<<vortonString;
        vortonsTextStream.flush();
        for (int i=0; i<freevortons.size();i++)
        {
            vortonString.clear();
            vortonString=QString::number(i)+"\t";
            vortonString+=QString::number(freevortons[i].getVorticity())+"\t";
            vortonString+=QString::number(freevortons[i].getMid().x())+"\t";
            vortonString+=QString::number(freevortons[i].getMid().y())+"\t";
            vortonString+=QString::number(freevortons[i].getMid().z())+"\t";
            vortonString+=QString::number(freevortons[i].getTail().x()-freevortons[i].getMid().x())+"\t";
            vortonString+=QString::number(freevortons[i].getTail().y()-freevortons[i].getMid().y())+"\t";
            vortonString+=QString::number(freevortons[i].getTail().z()-freevortons[i].getMid().z())+"\t";
            vortonsTextStream<<vortonString+"\n\n";
            vortonsTextStream.flush();
        }
        int currentVorton=freevortons.size();
        for (int i=0; i<frames.size();i++)
        {
            for (int j=0; j<frames[i]->getAnglesNum();j++)
            {
                vortonString.clear();
                vortonString=QString::number(currentVorton)+"\t";
                vortonString+=QString::number(frames[i]->at(j).getVorticity())+"\t";
                vortonString+=QString::number(frames[i]->at(j).getMid().x())+"\t";
                vortonString+=QString::number(frames[i]->at(j).getMid().y())+"\t";
                vortonString+=QString::number(frames[i]->at(j).getMid().z())+"\t";
                vortonString+=QString::number(frames[i]->at(j).getTail().x()-frames[i]->at(j).getMid().x())+"\t";
                vortonString+=QString::number(frames[i]->at(j).getTail().y()-frames[i]->at(j).getMid().y())+"\t";
                vortonString+=QString::number(frames[i]->at(j).getTail().z()-frames[i]->at(j).getMid().z())+"\t";
                vortonsTextStream<<vortonString+"\n\n";
                vortonsTextStream.flush();
                currentVorton++;
            }
        }
    }
    vortonFile.close();
}

/*!
Создает необходимые файлы внутри текущего каталога. Функция вызывается автоматически при создании объекта класса.
*/

void Logger::createFiles()
{
    QDir folder(path);
    folder.mkdir("VortonFiles");
    logFile=std::shared_ptr<QFile>(new QFile(path+"/logs.txt"));
    passportFile=std::shared_ptr<QFile>(new QFile(path+"/passport.txt"));
    forcesFile=std::shared_ptr<QFile>(new QFile(path+"/forces.csv"));
    if (type==SPHERE||type==CYLINDER)
    {
        cpFile=std::shared_ptr<QFile>(new QFile(path+"/cp.csv"));
        if (cpFile->open(QIODevice::WriteOnly))
        {
            cpTextStream=std::shared_ptr<QTextStream>(new QTextStream(cpFile.get()));
            *cpTextStream.get()<<"Файл для сил создан в "+QTime::currentTime().toString("H:m:s a")+"\n\n";
            *cpTextStream.get()<<QString("Cp \t");
            *cpTextStream.get()<<QString("teta \n");
        }
        else
        {
            QMessageBox::critical(new QWidget(),tr("Ошибка"), tr("Не удалось создать файл для записи сил"));
            exit(1);
        }
    }

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

/*!
Записывает данные Ср-распределения в файл
\param cp вектор, содержащий значения Ср
\param tetas вектор, содержащий значения углов(тета)
*/

void Logger::writeCpFile(const QVector<double> cp, const QVector<double> tetas)
{
    for (int i=0; i<cp.size(); i++)
    {
         *cpTextStream.get()<<QString::number(cp[i])+"\t";
         *cpTextStream.get()<<QString::number(tetas[i])+"\n";
    }
    cpTextStream.get()->flush();
}

/*!
Записывает данные логов в файл
\param stepNum текущий шаг расчета
\param stepTime текущее время расчета
\param beforeIntegrC значения счетчиков для вортонов с рамок
\param afterIntegrC значения счетчиков для вортонов в слое
\param beforeIntegrT значения таймеров вортонов для вортонов с рамок
\param afterIntegrT значения таймеров для вортонов в слое
\param restr значения сработанных ограничений
*/

void Logger::writeLogs(const int stepNum, const double stepTime, const int freeVortonsSize, const Counters beforeIntegrC, const Counters afterIntegrC, const Timers beforeIntegrT, const Timers afterIntegrT, const Restrictions restr)
{
    *logTextStream.get()<<"Начата запись шага расчета №"+QString::number(stepNum)+" \n\n";
    *logTextStream.get()<<"С рамок объединилось "+QString::number(beforeIntegrC.unitedNum)+" вортонов \n";
    *logTextStream.get()<<"По гамме c рамок удалилось "+QString::number(beforeIntegrC.vorticityEliminated)+" вортонов \n";
    *logTextStream.get()<<"Из фигуры назад возвращено "+QString::number(afterIntegrC.gotBackNum)+" вортонов \n";
    *logTextStream.get()<<"Из слоя развернуто относительно поверхности "+QString::number(afterIntegrC.rotatedNum)+" вортонов \n";
    *logTextStream.get()<<"В слое объединилось "+QString::number(afterIntegrC.unitedNum)+" вортонов \n";
    *logTextStream.get()<<"В слое удалилось по гамме "+QString::number(afterIntegrC.vorticityEliminated)+" вортонов \n";
    if (afterIntegrC.underScreenNum!=0)
        *logTextStream.get()<<"Под экран попало "+QString::number(afterIntegrC.underScreenNum)+" вортонов \n";
    *logTextStream.get()<<"По причине большой дальности удалено"+QString::number(afterIntegrC.tooFarNum)+" вортонов \n";
    *logTextStream.get()<<"Всего вортонов в потоке:"+QString::number(freeVortonsSize)+"\n\n";

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

/*!
Записывает начальные данные расчета в файл
\param solvPar параметры расчета
\param fragPar параметры разбиения тела
*/
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

/*!
Записывает начальные данные расчета в файл
\param solvPar параметры расчета
\param fragPar параметры разбиения тела
\param freeMotionPar параметры свободного движения
*/
void Logger::writePassport(const SolverParameters &solvPar, const FragmentationParameters &fragPar, const FreeMotionParameters &freeMotionPar)
{
    writePassport(solvPar,fragPar);
    *passportTextStream.get()<<"Cкорость тела: "+QString::number(freeMotionPar.bodyVel.x())+" "+QString::number(freeMotionPar.bodyVel.y())+" "+QString::number(freeMotionPar.bodyVel.z())+"\n";
    passportTextStream.get()->flush();
}

/*!
Записывает значения сил и аэродинамических коэффициентов в файл
\param forces вектор значения сил
\param c вектор значения аэродинамических характеристик
*/
void Logger::writeForces(const Vector3D forces,const Vector3D c)
{
    *forcesTextStream.get()<<QString::number(forces.x())+"\t";
    *forcesTextStream.get()<<QString::number(forces.y())+"\t";
    *forcesTextStream.get()<<QString::number(forces.z())+"\t";
    *forcesTextStream.get()<<QString::number(c.x())+"\t";
    *forcesTextStream.get()<<QString::number(c.y())+"\t";
    *forcesTextStream.get()<<QString::number(c.z())+"\n";
    forcesTextStream.get()->flush();
}

/*!
Записывает время, занятое расчетом, в файл
\param solvTime значение занятого времени
*/
void Logger::writeSolverTime(const double solvTime)
{
    *logTextStream.get()<<"Расчет занял "+QString::number(solvTime)+" с.\n";
    logTextStream.get()->flush();
}

/*!
Закрывает все файлы, открытые для записи
*/
void Logger::closeFiles()
{
    if (logFile->isOpen())
        logFile->close();
    if (forcesFile->isOpen())
        forcesFile->close();
    if (passportFile->isOpen())
        passportFile->close();
    if (type==SPHERE||type==CYLINDER)
    {
        if (cpFile->isOpen())
            cpFile->close();
    }
}

/*!
Возвращает текущий путь до кталога для записи
\return Значение пути
*/
QString Logger::getPath()
{
    return path;
}

void Logger::openVortonFiles(QString vortonsDir)
{
    QDir folder(vortonsDir);
    QStringList fileList=folder.entryList();
    for (int i=2; i<fileList.size(); i++)
    {
        QString vortonsPath=vortonsDir+"/"+fileList[i];
        QFile vortonsFile(vortonsPath);
        if (vortonsFile.open(QIODevice::ReadOnly))
        {
            QString line = vortonsFile.readLine();
            QStringList lineSplitted=line.split("\t", QString::SkipEmptyParts);
            int vortonsSize=lineSplitted[0].toInt();
            int framesSize=lineSplitted[1].toInt();
            QVector<Vorton> freeVortons;
            QVector<Vorton> frames;

            while(lineSplitted[0].toInt()!=vortonsSize-1)
            {
                line=vortonsFile.readLine();
                if (line!="\n")
                {
                lineSplitted=line.split("\t", QString::SkipEmptyParts);
                freeVortons.push_back(Vorton(Vector3D(lineSplitted[2].toDouble(),lineSplitted[3].toDouble(),lineSplitted[4].toDouble()),
                        Vector3D(lineSplitted[2].toDouble()+lineSplitted[5].toDouble(),lineSplitted[3].toDouble()+lineSplitted[6].toDouble(),lineSplitted[4].toDouble()+lineSplitted[7].toDouble()),
                        lineSplitted[1].toDouble(),0.1));
                 }
            }
            while(lineSplitted[0].toInt()!=vortonsSize+framesSize-1)
            {

                line=vortonsFile.readLine();
                if (line!="\n")
                {
                lineSplitted=line.split("\t", QString::SkipEmptyParts);
                frames.push_back(Vorton(Vector3D(lineSplitted[2].toDouble(),lineSplitted[3].toDouble(),lineSplitted[4].toDouble()),
                        Vector3D(lineSplitted[2].toDouble()+lineSplitted[5].toDouble(),lineSplitted[3].toDouble()+lineSplitted[6].toDouble(),lineSplitted[4].toDouble()+lineSplitted[7].toDouble()),
                        lineSplitted[1].toDouble(),0.1));
                }
            }
            emit sendVortons(freeVortons,frames);
            QThread::sleep(1);
        }
        vortonsFile.close();
    }

}

