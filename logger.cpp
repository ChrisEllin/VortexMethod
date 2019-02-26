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
    QDir dataDir(path);
    dataDir.mkdir("traces");
    dataDir.mkdir("mesh");
    dataDir.mkdir("streamlines");
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

    QDir dataDir(path);
    dataDir.mkdir("traces");
    dataDir.mkdir("mesh");
    dataDir.mkdir("streamlines");
    createFiles();
}

std::shared_ptr<QPair<QVector<QVector<Vorton>>,QVector<QVector<Vorton>>>> Logger::loadKadrDir(const QString vortonsDir)
{
    QVector<QVector<Vorton>> allVortons;
    QVector<QVector<Vorton>> allFrames;
    QPair<QVector<QVector<Vorton>>,QVector<QVector<Vorton>>> vortonsAndFrames;
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
            allVortons.push_back(freeVortons);
            allFrames.push_back(frames);
        }
        vortonsFile.close();
    }
    vortonsAndFrames.first=allVortons;
    vortonsAndFrames.second=allFrames;
    std::shared_ptr<QPair<QVector<QVector<Vorton>>,QVector<QVector<Vorton>>>> resultedPointer=std::make_shared<QPair<QVector<QVector<Vorton>>,QVector<QVector<Vorton>>>>(vortonsAndFrames);
    return resultedPointer;
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
    tableFile=std::shared_ptr<QFile>(new QFile(path+"/tableLog.csv"));
    if (type==SPHERE||type==CYLINDER||type==ROTATIONBODY)
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
    if (tableFile->open(QIODevice::WriteOnly))
    {
        tableTextStream=std::shared_ptr<QTextStream>(new QTextStream(tableFile.get()));
        *tableTextStream.get()<<QString("Step Number \t");
        *tableTextStream.get()<<QString("Current Time \t");
        *tableTextStream.get()<<QString("Generated Number \t");
        *tableTextStream.get()<<QString("United Number \t");
        *tableTextStream.get()<<QString("Deleted gamma Number \t");
        *tableTextStream.get()<<QString("Deleted far Number \t");
        *tableTextStream.get()<<QString("Remained Number \t");
        *tableTextStream.get()<<QString("Reguliser \t");
        *tableTextStream.get()<<QString("Max Gamma \t");
        *tableTextStream.get()<<QString("Velocity in center \t");
        *tableTextStream.get()<<QString("Conditionality number \n");
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

void Logger::writeCpDegreeFile(const QVector<double> cp, const QVector<double> tetas, const int degree)
{
    QFile cpDegreeFile(path+"/cp"+QString::number(degree)+".csv");
    if (cpDegreeFile.open(QIODevice::WriteOnly))
    {
        QTextStream cpDegreeTextStream(&cpDegreeFile);
        cpDegreeTextStream<<"cp, forming\n";
        for (int i=0; i<tetas.size(); i++)
        {
            i<cp.size() ? cpDegreeTextStream<<QString::number(cp[i])+",\t" : cpDegreeTextStream<<" ,\t";
            cpDegreeTextStream<<QString::number(tetas[i])+",\n";
        }
       cpDegreeTextStream.flush();
    }
    cpDegreeFile.close();
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
        //*passportTextStream.get()<<"Конечная точка по х: "+QString::number(fragPar.rotationBodyXEnd)+"\n";
        *passportTextStream.get()<<"Высота среза в начале: "+QString::number(fragPar.rotationBodySectionDistance)+"\n";
        *passportTextStream.get()<<"Высота среза в конце: "+QString::number(fragPar.rotationBodySectionEndDistance)+"\n";
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        *passportTextStream.get()<<QString("Тип тела: Тело вращения со срезом \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.rotationBodyFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по частям: "+QString::number(fragPar.rotationBodyPartFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по радиусу на срезе: "+QString::number(fragPar.rotationBodyRFragNum)+"\n";
        *passportTextStream.get()<<"Начальная точка по х: "+QString::number(fragPar.rotationBodyXBeg)+"\n";
        //*passportTextStream.get()<<"Конечная точка по х: "+QString::number(fragPar.rotationBodyXEnd)+"\n";
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

void Logger::writePassport(const SolverParameters &solvPar, const FragmentationParameters &fragPar, const FormingParameters forming, const FramesSizes framesSizes)
{
    writePassport(solvPar,fragPar);
    switch (type)
    {
    case ROTATIONBODY:
    {
        switch (forming.typeNum)
        {
        case 0:
        {
        *passportTextStream.get()<<QString("Полусфера-цилиндр-конус\n");
        *passportTextStream.get()<<"Диаметр образующей: "+QString::number(forming.diameter)+"\n";
        *passportTextStream.get()<<"Длина первой секции: "+QString::number(forming.sectorOneLength)+"\n";
        *passportTextStream.get()<<"Длина второй секции: "+QString::number(forming.sectorTwoLength)+"\n";
        break;
        }
        case 1:
        {
        *passportTextStream.get()<<QString("Конус-цилиндр-конус\n");
        *passportTextStream.get()<<"Диаметр образующей: "+QString::number(forming.diameter)+"\n";
        *passportTextStream.get()<<"Длина образующей: "+QString::number(forming.sectorOneLength)+"\n";
        *passportTextStream.get()<<"Угол конуса: "+QString::number(forming.angle)+"\n";
        break;
        }
        case 2:
        {
        *passportTextStream.get()<<QString("Эллипсоид вращения\n");
        *passportTextStream.get()<<"Диаметр образующей: "+QString::number(forming.diameter)+"\n";
        *passportTextStream.get()<<"Длина образующей: "+QString::number(forming.sectorOneLength)+"\n";
        break;
        }
        }
        break;
    }
    case ROTATIONBOTTOMCUT:
    {
        *passportTextStream.get()<<"Диаметр образующей: "+QString::number(forming.diameter)+"\n";
        *passportTextStream.get()<<"Диаметр ''хвоста'': "+QString::number(forming.tailDiameter)+"\n";
        *passportTextStream.get()<<"Длина первой секции: "+QString::number(forming.sectorOneLength)+"\n";
        *passportTextStream.get()<<"Длина второй секции: "+QString::number(forming.sectorTwoLength)+"\n";
        break;
    }
    }
    *passportTextStream.get()<<QString("Параметры сетки \n");
    *passportTextStream.get()<<"Минимальная длина "+QString::number(framesSizes.minFrameSize)+"\n";
    *passportTextStream.get()<<"Максимальная длина "+QString::number(framesSizes.maxFrameSize)+"\n";
    *passportTextStream.get()<<"Средняя длина "+QString::number(framesSizes.averFrameSize)+"\n";
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

void Logger::writeTable(const int stepNum, const double stepTime,const double generatedNum, const double maxGamma, const Vector3D velocity, const double reguliser, const int freeVortonsSize, const Counters beforeIntegrC, const Counters afterIntegrC, const double conditionalNum)
{
    *tableTextStream.get()<<QString::number(stepNum)+"\t";
    *tableTextStream.get()<<QString::number(stepTime)+"\t";
    *tableTextStream.get()<<QString::number(generatedNum)+"\t";
    *tableTextStream.get()<<QString::number(beforeIntegrC.unitedNum)+"\t";
    *tableTextStream.get()<<QString::number(beforeIntegrC.vorticityEliminated)+"\t";
    *tableTextStream.get()<<QString::number(afterIntegrC.tooFarNum)+"\t";
    *tableTextStream.get()<<QString::number(freeVortonsSize)+"\t";
    *tableTextStream.get()<<QString::number(reguliser)+"\t";
    *tableTextStream.get()<<QString::number(maxGamma)+"\t";
    *tableTextStream.get()<<QString::number(velocity.length())+"\t";
    *tableTextStream.get()<<QString::number(conditionalNum)+"\n";
    tableTextStream.get()->flush();
}

void Logger::createParaviewFile(QVector<std::shared_ptr<MultiFrame>> &frames, QVector<double>& forces, QVector<Vector3D>& velocities,QVector<double> &tangentialVelocities,QVector<double> &normalVelocities,QVector<std::shared_ptr<MultiFrame>> &sectionFrames, int currentStep)
{
    QFile paraviewFile(path+"/mesh/visual.vtk."+QString::number(currentStep));
    if (paraviewFile.open(QIODevice::WriteOnly))
    {
        QTextStream paraviewTs(&paraviewFile);
        paraviewTs<<"# vtk DataFile Version 3.0\n";
        paraviewTs<<"vtk output\n";
        paraviewTs<<"ASCII\n";
        paraviewTs<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<frames.size();i++)
            points+=frames[i]->getAnglesNum();
        for (int i=0; i<sectionFrames.size();i++)
            points+=sectionFrames[i]->getAnglesNum();
        paraviewTs<<"POINTS "<<points<<" float\n";
        paraviewTs.flush();
        for (int i=0; i<frames.size();i++)
        {
            for (int j=0; j<frames[i]->getAnglesNum(); j++)
                paraviewTs<<frames[i]->at(j).getTail().x()<<" "<<frames[i]->at(j).getTail().y()<<" "<<frames[i]->at(j).getTail().z()<<"\n";
            paraviewTs.flush();
        }
        for (int i=0; i<sectionFrames.size(); i++)
        {
            for (int j=0; j<sectionFrames[i]->getAnglesNum(); j++)
                paraviewTs<<sectionFrames[i]->at(j).getTail().x()<<" "<<sectionFrames[i]->at(j).getTail().y()<<" "<<sectionFrames[i]->at(j).getTail().z()<<"\n";
            paraviewTs.flush();
        }
        paraviewTs<<"CELLS "<<frames.size()+sectionFrames.size()<<" "<<points+frames.size()+sectionFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<frames.size(); i++)
        {
            paraviewTs<<frames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+frames[i]->getAnglesNum();j++)
            {
                paraviewTs<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            paraviewTs<<"\n";
        }
        for (int i=0; i<sectionFrames.size(); i++)
        {
            paraviewTs<<sectionFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+sectionFrames[i]->getAnglesNum();j++)
            {
                paraviewTs<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            paraviewTs<<"\n";
        }
        paraviewTs.flush();
        paraviewTs<<"CELL_TYPES "<<frames.size()+sectionFrames.size()<<"\n";
        for (int i=0;i<frames.size(); i++)
            frames[i]->getAnglesNum()==4 ? paraviewTs<<"9\n" : paraviewTs<<"7\n";
        paraviewTs.flush();
        for (int i=0;i<sectionFrames.size(); i++)
            sectionFrames[i]->getAnglesNum()==4 ? paraviewTs<<"9\n" : paraviewTs<<"7\n";
        paraviewTs.flush();
        paraviewTs<<"CELL_DATA "<<frames.size()+sectionFrames.size()<<"\n";
        paraviewTs<<"SCALARS pressure float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<forces[i] : paraviewTs<<" "<<forces[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"SCALARS tangential_speed float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<tangentialVelocities[i] : paraviewTs<<" "<<tangentialVelocities[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"SCALARS normal_speed float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocities[i] : paraviewTs<<" "<<normalVelocities[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            paraviewTs<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        paraviewTs.flush();
    }
    paraviewFile.close();
}

void Logger::createParaviewFile(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocities, int currentStep)
{
    QFile paraviewFile(path+"/mesh/visual.vtk."+QString::number(currentStep));
    if (paraviewFile.open(QIODevice::WriteOnly))
    {
        QTextStream paraviewTs(&paraviewFile);
        paraviewTs<<"# vtk DataFile Version 3.0\n";
        paraviewTs<<"vtk output\n";
        paraviewTs<<"ASCII\n";
        paraviewTs<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<frames.size();i++)
            points+=frames[i]->getAnglesNum();
        paraviewTs<<"POINTS "<<points<<" float\n";
        paraviewTs.flush();
        for (int i=0; i<frames.size();i++)
        {
            for (int j=0; j<frames[i]->getAnglesNum(); j++)
                paraviewTs<<frames[i]->at(j).getTail().x()<<" "<<frames[i]->at(j).getTail().y()<<" "<<frames[i]->at(j).getTail().z()<<"\n";
            paraviewTs.flush();
        }

        paraviewTs<<"CELLS "<<frames.size()<<" "<<points+frames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<frames.size(); i++)
        {
            paraviewTs<<frames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+frames[i]->getAnglesNum();j++)
            {
                paraviewTs<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            paraviewTs<<"\n";
        }

        paraviewTs.flush();
        paraviewTs<<"CELL_TYPES "<<frames.size()<<"\n";
        for (int i=0;i<frames.size(); i++)
            frames[i]->getAnglesNum()==4 ? paraviewTs<<"9\n" : paraviewTs<<"7\n";
        paraviewTs.flush();


        paraviewTs<<"CELL_DATA "<<frames.size()<<"\n";
        paraviewTs<<"SCALARS pressure float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<forces[i] : paraviewTs<<" "<<forces[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"SCALARS tangential_speed float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<tangentialVelocities[i] : paraviewTs<<" "<<tangentialVelocities[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"SCALARS normal_speed float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocities[i] : paraviewTs<<" "<<normalVelocities[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            paraviewTs<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        paraviewTs.flush();
    }
    paraviewFile.close();
}

void Logger::createParaviewFile(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocitiesBefore, QVector<double> &normalVelocitiesAfter, QVector<double> &normalVelocitiesEnd,QVector<std::shared_ptr<MultiFrame> > &sectionFrames, int currentStep)
{
    QFile paraviewFile(path+"/mesh/visual.vtk."+QString::number(currentStep));
    if (paraviewFile.open(QIODevice::WriteOnly))
    {
        QTextStream paraviewTs(&paraviewFile);
        paraviewTs<<"# vtk DataFile Version 3.0\n";
        paraviewTs<<"vtk output\n";
        paraviewTs<<"ASCII\n";
        paraviewTs<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<frames.size();i++)
            points+=frames[i]->getAnglesNum();
        for (int i=0; i<sectionFrames.size();i++)
            points+=sectionFrames[i]->getAnglesNum();
        paraviewTs<<"POINTS "<<points<<" float\n";
        paraviewTs.flush();
        for (int i=0; i<frames.size();i++)
        {
            for (int j=0; j<frames[i]->getAnglesNum(); j++)
                paraviewTs<<frames[i]->at(j).getTail().x()<<" "<<frames[i]->at(j).getTail().y()<<" "<<frames[i]->at(j).getTail().z()<<"\n";
            paraviewTs.flush();
        }
        for (int i=0; i<sectionFrames.size(); i++)
        {
            for (int j=0; j<sectionFrames[i]->getAnglesNum(); j++)
                paraviewTs<<sectionFrames[i]->at(j).getTail().x()<<" "<<sectionFrames[i]->at(j).getTail().y()<<" "<<sectionFrames[i]->at(j).getTail().z()<<"\n";
            paraviewTs.flush();
        }
        paraviewTs<<"CELLS "<<frames.size()+sectionFrames.size()<<" "<<points+frames.size()+sectionFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<frames.size(); i++)
        {
            paraviewTs<<frames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+frames[i]->getAnglesNum();j++)
            {
                paraviewTs<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            paraviewTs<<"\n";
        }
        for (int i=0; i<sectionFrames.size(); i++)
        {
            paraviewTs<<sectionFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+sectionFrames[i]->getAnglesNum();j++)
            {
                paraviewTs<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            paraviewTs<<"\n";
        }
        paraviewTs.flush();
        paraviewTs<<"CELL_TYPES "<<frames.size()+sectionFrames.size()<<"\n";
        for (int i=0;i<frames.size(); i++)
            frames[i]->getAnglesNum()==4 ? paraviewTs<<"9\n" : paraviewTs<<"7\n";
        paraviewTs.flush();
        for (int i=0;i<sectionFrames.size(); i++)
            sectionFrames[i]->getAnglesNum()==4 ? paraviewTs<<"9\n" : paraviewTs<<"7\n";
        paraviewTs.flush();
        paraviewTs<<"CELL_DATA "<<frames.size()+sectionFrames.size()<<"\n";
        paraviewTs<<"SCALARS pressure float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<forces[i] : paraviewTs<<" "<<forces[i];
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"SCALARS tangential_speed float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<tangentialVelocities[i] : paraviewTs<<" "<<tangentialVelocities[i];
        paraviewTs<<"\n";
        paraviewTs.flush();

        paraviewTs<<"SCALARS normal_speedB float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesBefore[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesBefore[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();
        paraviewTs<<"SCALARS normal_speedA float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesAfter[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesAfter[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();


        paraviewTs<<"SCALARS normal_speedC float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesEnd[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesEnd[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();

        paraviewTs<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            paraviewTs<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        paraviewTs.flush();
    }
    paraviewFile.close();
}

void Logger::createParaviewStreamlinesFile(QVector<Vector3D> velocities,QPair<int,int> boundary,double step, int currentStep)
{
    QFile streamLinesFile(path+"/streamlines/streamlines.vtk."+QString::number(currentStep));
    if (streamLinesFile.open(QIODevice::WriteOnly))
    {
        qDebug()<<boundary.first;
        qDebug()<<boundary.second;
        QTextStream streamLinesTextStream(&streamLinesFile);
        streamLinesTextStream<<"# vtk DataFile Version 3.0\n";
        streamLinesTextStream<<"vtk output\n";
        streamLinesTextStream<<"ASCII\n";
        streamLinesTextStream<<"DATASET STRUCTURED_POINTS\n";
        int xDimension=static_cast<int>(ceil(static_cast<double>(boundary.first+2)/step))+1;
        int yDimension=static_cast<int>(ceil(static_cast<double>(boundary.second*2+2)/step))+1;
        streamLinesTextStream<<"DIMENSIONS "+QString::number(xDimension)+" "+QString::number(yDimension)+" "+QString::number(yDimension)+"\n";
        int pointsQuant=xDimension*yDimension*yDimension;
        streamLinesTextStream<<"ASPECT_RATIO "+QString::number(step)+" "+QString::number(step)+" "+QString::number(step)+"\n";
        streamLinesTextStream<<"ORIGIN -1 "+QString::number(-1*static_cast<int>(ceil(boundary.second+1)))+" "+QString::number(-1*static_cast<int>(ceil(boundary.second+1)))+"\n";
        streamLinesTextStream<<"POINT_DATA "+QString::number(pointsQuant)+" \n";
        streamLinesTextStream<<"VECTORS velocities float\n";
        if (pointsQuant!=velocities.size())
            qDebug()<<"Error!";
        for (int i=0; i<velocities.size();i++)
            streamLinesTextStream<<QString::number(velocities[i].x())+" "+QString::number(velocities[i].y())+" "+QString::number(velocities[i].z())+"\n";
        streamLinesTextStream.flush();
    }
    streamLinesFile.close();
}

void Logger::createParaviewTraceVerticesFile(QVector<Vorton> &vortons, int currentStep)
{
    QFile traceFile(path+"/traces/trace.vtk."+QString::number(currentStep));
    if (traceFile.open(QIODevice::WriteOnly))
    {
        QTextStream traceStream(&traceFile);
        traceStream<<"# vtk DataFile Version 3.0\n";
        traceStream<<"vtk output\n";
        traceStream<<"ASCII\n";
        traceStream<<"DATASET POLYDATA\n";
        traceStream<<"POINTS "+QString::number(vortons.size()*2)+" float\n";
        for (int i=0; i<vortons.size();i++)
        {
            Vector3D mid=vortons[i].getMid();
            Vector3D tail=vortons[i].getTail();
            traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
            traceStream<<QString::number(tail.x())+" "+QString::number(tail.y())+" "+QString::number(tail.z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(vortons.size())+" "+QString::number(vortons.size()*3)+"\n";
        for (int i=0; i<vortons.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
    }
    traceFile.close();
}

void Logger::createParaviewTraceFile(QVector<Vorton> &vortons, int currentStep)
{
    QFile traceFile(path+"/traces/trace.csv."+QString::number(currentStep));
    if (traceFile.open(QIODevice::WriteOnly))
    {
        QTextStream traceStream(&traceFile);
        traceStream<<"x coord, y coord, z coord\n";
        for (int i=0; i<vortons.size();i++)
            traceStream<<(2.0*vortons[i].getMid()-vortons[i].getTail()).x()<<", "<<(2.0*vortons[i].getMid()-vortons[i].getTail()).y()<<", "<<(2.0*vortons[i].getMid()-vortons[i].getTail()).z()<<"\n";
        traceStream.flush();
    }
    traceFile.close();

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
    if (type==SPHERE||type==CYLINDER||type==ROTATIONBODY)
    {
        if (cpFile->isOpen())
            cpFile->close();
    }
    if (tableFile->isOpen())
        tableFile->close();
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

