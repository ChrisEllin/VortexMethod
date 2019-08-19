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
    dataDir.mkdir("graphs");
    dataDir.mkdir("field");
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
    dataDir.mkdir("graphs");
    dataDir.mkdir("field");
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
    gammaFile=std::shared_ptr<QFile>(new QFile(path+"/gammaFile.txt"));
    passportFile=std::shared_ptr<QFile>(new QFile(path+"/passport.txt"));
    forcesFile=std::shared_ptr<QFile>(new QFile(path+"/forces.csv"));
    tableFile=std::shared_ptr<QFile>(new QFile(path+"/tableLog.csv"));
   cpFile=std::shared_ptr<QFile>(new QFile(path+"/cp.csv"));
        if (cpFile->open(QIODevice::WriteOnly))
        {
            cpTextStream=std::shared_ptr<QTextStream>(new QTextStream(cpFile.get()));
            *cpTextStream.get()<<"Файл для сил создан в "+QTime::currentTime().toString("H:m:s a")+"\n\n";
            *cpTextStream.get()<<QString("Cp \t");
            *cpTextStream.get()<<QString("teta \n");
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
    if (gammaFile->open(QIODevice::WriteOnly))
    {
        gammaTextStream=std::shared_ptr<QTextStream>(new QTextStream(gammaFile.get()));
        *gammaTextStream.get()<<"InitialMaxGamma\t Gamma after SLAU\t Gamma after union frames\t Gamma after union vortons\n";
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
        *tableTextStream.get()<<QString("Conditionality number \t");
        *tableTextStream.get()<<QString("Gamma Restrictions Vortons \n");
    }

}

void Logger::parseSalomeMesh(const QString &path, const double raise, double vortonsRad, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals, QVector<Vector3D> &controlPointsRaised, QVector<double> &squares)
{
    QFile salomeMeshFile(path);
    if (salomeMeshFile.open(QIODevice::ReadOnly))
    {
        QTextStream stream(&salomeMeshFile);
        QString currentline=stream.readLine();
        while (currentline!="Vertices") {
            currentline=stream.readLine();
        };
        QVector<Vector3D> vertices;
        QVector<TriangleVectors> triangles;
        double verticesQuant=QString(stream.readLine()).toInt();
        for (int i=0; i<verticesQuant;i++)
        {
            QStringList vertex=QString(stream.readLine()).split(" ");
            vertices.append(Vector3D(vertex[0].toDouble(),vertex[1].toDouble(),vertex[2].toDouble()));
        }
        currentline=stream.readLine();
        while(currentline!="Triangles")
        {
            currentline=stream.readLine();
        }
        double trianglesQuant=QString(stream.readLine()).toInt();
        for (int i=0;i<trianglesQuant;i++)
        {
            QStringList triangle=QString(stream.readLine()).split(" ");
            TriangleVectors triangleVec;
            triangleVec.a=vertices[triangle[0].toInt()];
            triangleVec.b=vertices[triangle[1].toInt()];
            triangleVec.c=vertices[triangle[2].toInt()];
            triangleVec.calcParameters(raise);
            frames.append(std::make_shared<MultiFrame>(3,triangleVec.center,triangleVec.a,triangleVec.b,vortonsRad));
            controlPoints.append(triangleVec.center);
            controlPointsRaised.append(triangleVec.controlPointRaised);
            normals.append(triangleVec.normal);
            squares.append(triangleVec.square);
        }
    }
    salomeMeshFile.close();
}

void Logger::writeCpGraphs(QVector<QVector<double> > &cpArray, QVector<double> &fis)
{
    switch (type)
    {
    case ROTATIONBODY:
    {
    QCustomPlot graph;
    graph.clearGraphs();

    graph.legend->setVisible(true);   //Включаем Легенду графика
    graph.axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);

    graph.addGraph();
    graph.graph(0)->setName("Cp0");
    graph.graph(0)->setPen(QColor(255,0,0));
    graph.addGraph();
    graph.graph(1)->setName("Cp90");
    graph.graph(1)->setPen(QColor(0,255,0));
    graph.addGraph();
    graph.graph(2)->setName("Cp180");
    graph.graph(2)->setPen(QColor(0,0,255));
    graph.addGraph();
    graph.graph(3)->setName("Cp270");
    graph.graph(3)->setPen(QColor(0,125,125));

    //Подписываем оси Ox и Oy
    graph.xAxis->setLabel("Position");
    graph.yAxis->setLabel("Cp");


    double minCp=cpArray[0][0];
    double maxCp=cpArray[0][0];
    for (int i=0; i<cpArray.size();i++)
    {
        for (int j=0; j<cpArray[i].size();j++)
        {
            if (cpArray[i][j]>maxCp)
                maxCp=cpArray[i][j];
            if (cpArray[i][j]<minCp)
                minCp=cpArray[i][j];
        }
    }
    graph.xAxis->setRange(*std::min_element(fis.begin(),fis.end()), *std::max_element(fis.begin(),fis.end()));
    graph.yAxis->setRange(minCp, maxCp);



   graph.graph(0)->setData(fis, cpArray[0]);
   graph.graph(1)->setData(fis, cpArray[1]);
   graph.graph(2)->setData(fis, cpArray[2]);
   graph.graph(3)->setData(fis, cpArray[3]);
   graph.replot();

   QString fileName(path+"/СpGraph.png");
   QFile file(fileName);

   if (file.open(QIODevice::WriteOnly))
       graph.savePng(fileName);
    file.close();

    break;
    }
    case ROTATIONBOTTOMCUT:
    {
    QCustomPlot graph;
    graph.clearGraphs();

    graph.legend->setVisible(true);   //Включаем Легенду графика
    graph.axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);

    graph.addGraph();
    graph.graph(0)->setName("Cp0");
    graph.graph(0)->setPen(QColor(255,0,0));
    graph.addGraph();
    graph.graph(1)->setName("Cp90");
    graph.graph(1)->setPen(QColor(0,255,0));
    graph.addGraph();
    graph.graph(2)->setName("Cp180");
    graph.graph(2)->setPen(QColor(0,0,255));
    graph.addGraph();
    graph.graph(3)->setName("Cp270");
    graph.graph(3)->setPen(QColor(0,125,125));

    //Подписываем оси Ox и Oy
    graph.xAxis->setLabel("Position");
    graph.yAxis->setLabel("Cp");


    double minCp=cpArray[0][0];
    double maxCp=cpArray[0][0];
    for (int i=0; i<cpArray.size();i++)
    {
        for (int j=0; j<cpArray[i].size();j++)
        {
            if (cpArray[i][j]>maxCp)
                maxCp=cpArray[i][j];
            if (cpArray[i][j]<minCp)
                minCp=cpArray[i][j];
        }
    }
    graph.xAxis->setRange(*std::min_element(fis.begin(),fis.end()), *std::max_element(fis.begin(),fis.end()));
    graph.yAxis->setRange(minCp, maxCp);



   graph.graph(0)->setData(fis, cpArray[0]);
   graph.graph(1)->setData(fis, cpArray[1]);
   graph.graph(2)->setData(fis, cpArray[2]);
   graph.graph(2)->setData(fis, cpArray[3]);
   graph.replot();

   QString fileName(path+"/СpGraph.png");
   QFile file(fileName);

   if (file.open(QIODevice::WriteOnly))
       graph.savePng(fileName);
    file.close();

    break;
    }
    }
}

void Logger::writeNormals(QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals)
{
    QFile traceFile(path+"/normalsVis.vtk");
    if (traceFile.open(QIODevice::WriteOnly))
    {
        QTextStream traceStream(&traceFile);
        traceStream<<"# vtk DataFile Version 3.0\n";
        traceStream<<"vtk output\n";
        traceStream<<"ASCII\n";
        traceStream<<"DATASET POLYDATA\n";
        traceStream<<"POINTS "+QString::number(controlPoints.size()*2)+" float\n";
        for (int i=0; i<controlPoints.size();i++)
        {

            traceStream<<QString::number(controlPoints[i].x())+" "+QString::number(controlPoints[i].y())+" "+QString::number(controlPoints[i].z())+"\n";
            traceStream<<QString::number((controlPoints[i]+normals[i]).x())+" "+QString::number((controlPoints[i]+normals[i]).y())+" "+QString::number((controlPoints[i]+normals[i]).z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(controlPoints.size())+" "+QString::number(controlPoints.size()*3)+"\n";
        for (int i=0; i<controlPoints.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
    }
    traceFile.close();
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
    *logTextStream.get()<<"Из фигуры назад возвращено "+QString::number(beforeIntegrC.gotBackNum)+" вортонов \n";
    *logTextStream.get()<<"Не удалось возвратить "+QString::number(beforeIntegrC.gotBackDeletedNum)+" вортонов \n";
    *logTextStream.get()<<"Из следа развернуто относительно поверхности "+QString::number(beforeIntegrC.rotatedNum)+" вортонов \n";
    *logTextStream.get()<<"В следа объединилось "+QString::number(afterIntegrC.unitedNum)+" вортонов \n";
    *logTextStream.get()<<"В следа удалилось по гамме "+QString::number(afterIntegrC.vorticityEliminated)+" вортонов \n";
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
    *logTextStream.get()<<"Возвращение в поток и разворот занял "+QString::number(beforeIntegrT.getBackAndRotateTimer)+" с.\n";
    *logTextStream.get()<<"Расчет сил занял "+QString::number(beforeIntegrT.forceTimer)+" с.\n";
    *logTextStream.get()<<"Объединение вортонов в следа заняло "+QString::number(afterIntegrT.unionTimer)+" с.\n";
    *logTextStream.get()<<"Удаление по гамме вортонов в следа заняло "+QString::number(afterIntegrT.removeVorticityTimer)+" с.\n";
    //*logTextStream.get()<<"Разворот вортонов в потоке заняло "+QString::number(afterIntegrT.rotateTimer)+" с.\n";
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
    case ROTATIONTWOBOTTOM:
    {
        *passportTextStream.get()<<QString("Тип тела: Тело вращения(двудонное) \n\n");
        *passportTextStream.get()<<"Количество разбиений по фи: "+QString::number(fragPar.rotationBodyFiFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по образующей: "+QString::number(fragPar.rotationBodyPartFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по радиусу в носу: "+QString::number(fragPar.rotationBodyRFragNum)+"\n";
        *passportTextStream.get()<<"Количество разбиений по радиусу в торце: "+QString::number(fragPar.rotationBodyR2FragNum)+"\n";
        *passportTextStream.get()<<"Начальная точка по х: "+QString::number(fragPar.rotationBodyXBeg)+"\n";

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

void Logger::writePassport(const SolverParameters &solvPar, const FragmentationParameters &fragPar, const FormingParameters forming, const FramesSizes framesSizes, double maxGammFactor)
{
    writePassport(solvPar,fragPar);
    *passportTextStream.get()<<"Максимальный множитель гамма: "+QString::number(maxGammFactor)+"\n";
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
    case ROTATIONTWOBOTTOM:
    {
        //*passportTextStream.get()<<"Длина образующей: "+QString::number(forming)+"\n";
        *passportTextStream.get()<<"Диаметр ''хвоста'': "+QString::number(forming.tailDiameter)+"\n";
        *passportTextStream.get()<<"Длина первой секции: "+QString::number(forming.sectorOneLength)+"\n";
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

void Logger::writeTable(const int stepNum, const double stepTime, const double generatedNum, const double maxGamma, const Vector3D velocity, const double reguliser, const int freeVortonsSize, const Counters beforeIntegrC, const Counters afterIntegrC, const double conditionalNum,double quant2)
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
    *tableTextStream.get()<<QString::number(conditionalNum)+"\t";
    *tableTextStream.get()<<QString::number(quant2)+"\n";
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

        paraviewTs<<"SCALARS gamma float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<frames[i]->getVorticity()<<"\n" : paraviewTs<<" "<<frames[i]->getVorticity()<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();

        paraviewTs<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            paraviewTs<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        paraviewTs.flush();
    }
    paraviewFile.close();
}

void Logger::createParaviewFile(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<double> &forces, QVector<Vector3D> &velocities, QVector<double> &tangentialVelocities, QVector<double> &normalVelocitiesAfter, QVector<double> &normalVelocitiesBefore, QVector<double> &normalVelocitiesCenter,QVector<double> &normalVelocitiesEnd, QVector<std::shared_ptr<MultiFrame> > &sectionFrames, int currentStep)
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

        paraviewTs<<"SCALARS normal_speedA(after_SLAU) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesAfter[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesAfter[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();


        paraviewTs<<"SCALARS normal_speedB(after_setting_epsilon) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesBefore[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesBefore[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();



        paraviewTs<<"SCALARS normal_speedC(after_movement) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesCenter[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesCenter[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();


        paraviewTs<<"SCALARS normal_speedD(after_all_operations) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesEnd[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesEnd[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();

        paraviewTs<<"SCALARS gamma float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<frames[i]->getVorticity()<<"\n" : paraviewTs<<" "<<frames[i]->getVorticity()<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();

        paraviewTs<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            paraviewTs<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        paraviewTs.flush();
    }
    paraviewFile.close();
}

void Logger::createParaviewFile(QVector<std::shared_ptr<MultiFrame> > &frames, QVector<double> &forces, QVector<double> &normalVelocitiesAfter, QVector<double> &normalVelocitiesBefore, QVector<double> &normalVelocitiesCenter, QVector<double> &normalVelocitiesEnd, int currentStep)
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


        paraviewTs<<"SCALARS normal_speedA(after_SLAU) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesAfter[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesAfter[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();


        paraviewTs<<"SCALARS normal_speedB(after_setting_epsilon) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesBefore[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesBefore[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();



        paraviewTs<<"SCALARS normal_speedC(after_movement) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesCenter[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesCenter[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();


        paraviewTs<<"SCALARS normal_speedD(after_all_operations) float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<normalVelocitiesEnd[i]<<"\n" : paraviewTs<<" "<<normalVelocitiesEnd[i]<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();

        paraviewTs<<"SCALARS gamma float 1\n";
        paraviewTs<<"LOOKUP_TABLE default\n";
        for (int i=0; i<forces.size(); i++)
            i==0 ? paraviewTs<<frames[i]->getVorticity()<<"\n" : paraviewTs<<" "<<frames[i]->getVorticity()<<"\n";
        paraviewTs<<"\n";
        paraviewTs.flush();


    }
    paraviewFile.close();
}

void Logger::createParaviewStreamlinesFile(QVector<Vector3D> velocities,QPair<int,int> boundary,double step, int currentStep)
{
    QFile streamLinesFile(path+"/streamlines/streamlines.vtk."+QString::number(currentStep));
    if (streamLinesFile.open(QIODevice::WriteOnly))
    {
//        qDebug()<<boundary.first<<"bf";
//        qDebug()<<boundary.second<<"bs";
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
           // Vector3D begin=2.0*mid-tail;
            //traceStream<<QString::number(begin.x())+" "+QString::number(begin.y())+" "+QString::number(begin.z())+"\n";
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
        traceStream<<"POINT_DATA "+QString::number(vortons.size()*2)+"\n";
        traceStream<<"SCALARS gamma double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
        }
        traceStream.flush();

    }
    traceFile.close();
}

void Logger::createParaviewVelocityField(QVector<Vorton> &vortons, double tau, int currentStep)
{

    QFile traceFile(path+"/field/field.vtk."+QString::number(currentStep));
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
            Vector3D move=vortons[i].getMove();

            traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
            traceStream<<QString::number(mid.x()+move.x())+" "+QString::number(mid.y()+move.y())+" "+QString::number(mid.z()+move.z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(vortons.size())+" "+QString::number(vortons.size()*3)+"\n";
        for (int i=0; i<vortons.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
        traceStream<<"POINT_DATA "+QString::number(vortons.size()*2)+"\n";
        traceStream<<"SCALARS gamma double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
        }
        traceStream.flush();
        traceStream<<"SCALARS velocity double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
        }
        traceStream.flush();

    }
    traceFile.close();
}

void Logger::createParaviewVelocityField(QVector<Vorton> &vortons, QVector<Vector3D> &moveFromFrames, QVector<Vector3D> &moveFromVortons, QVector<Vector3D> &moveFromGetBack,double tau, int currentStep)
{
    QFile traceFile(path+"/field/field.vtk."+QString::number(currentStep));
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
            Vector3D move=vortons[i].getMove();

            traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
            traceStream<<QString::number(mid.x()+move.x())+" "+QString::number(mid.y()+move.y())+" "+QString::number(mid.z()+move.z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(vortons.size())+" "+QString::number(vortons.size()*3)+"\n";
        for (int i=0; i<vortons.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
        traceStream<<"POINT_DATA "+QString::number(vortons.size()*2)+"\n";
        traceStream<<"SCALARS gamma double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
        }
        traceStream.flush();
        traceStream<<"SCALARS velocity double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
        }
        traceStream.flush();

    }
    traceFile.close();

    QFile traceFrFile(path+"/field/fieldFrames.vtk."+QString::number(currentStep));
    if (traceFrFile.open(QIODevice::WriteOnly))
    {
        QTextStream traceStream(&traceFrFile);
        traceStream<<"# vtk DataFile Version 3.0\n";
        traceStream<<"vtk output\n";
        traceStream<<"ASCII\n";
        traceStream<<"DATASET POLYDATA\n";
        traceStream<<"POINTS "+QString::number(vortons.size()*2)+" float\n";
        for (int i=0; i<vortons.size();i++)
        {
            Vector3D mid=vortons[i].getMid();
            Vector3D move=moveFromFrames[i];

            traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
            traceStream<<QString::number(mid.x()+move.x())+" "+QString::number(mid.y()+move.y())+" "+QString::number(mid.z()+move.z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(vortons.size())+" "+QString::number(vortons.size()*3)+"\n";
        for (int i=0; i<vortons.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
        traceStream<<"POINT_DATA "+QString::number(vortons.size()*2)+"\n";
        traceStream<<"SCALARS gamma double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
        }
        traceStream.flush();
        traceStream<<"SCALARS velocity double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
        }
        traceStream.flush();

    }
    traceFrFile.close();

    QFile traceVFile(path+"/field/fieldVortons.vtk."+QString::number(currentStep));
    if (traceVFile.open(QIODevice::WriteOnly))
    {
        QTextStream traceStream(&traceVFile);
        traceStream<<"# vtk DataFile Version 3.0\n";
        traceStream<<"vtk output\n";
        traceStream<<"ASCII\n";
        traceStream<<"DATASET POLYDATA\n";
        traceStream<<"POINTS "+QString::number(vortons.size()*2)+" float\n";
        for (int i=0; i<vortons.size();i++)
        {
            Vector3D mid=vortons[i].getMid();
            Vector3D move=moveFromVortons[i];

            traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
            traceStream<<QString::number(mid.x()+move.x())+" "+QString::number(mid.y()+move.y())+" "+QString::number(mid.z()+move.z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(vortons.size())+" "+QString::number(vortons.size()*3)+"\n";
        for (int i=0; i<vortons.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
        traceStream<<"POINT_DATA "+QString::number(vortons.size()*2)+"\n";
        traceStream<<"SCALARS gamma double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
        }
        traceStream.flush();
        traceStream<<"SCALARS velocity double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
        }
        traceStream.flush();

    }
    traceVFile.close();

    QFile traceGBFile(path+"/field/fieldGetBack.vtk."+QString::number(currentStep));
    if (traceGBFile.open(QIODevice::WriteOnly))
    {
        QTextStream traceStream(&traceGBFile);
        traceStream<<"# vtk DataFile Version 3.0\n";
        traceStream<<"vtk output\n";
        traceStream<<"ASCII\n";
        traceStream<<"DATASET POLYDATA\n";
        traceStream<<"POINTS "+QString::number(vortons.size()*2)+" float\n";
        for (int i=0; i<vortons.size();i++)
        {
            Vector3D mid=vortons[i].getMid();
            Vector3D move=moveFromGetBack[i];

            traceStream<<QString::number(mid.x())+" "+QString::number(mid.y())+" "+QString::number(mid.z())+"\n";
            traceStream<<QString::number(mid.x()+move.x())+" "+QString::number(mid.y()+move.y())+" "+QString::number(mid.z()+move.z())+"\n";
        }
        traceStream.flush();
        traceStream<<"LINES "+QString::number(vortons.size())+" "+QString::number(vortons.size()*3)+"\n";
        for (int i=0; i<vortons.size()*2;i=i+2)
        {
            traceStream<<"2 "+QString::number(i)+" "+QString::number(i+1)+"\n";
        }
        traceStream.flush();
        traceStream<<"POINT_DATA "+QString::number(vortons.size()*2)+"\n";
        traceStream<<"SCALARS gamma double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
            traceStream<<QString::number(vortons[i].getVorticity())+"\n";
        }
        traceStream.flush();
        traceStream<<"SCALARS velocity double 1\n";
        traceStream<<"LOOKUP_TABLE default\n";
        for (int i=0; i<vortons.size();i++)
        {
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
            traceStream<<QString::number(vortons[i].getMove().length()/tau)+"\n";
        }
        traceStream.flush();

    }
    traceGBFile.close();
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

void Logger::createCenterGraphs(FormingParameters pars, double step, int currentStep, QVector<Vorton>& freeVortons, Vector3D velInf, QVector<std::shared_ptr<MultiFrame>>& xFrames, QVector<std::shared_ptr<MultiFrame> > &yFrames, QVector<std::shared_ptr<MultiFrame> > &zFrames)
{
    QFile xFile(path+"/graphs/y0Graph.vtk."+QString::number(currentStep));
    if (xFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&xFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<xFrames.size();i++)
            points+=xFrames[i]->getAnglesNum();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<xFrames.size();i++)
        {
            for (int j=0; j<xFrames[i]->getAnglesNum(); j++)
                yTextStream<<xFrames[i]->at(j).getTail().x()<<" "<<xFrames[i]->at(j).getTail().y()<<" "<<xFrames[i]->at(j).getTail().z()<<"\n";
            yTextStream.flush();
        }

        yTextStream<<"CELLS "<<xFrames.size()<<" "<<points+xFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<xFrames.size(); i++)
        {
            yTextStream<<xFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+xFrames[i]->getAnglesNum();j++)
            {
                yTextStream<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            yTextStream<<"\n";
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<xFrames.size()<<"\n";
        for (int i=0;i<xFrames.size(); i++)
            xFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<xFrames.size()<<"\n";
        QVector<Vector3D> velocities;
        for (int i=0; i<xFrames.size();i++)
            velocities.push_back(FrameCalculations::velocity(xFrames[i]->getCenter(),velInf,freeVortons));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    xFile.close();

    QFile yFile(path+"/graphs/z0Graph.vtk."+QString::number(currentStep));
    if (yFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&yFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<yFrames.size();i++)
            points+=yFrames[i]->getAnglesNum();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<yFrames.size();i++)
        {
            for (int j=0; j<yFrames[i]->getAnglesNum(); j++)
                yTextStream<<yFrames[i]->at(j).getTail().x()<<" "<<yFrames[i]->at(j).getTail().y()<<" "<<yFrames[i]->at(j).getTail().z()<<"\n";
            yTextStream.flush();
        }

        yTextStream<<"CELLS "<<yFrames.size()<<" "<<points+yFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<yFrames.size(); i++)
        {
            yTextStream<<yFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+yFrames[i]->getAnglesNum();j++)
            {
                yTextStream<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            yTextStream<<"\n";
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<yFrames.size()<<"\n";
        for (int i=0;i<yFrames.size(); i++)
            yFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<yFrames.size()<<"\n";
        QVector<Vector3D> velocities;
        for (int i=0; i<yFrames.size();i++)
            velocities.push_back(FrameCalculations::velocity(yFrames[i]->getCenter(),velInf,freeVortons));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    yFile.close();

    QFile zFile(path+"/graphs/x0Graph.vtk."+QString::number(currentStep));
    if (zFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&zFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<zFrames.size();i++)
            points+=zFrames[i]->getAnglesNum();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<zFrames.size();i++)
        {
            for (int j=0; j<zFrames[i]->getAnglesNum(); j++)
                yTextStream<<zFrames[i]->at(j).getTail().x()<<" "<<zFrames[i]->at(j).getTail().y()<<" "<<zFrames[i]->at(j).getTail().z()<<"\n";
            yTextStream.flush();
        }

        yTextStream<<"CELLS "<<zFrames.size()<<" "<<points+zFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<zFrames.size(); i++)
        {
            yTextStream<<zFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+zFrames[i]->getAnglesNum();j++)
            {
                yTextStream<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            yTextStream<<"\n";
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<zFrames.size()<<"\n";
        for (int i=0;i<zFrames.size(); i++)
            zFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<zFrames.size()<<"\n";
        QVector<Vector3D> velocities;
        for (int i=0; i<zFrames.size();i++)
            velocities.push_back(FrameCalculations::velocity(zFrames[i]->getCenter(),velInf,freeVortons));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    zFile.close();
}

void Logger::createCenterGraphs(FormingParameters pars, double step, int currentStep, QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, Vector3D velInf, QVector<std::shared_ptr<MultiFrame> > &xFrames, QVector<std::shared_ptr<MultiFrame> > &yFrames, QVector<std::shared_ptr<MultiFrame> > &zFrames,QVector<std::shared_ptr<MultiFrame> > &frames, int inter)
{
    if (inter==0)
    {
    QFile xFile(path+"/graphs/y00Graph.vtk."+QString::number(currentStep));
    if (xFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&xFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<xFrames.size();i++)
            points+=xFrames[i]->getAnglesNum();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<xFrames.size();i++)
        {
            for (int j=0; j<xFrames[i]->getAnglesNum(); j++)
                yTextStream<<xFrames[i]->at(j).getTail().x()<<" "<<xFrames[i]->at(j).getTail().y()<<" "<<xFrames[i]->at(j).getTail().z()<<"\n";
            yTextStream.flush();
        }

        yTextStream<<"CELLS "<<xFrames.size()<<" "<<points+xFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<xFrames.size(); i++)
        {
            yTextStream<<xFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+xFrames[i]->getAnglesNum();j++)
            {
                yTextStream<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            yTextStream<<"\n";
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<xFrames.size()<<"\n";
        for (int i=0;i<xFrames.size(); i++)
            xFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<xFrames.size()<<"\n";
        QVector<Vector3D> velocities;
        for (int i=0; i<xFrames.size();i++)
            velocities.push_back(FrameCalculations::velocity(xFrames[i]->getCenter(),velInf,freeVortons,frames));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    xFile.close();

    QFile yFile(path+"/graphs/z00Graph.vtk."+QString::number(currentStep));
    if (yFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&yFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<yFrames.size();i++)
            points+=yFrames[i]->getAnglesNum();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<yFrames.size();i++)
        {
            for (int j=0; j<yFrames[i]->getAnglesNum(); j++)
                yTextStream<<yFrames[i]->at(j).getTail().x()<<" "<<yFrames[i]->at(j).getTail().y()<<" "<<yFrames[i]->at(j).getTail().z()<<"\n";
            yTextStream.flush();
        }

        yTextStream<<"CELLS "<<yFrames.size()<<" "<<points+yFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<yFrames.size(); i++)
        {
            yTextStream<<yFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+yFrames[i]->getAnglesNum();j++)
            {
                yTextStream<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            yTextStream<<"\n";
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<yFrames.size()<<"\n";
        for (int i=0;i<yFrames.size(); i++)
            yFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<yFrames.size()<<"\n";
        QVector<Vector3D> velocities;
        for (int i=0; i<yFrames.size();i++)
            velocities.push_back(FrameCalculations::velocity(yFrames[i]->getCenter(),velInf,freeVortons,frames));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    yFile.close();

    QFile zFile(path+"/graphs/x00Graph.vtk."+QString::number(currentStep));
    if (zFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&zFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        for (int i=0; i<zFrames.size();i++)
            points+=zFrames[i]->getAnglesNum();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<zFrames.size();i++)
        {
            for (int j=0; j<zFrames[i]->getAnglesNum(); j++)
                yTextStream<<zFrames[i]->at(j).getTail().x()<<" "<<zFrames[i]->at(j).getTail().y()<<" "<<zFrames[i]->at(j).getTail().z()<<"\n";
            yTextStream.flush();
        }

        yTextStream<<"CELLS "<<zFrames.size()<<" "<<points+zFrames.size()<<"\n";
        int currentPointNumber=0;
        for (int i=0; i<zFrames.size(); i++)
        {
            yTextStream<<zFrames[i]->getAnglesNum();
            int virtualNumber=currentPointNumber;
            for (int j=currentPointNumber; j<currentPointNumber+zFrames[i]->getAnglesNum();j++)
            {
                yTextStream<<" "<<j;
                virtualNumber++;
            }
            currentPointNumber=virtualNumber;
            yTextStream<<"\n";
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<zFrames.size()<<"\n";
        for (int i=0;i<zFrames.size(); i++)
            zFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<zFrames.size()<<"\n";
        QVector<Vector3D> velocities;
        for (int i=0; i<zFrames.size();i++)
            velocities.push_back(FrameCalculations::velocity(zFrames[i]->getCenter(),velInf,freeVortons,frames));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    zFile.close();
    }
    if (inter==1)
    {
        QFile xFile(path+"/graphs/y01Graph.vtk."+QString::number(currentStep));
        if (xFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&xFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            for (int i=0; i<xFrames.size();i++)
                points+=xFrames[i]->getAnglesNum();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<xFrames.size();i++)
            {
                for (int j=0; j<xFrames[i]->getAnglesNum(); j++)
                    yTextStream<<xFrames[i]->at(j).getTail().x()<<" "<<xFrames[i]->at(j).getTail().y()<<" "<<xFrames[i]->at(j).getTail().z()<<"\n";
                yTextStream.flush();
            }

            yTextStream<<"CELLS "<<xFrames.size()<<" "<<points+xFrames.size()<<"\n";
            int currentPointNumber=0;
            for (int i=0; i<xFrames.size(); i++)
            {
                yTextStream<<xFrames[i]->getAnglesNum();
                int virtualNumber=currentPointNumber;
                for (int j=currentPointNumber; j<currentPointNumber+xFrames[i]->getAnglesNum();j++)
                {
                    yTextStream<<" "<<j;
                    virtualNumber++;
                }
                currentPointNumber=virtualNumber;
                yTextStream<<"\n";
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<xFrames.size()<<"\n";
            for (int i=0;i<xFrames.size(); i++)
                xFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<xFrames.size()<<"\n";
            QVector<Vector3D> velocities;
            for (int i=0; i<xFrames.size();i++)
                velocities.push_back(FrameCalculations::velocity(xFrames[i]->getCenter(),velInf,freeVortons+newVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        xFile.close();

        QFile yFile(path+"/graphs/z01Graph.vtk."+QString::number(currentStep));
        if (yFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&yFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            for (int i=0; i<yFrames.size();i++)
                points+=yFrames[i]->getAnglesNum();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<yFrames.size();i++)
            {
                for (int j=0; j<yFrames[i]->getAnglesNum(); j++)
                    yTextStream<<yFrames[i]->at(j).getTail().x()<<" "<<yFrames[i]->at(j).getTail().y()<<" "<<yFrames[i]->at(j).getTail().z()<<"\n";
                yTextStream.flush();
            }

            yTextStream<<"CELLS "<<yFrames.size()<<" "<<points+yFrames.size()<<"\n";
            int currentPointNumber=0;
            for (int i=0; i<yFrames.size(); i++)
            {
                yTextStream<<yFrames[i]->getAnglesNum();
                int virtualNumber=currentPointNumber;
                for (int j=currentPointNumber; j<currentPointNumber+yFrames[i]->getAnglesNum();j++)
                {
                    yTextStream<<" "<<j;
                    virtualNumber++;
                }
                currentPointNumber=virtualNumber;
                yTextStream<<"\n";
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<yFrames.size()<<"\n";
            for (int i=0;i<yFrames.size(); i++)
                yFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<yFrames.size()<<"\n";
            QVector<Vector3D> velocities;
            for (int i=0; i<yFrames.size();i++)
                velocities.push_back(FrameCalculations::velocity(yFrames[i]->getCenter(),velInf,freeVortons+newVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        yFile.close();

        QFile zFile(path+"/graphs/x01Graph.vtk."+QString::number(currentStep));
        if (zFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&zFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            for (int i=0; i<zFrames.size();i++)
                points+=zFrames[i]->getAnglesNum();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<zFrames.size();i++)
            {
                for (int j=0; j<zFrames[i]->getAnglesNum(); j++)
                    yTextStream<<zFrames[i]->at(j).getTail().x()<<" "<<zFrames[i]->at(j).getTail().y()<<" "<<zFrames[i]->at(j).getTail().z()<<"\n";
                yTextStream.flush();
            }

            yTextStream<<"CELLS "<<zFrames.size()<<" "<<points+zFrames.size()<<"\n";
            int currentPointNumber=0;
            for (int i=0; i<zFrames.size(); i++)
            {
                yTextStream<<zFrames[i]->getAnglesNum();
                int virtualNumber=currentPointNumber;
                for (int j=currentPointNumber; j<currentPointNumber+zFrames[i]->getAnglesNum();j++)
                {
                    yTextStream<<" "<<j;
                    virtualNumber++;
                }
                currentPointNumber=virtualNumber;
                yTextStream<<"\n";
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<zFrames.size()<<"\n";
            for (int i=0;i<zFrames.size(); i++)
                zFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<zFrames.size()<<"\n";
            QVector<Vector3D> velocities;
            for (int i=0; i<zFrames.size();i++)
                velocities.push_back(FrameCalculations::velocity(zFrames[i]->getCenter(),velInf,freeVortons+newVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        zFile.close();
    }
    if (inter==2)
    {
        QFile xFile(path+"/graphs/y02Graph.vtk."+QString::number(currentStep));
        if (xFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&xFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            for (int i=0; i<xFrames.size();i++)
                points+=xFrames[i]->getAnglesNum();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<xFrames.size();i++)
            {
                for (int j=0; j<xFrames[i]->getAnglesNum(); j++)
                    yTextStream<<xFrames[i]->at(j).getTail().x()<<" "<<xFrames[i]->at(j).getTail().y()<<" "<<xFrames[i]->at(j).getTail().z()<<"\n";
                yTextStream.flush();
            }

            yTextStream<<"CELLS "<<xFrames.size()<<" "<<points+xFrames.size()<<"\n";
            int currentPointNumber=0;
            for (int i=0; i<xFrames.size(); i++)
            {
                yTextStream<<xFrames[i]->getAnglesNum();
                int virtualNumber=currentPointNumber;
                for (int j=currentPointNumber; j<currentPointNumber+xFrames[i]->getAnglesNum();j++)
                {
                    yTextStream<<" "<<j;
                    virtualNumber++;
                }
                currentPointNumber=virtualNumber;
                yTextStream<<"\n";
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<xFrames.size()<<"\n";
            for (int i=0;i<xFrames.size(); i++)
                xFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<xFrames.size()<<"\n";
            QVector<Vector3D> velocities;
            for (int i=0; i<xFrames.size();i++)
                velocities.push_back(FrameCalculations::velocity(xFrames[i]->getCenter(),velInf,freeVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        xFile.close();

        QFile yFile(path+"/graphs/z02Graph.vtk."+QString::number(currentStep));
        if (yFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&yFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            for (int i=0; i<yFrames.size();i++)
                points+=yFrames[i]->getAnglesNum();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<yFrames.size();i++)
            {
                for (int j=0; j<yFrames[i]->getAnglesNum(); j++)
                    yTextStream<<yFrames[i]->at(j).getTail().x()<<" "<<yFrames[i]->at(j).getTail().y()<<" "<<yFrames[i]->at(j).getTail().z()<<"\n";
                yTextStream.flush();
            }

            yTextStream<<"CELLS "<<yFrames.size()<<" "<<points+yFrames.size()<<"\n";
            int currentPointNumber=0;
            for (int i=0; i<yFrames.size(); i++)
            {
                yTextStream<<yFrames[i]->getAnglesNum();
                int virtualNumber=currentPointNumber;
                for (int j=currentPointNumber; j<currentPointNumber+yFrames[i]->getAnglesNum();j++)
                {
                    yTextStream<<" "<<j;
                    virtualNumber++;
                }
                currentPointNumber=virtualNumber;
                yTextStream<<"\n";
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<yFrames.size()<<"\n";
            for (int i=0;i<yFrames.size(); i++)
                yFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<yFrames.size()<<"\n";
            QVector<Vector3D> velocities;
            for (int i=0; i<yFrames.size();i++)
                velocities.push_back(FrameCalculations::velocity(yFrames[i]->getCenter(),velInf,freeVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        yFile.close();

        QFile zFile(path+"/graphs/x02Graph.vtk."+QString::number(currentStep));
        if (zFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&zFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            for (int i=0; i<zFrames.size();i++)
                points+=zFrames[i]->getAnglesNum();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<zFrames.size();i++)
            {
                for (int j=0; j<zFrames[i]->getAnglesNum(); j++)
                    yTextStream<<zFrames[i]->at(j).getTail().x()<<" "<<zFrames[i]->at(j).getTail().y()<<" "<<zFrames[i]->at(j).getTail().z()<<"\n";
                yTextStream.flush();
            }

            yTextStream<<"CELLS "<<zFrames.size()<<" "<<points+zFrames.size()<<"\n";
            int currentPointNumber=0;
            for (int i=0; i<zFrames.size(); i++)
            {
                yTextStream<<zFrames[i]->getAnglesNum();
                int virtualNumber=currentPointNumber;
                for (int j=currentPointNumber; j<currentPointNumber+zFrames[i]->getAnglesNum();j++)
                {
                    yTextStream<<" "<<j;
                    virtualNumber++;
                }
                currentPointNumber=virtualNumber;
                yTextStream<<"\n";
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<zFrames.size()<<"\n";
            for (int i=0;i<zFrames.size(); i++)
                zFrames[i]->getAnglesNum()==4 ? yTextStream<<"9\n" : yTextStream<<"7\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<zFrames.size()<<"\n";
            QVector<Vector3D> velocities;
            for (int i=0; i<zFrames.size();i++)
                velocities.push_back(FrameCalculations::velocity(zFrames[i]->getCenter(),velInf,freeVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        zFile.close();
    }
}

void Logger::writeGammas(QVector<double> gammaMax,double initialGamma)
{
    *gammaTextStream.get()<<initialGamma<<"\t"<<gammaMax[0]<<"\t"<<gammaMax[1]<<"\t"<<gammaMax[2]<<"\n";
    gammaTextStream.get()->flush();
}

void Logger::createQuadroGraphs(int currentStep, QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, Vector3D velInf, QVector<QVector<Vector3D>> &graphNodesX, QVector<QVector<Vector3D>> &graphNodesY, QVector<QVector<Vector3D>> &graphNodesZ, QVector<std::shared_ptr<MultiFrame> > &frames, int inter)
{
    if (inter==0)
    {
    QFile xFile(path+"/graphs/x00Graph.vtk."+QString::number(currentStep));
    if (xFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&xFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        points=graphNodesX.size()*graphNodesX[0].size();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<graphNodesX.size();i++)
        {
            for (int j=0; j<graphNodesX[i].size(); j++)
                yTextStream<<graphNodesX[i][j].x()<<" "<<graphNodesX[i][j].y()<<" "<<graphNodesX[i][j].z()<<"\n";
            yTextStream.flush();
        }
        int cellSize=(graphNodesX.size()-1)*(graphNodesX[0].size()-1);
        yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
        for (int i=0; i<graphNodesX.size()-1; i++)
        {
            for (int j=0; j<graphNodesX[i].size()-1;j++)
                yTextStream<<"4 "<<i*graphNodesX[i].size()+j<<" "<<i*graphNodesX[i].size()+j+1<<" "<<(i+1)*graphNodesX[i].size()+j<<" "<<(i+1)*graphNodesX[i].size()+j+1<<"\n";
         yTextStream.flush();
        }

        yTextStream.flush();
        yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
        for (int i=0;i<cellSize; i++)
           yTextStream<<"8\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<cellSize<<"\n";

        QVector<Vector3D> centers;
        for (int i=0; i<graphNodesX.size()-1; i++)
            for (int j=0; j<graphNodesX[i].size()-1;j++)
               centers.push_back((graphNodesX[i+1][j]+graphNodesX[i][j+1])*0.5);

        QVector<Vector3D> velocities;
        for (int i=0; i<centers.size();i++)
            velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons,frames));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    xFile.close();

    QFile yFile(path+"/graphs/y00Graph.vtk."+QString::number(currentStep));
    if (yFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&yFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        points=graphNodesY.size()*graphNodesY[0].size();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<graphNodesY.size();i++)
        {
            for (int j=0; j<graphNodesY[i].size(); j++)
                yTextStream<<graphNodesY[i][j].x()<<" "<<graphNodesY[i][j].y()<<" "<<graphNodesY[i][j].z()<<"\n";
            yTextStream.flush();
        }
        int cellSize=(graphNodesY.size()-1)*(graphNodesY[0].size()-1);
        yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
        for (int i=0; i<graphNodesY.size()-1; i++)
        {
            for (int j=0; j<graphNodesY[i].size()-1;j++)
                yTextStream<<"4 "<<i*graphNodesY[i].size()+j<<" "<<i*graphNodesY[i].size()+j+1<<" "<<(i+1)*graphNodesY[i].size()+j<<" "<<(i+1)*graphNodesY[i].size()+j+1<<"\n";
         yTextStream.flush();
        }
        yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
        for (int i=0;i<cellSize; i++)
           yTextStream<<"8\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<cellSize<<"\n";

        QVector<Vector3D> centers;
        for (int i=0; i<graphNodesY.size()-1; i++)
            for (int j=0; j<graphNodesY[i].size()-1;j++)
               centers.push_back((graphNodesY[i+1][j]+graphNodesY[i][j+1])*0.5);

        QVector<Vector3D> velocities;
        for (int i=0; i<centers.size();i++)
            velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons,frames));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    yFile.close();

    QFile zFile(path+"/graphs/z00Graph.vtk."+QString::number(currentStep));
    if (zFile.open(QIODevice::WriteOnly))
    {
        QTextStream yTextStream(&zFile);
        yTextStream<<"# vtk DataFile Version 3.0\n";
        yTextStream<<"vtk output\n";
        yTextStream<<"ASCII\n";
        yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
        int points=0;
        points=graphNodesZ.size()*graphNodesZ[0].size();
        yTextStream<<"POINTS "<<points<<" float\n";
        yTextStream.flush();
        for (int i=0; i<graphNodesZ.size();i++)
        {
            for (int j=0; j<graphNodesZ[i].size(); j++)
                yTextStream<<graphNodesZ[i][j].x()<<" "<<graphNodesZ[i][j].y()<<" "<<graphNodesZ[i][j].z()<<"\n";
            yTextStream.flush();
        }
        int cellSize=(graphNodesZ.size()-1)*(graphNodesZ[0].size()-1);
        yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
        for (int i=0; i<graphNodesZ.size()-1; i++)
        {
            for (int j=0; j<graphNodesZ[i].size()-1;j++)
                yTextStream<<"4 "<<i*graphNodesZ[i].size()+j<<" "<<i*graphNodesZ[i].size()+j+1<<" "<<(i+1)*graphNodesZ[i].size()+j<<" "<<(i+1)*graphNodesZ[i].size()+j+1<<"\n";
         yTextStream.flush();
        }
        yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
        for (int i=0;i<cellSize; i++)
           yTextStream<<"8\n";
        yTextStream.flush();


        yTextStream<<"CELL_DATA "<<cellSize<<"\n";

        QVector<Vector3D> centers;
        for (int i=0; i<graphNodesZ.size()-1; i++)
            for (int j=0; j<graphNodesZ[i].size()-1;j++)
               centers.push_back((graphNodesZ[i+1][j]+graphNodesZ[i][j+1])*0.5);

        QVector<Vector3D> velocities;
        for (int i=0; i<centers.size();i++)
            velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons,frames));
        yTextStream<<"VECTORS velocities float\n";
        for (int i=0; i<velocities.size();i++)
            yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
        yTextStream.flush();
    }
    zFile.close();
    }
    if (inter==1)
    {
        QFile xFile(path+"/graphs/x01Graph.vtk."+QString::number(currentStep));
        if (xFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&xFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            points=graphNodesX.size()*graphNodesX[0].size();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<graphNodesX.size();i++)
            {
                for (int j=0; j<graphNodesX[i].size(); j++)
                    yTextStream<<graphNodesX[i][j].x()<<" "<<graphNodesX[i][j].y()<<" "<<graphNodesX[i][j].z()<<"\n";
                yTextStream.flush();
            }
            int cellSize=(graphNodesX.size()-1)*(graphNodesX[0].size()-1);
            yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
            for (int i=0; i<graphNodesX.size()-1; i++)
            {
                for (int j=0; j<graphNodesX[i].size()-1;j++)
                    yTextStream<<"4 "<<i*graphNodesX[i].size()+j<<" "<<i*graphNodesX[i].size()+j+1<<" "<<(i+1)*graphNodesX[i].size()+j<<" "<<(i+1)*graphNodesX[i].size()+j+1<<"\n";
             yTextStream.flush();
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
            for (int i=0;i<cellSize; i++)
               yTextStream<<"8\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<cellSize<<"\n";

            QVector<Vector3D> centers;
            for (int i=0; i<graphNodesX.size()-1; i++)
                for (int j=0; j<graphNodesX[i].size()-1;j++)
                   centers.push_back((graphNodesX[i+1][j]+graphNodesX[i][j+1])*0.5);

            QVector<Vector3D> velocities;
            for (int i=0; i<centers.size();i++)
                velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons+newVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        xFile.close();

        QFile yFile(path+"/graphs/y01Graph.vtk."+QString::number(currentStep));
        if (yFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&yFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            points=graphNodesY.size()*graphNodesY[0].size();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<graphNodesY.size();i++)
            {
                for (int j=0; j<graphNodesY[i].size(); j++)
                    yTextStream<<graphNodesY[i][j].x()<<" "<<graphNodesY[i][j].y()<<" "<<graphNodesY[i][j].z()<<"\n";
                yTextStream.flush();
            }
            int cellSize=(graphNodesY.size()-1)*(graphNodesY[0].size()-1);
            yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
            for (int i=0; i<graphNodesY.size()-1; i++)
            {
                for (int j=0; j<graphNodesY[i].size()-1;j++)
                    yTextStream<<"4 "<<i*graphNodesY[i].size()+j<<" "<<i*graphNodesY[i].size()+j+1<<" "<<(i+1)*graphNodesY[i].size()+j<<" "<<(i+1)*graphNodesY[i].size()+j+1<<"\n";
             yTextStream.flush();
            }
            yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
            for (int i=0;i<cellSize; i++)
               yTextStream<<"8\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<cellSize<<"\n";

            QVector<Vector3D> centers;
            for (int i=0; i<graphNodesY.size()-1; i++)
                for (int j=0; j<graphNodesY[i].size()-1;j++)
                   centers.push_back((graphNodesY[i+1][j]+graphNodesY[i][j+1])*0.5);

            QVector<Vector3D> velocities;
            for (int i=0; i<centers.size();i++)
                velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons+newVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        yFile.close();

        QFile zFile(path+"/graphs/z01Graph.vtk."+QString::number(currentStep));
        if (zFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&zFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            points=graphNodesZ.size()*graphNodesZ[0].size();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<graphNodesZ.size();i++)
            {
                for (int j=0; j<graphNodesZ[i].size(); j++)
                    yTextStream<<graphNodesZ[i][j].x()<<" "<<graphNodesZ[i][j].y()<<" "<<graphNodesZ[i][j].z()<<"\n";
                yTextStream.flush();
            }
            int cellSize=(graphNodesZ.size()-1)*(graphNodesZ[0].size()-1);
            yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
            for (int i=0; i<graphNodesZ.size()-1; i++)
            {
                for (int j=0; j<graphNodesZ[i].size()-1;j++)
                    yTextStream<<"4 "<<i*graphNodesZ[i].size()+j<<" "<<i*graphNodesZ[i].size()+j+1<<" "<<(i+1)*graphNodesZ[i].size()+j<<" "<<(i+1)*graphNodesZ[i].size()+j+1<<"\n";
             yTextStream.flush();
            }
            yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
            for (int i=0;i<cellSize; i++)
               yTextStream<<"8\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<cellSize<<"\n";

            QVector<Vector3D> centers;
            for (int i=0; i<graphNodesZ.size()-1; i++)
                for (int j=0; j<graphNodesZ[i].size()-1;j++)
                   centers.push_back((graphNodesZ[i+1][j]+graphNodesZ[i][j+1])*0.5);

            QVector<Vector3D> velocities;
            for (int i=0; i<centers.size();i++)
                velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons+newVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        zFile.close();
    }
    if (inter==2)
    {
        QFile xFile(path+"/graphs/x02Graph.vtk."+QString::number(currentStep));
        if (xFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&xFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            points=graphNodesX.size()*graphNodesX[0].size();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<graphNodesX.size();i++)
            {
                for (int j=0; j<graphNodesX[i].size(); j++)
                    yTextStream<<graphNodesX[i][j].x()<<" "<<graphNodesX[i][j].y()<<" "<<graphNodesX[i][j].z()<<"\n";
                yTextStream.flush();
            }
            int cellSize=(graphNodesX.size()-1)*(graphNodesX[0].size()-1);
            yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
            for (int i=0; i<graphNodesX.size()-1; i++)
            {
                for (int j=0; j<graphNodesX[i].size()-1;j++)
                    yTextStream<<"4 "<<i*graphNodesX[i].size()+j<<" "<<i*graphNodesX[i].size()+j+1<<" "<<(i+1)*graphNodesX[i].size()+j<<" "<<(i+1)*graphNodesX[i].size()+j+1<<"\n";
             yTextStream.flush();
            }

            yTextStream.flush();
            yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
            for (int i=0;i<cellSize; i++)
               yTextStream<<"8\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<cellSize<<"\n";

            QVector<Vector3D> centers;
            for (int i=0; i<graphNodesX.size()-1; i++)
                for (int j=0; j<graphNodesX[i].size()-1;j++)
                   centers.push_back((graphNodesX[i+1][j]+graphNodesX[i][j+1])*0.5);

            QVector<Vector3D> velocities;
            for (int i=0; i<centers.size();i++)
                velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        xFile.close();

        QFile yFile(path+"/graphs/y02Graph.vtk."+QString::number(currentStep));
        if (yFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&yFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            points=graphNodesY.size()*graphNodesY[0].size();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<graphNodesY.size();i++)
            {
                for (int j=0; j<graphNodesY[i].size(); j++)
                    yTextStream<<graphNodesY[i][j].x()<<" "<<graphNodesY[i][j].y()<<" "<<graphNodesY[i][j].z()<<"\n";
                yTextStream.flush();
            }
            int cellSize=(graphNodesY.size()-1)*(graphNodesY[0].size()-1);
            yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
            for (int i=0; i<graphNodesY.size()-1; i++)
            {
                for (int j=0; j<graphNodesY[i].size()-1;j++)
                    yTextStream<<"4 "<<i*graphNodesY[i].size()+j<<" "<<i*graphNodesY[i].size()+j+1<<" "<<(i+1)*graphNodesY[i].size()+j<<" "<<(i+1)*graphNodesY[i].size()+j+1<<"\n";
             yTextStream.flush();
            }
            yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
            for (int i=0;i<cellSize; i++)
               yTextStream<<"8\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<cellSize<<"\n";

            QVector<Vector3D> centers;
            for (int i=0; i<graphNodesY.size()-1; i++)
                for (int j=0; j<graphNodesY[i].size()-1;j++)
                   centers.push_back((graphNodesY[i+1][j]+graphNodesY[i][j+1])*0.5);

            QVector<Vector3D> velocities;
            for (int i=0; i<centers.size();i++)
                velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        yFile.close();

        QFile zFile(path+"/graphs/z02Graph.vtk."+QString::number(currentStep));
        if (zFile.open(QIODevice::WriteOnly))
        {
            QTextStream yTextStream(&zFile);
            yTextStream<<"# vtk DataFile Version 3.0\n";
            yTextStream<<"vtk output\n";
            yTextStream<<"ASCII\n";
            yTextStream<<"DATASET UNSTRUCTURED_GRID\n";
            int points=0;
            points=graphNodesZ.size()*graphNodesZ[0].size();
            yTextStream<<"POINTS "<<points<<" float\n";
            yTextStream.flush();
            for (int i=0; i<graphNodesZ.size();i++)
            {
                for (int j=0; j<graphNodesZ[i].size(); j++)
                    yTextStream<<graphNodesZ[i][j].x()<<" "<<graphNodesZ[i][j].y()<<" "<<graphNodesZ[i][j].z()<<"\n";
                yTextStream.flush();
            }
            int cellSize=(graphNodesZ.size()-1)*(graphNodesZ[0].size()-1);
            yTextStream<<"CELLS "<<cellSize<<" "<<cellSize*5<<"\n";
            for (int i=0; i<graphNodesZ.size()-1; i++)
            {
                for (int j=0; j<graphNodesZ[i].size()-1;j++)
                    yTextStream<<"4 "<<i*graphNodesZ[i].size()+j<<" "<<i*graphNodesZ[i].size()+j+1<<" "<<(i+1)*graphNodesZ[i].size()+j<<" "<<(i+1)*graphNodesZ[i].size()+j+1<<"\n";
             yTextStream.flush();
            }
            yTextStream<<"CELL_TYPES "<<cellSize<<"\n";
            for (int i=0;i<cellSize; i++)
               yTextStream<<"8\n";
            yTextStream.flush();


            yTextStream<<"CELL_DATA "<<cellSize<<"\n";

            QVector<Vector3D> centers;
            for (int i=0; i<graphNodesZ.size()-1; i++)
                for (int j=0; j<graphNodesZ[i].size()-1;j++)
                   centers.push_back((graphNodesZ[i+1][j]+graphNodesZ[i][j+1])*0.5);

            QVector<Vector3D> velocities;
            for (int i=0; i<centers.size();i++)
                velocities.push_back(FrameCalculations::velocity(centers[i],velInf,freeVortons));
            yTextStream<<"VECTORS velocities float\n";
            for (int i=0; i<velocities.size();i++)
                yTextStream<<velocities[i].x()<<" "<<velocities[i].y()<<" "<<velocities[i].z()<<"\n";
            yTextStream.flush();
        }
        zFile.close();
    }
}

QVector<Vorton> Logger::gaVortons(const QString vortonsDir, int currentFileNum)
{
    QVector<Vorton> gaVortonsVec;
    QDir folder(vortonsDir);
    QStringList files=folder.entryList();
    if (currentFileNum+2<files.size())
    {
        QString vortonsPath=vortonsDir+"/"+files[currentFileNum+2];
        qDebug()<<vortonsPath;
        QFile vortonFile(vortonsPath);
        if (vortonFile.open(QIODevice::ReadOnly))
        {
            QString vortonsNum=vortonFile.readLine();
            int size=vortonsNum.toInt();
            for (int i=0; i<size; i++)
            {
                vortonFile.readLine();
                QString gammaStr=vortonFile.readLine();
                double gamma=gammaStr.toDouble();
                QString r0Str=vortonFile.readLine();
                QStringList r0StrL=r0Str.split(" ",QString::SkipEmptyParts);
                Vorton newVort;
                newVort.setVorticity(gamma);
                newVort.setRadius(0.15);
                newVort.setMid(Vector3D(r0StrL[0].toDouble(),r0StrL[1].toDouble(),r0StrL[2].toDouble()));
                QString r1Str=vortonFile.readLine();
                QStringList r1StrL=r1Str.split(" ",QString::SkipEmptyParts);
                newVort.setTail(newVort.getMid()+Vector3D(r1StrL[0].toDouble(),r1StrL[1].toDouble(),r1StrL[2].toDouble()));
                gaVortonsVec.append(newVort);
            }
        }
        vortonFile.close();
    }
    return gaVortonsVec;
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
    if (gammaFile->isOpen())
        gammaFile->close();
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
    qDebug()<<"kek";
    QStringList fileList=folder.entryList();
     qDebug()<<"kek1";
    for (int i=2; i<fileList.size(); i++)
    {
        QString vortonsPath=vortonsDir+"/"+fileList[i];
        QFile vortonsFile(vortonsPath);
        if (vortonsFile.open(QIODevice::ReadOnly))
        {
            QString line = vortonsFile.readLine();
            QStringList lineSplitted=line.split("\t", QString::SkipEmptyParts);
            int vortonsSize=lineSplitted[0].toInt();

            int framesSize=0;
            if (lineSplitted.size()!=1)
            framesSize= lineSplitted[1].toInt();
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


void TriangleVectors::calcParameters(const double raise)
{
    normal=Vector3D::crossProduct(a,b).normalized();
    center=Vector3D((a.x()+b.x()+c.x())/3.0,(a.y()+b.y()+c.y())/3.0,(a.z()+b.z()+c.z())/3.0);
    controlPointRaised=center+raise*normal;
    square=0.5*Vector3D::crossProduct(a,b).length();
}
