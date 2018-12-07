#include "framecalculations.h"

/*!
Обнуляет значения всех счетчиков
*/
void Counters::clear()
{
    unitedNum=vorticityEliminated=tooFarNum=rotatedNum=gotBackNum=underScreenNum=0;
}

/*!
Обнуляет значения всех ограничений
*/
void Restrictions::clear()
{
    moveRestr=elongationRestr=turnRestr=0;
}

/*!
Обнуляет значения всех таймеров
*/
void Timers::clear()
{
    getBackAndRotateTimer=forceTimer=unionTimer=farTimer=integrationTimer=removeVorticityTimer=0.0;
}

/*!
Создает объект класса для работы с рамками с обнуленными счетчиками, ограничениями и таймерами
*/
FrameCalculations::FrameCalculations()
{

    counters.clear();
    restrictions.clear();
    timers.clear();
}

/*!
Рассчитывает значения углов тета для сферы
\param tetaFragNum Количество разбиений тела по тета
\return Вектор с рассчитанными значениями углов тета
*/
QVector<double> FrameCalculations::calcTetas(const int tetaFragNum)
{
    QVector<double> tetas;
    tetas.push_back(0.0);
    for (int i=1; i<tetaFragNum+2;i++)
        tetas.push_back(M_PI/(tetaFragNum+2)*(0.5+i));
    tetas.push_back(M_PI);
    return tetas;
}

/*!
Рассчитывает значения элементов матрицы, составленной из произведения единичных интенсивностей от каждой из рамок на соответствующую нормаль и вычисляет обратную к ней
\param frames Вектор рамок
\param controlPoints Вектор контрольных точек
\param normals Вектор нормалей
*/
void FrameCalculations::matrixCalc(QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    matrixSize=frames.size()+1;
    matrix.resize(matrixSize,matrixSize);
    for (int i=0; i<matrixSize-1; i++)
    {
        for (int j=0; j<matrixSize-1; j++)
            matrix(i,j)=Vector3D::dotProduct(frames[j]->qHelp(controlPoints[i]), normals[i]);
    }
    for (int i=0; i<matrixSize-1; i++)
        matrix(matrixSize-1,i)=matrix(i,matrixSize-1)=1.0; 
    matrix(matrixSize-1,matrixSize-1)=0.0;

    double tqw=matrix.determinant();
    QVector<double> mat(matrixSize);
    for (int i=0; i<matrixSize; i++)
    {
        mat[i]=0.0;
        for (int j=0; j<matrixSize; j++)
        {
            mat[i]+=matrix(i,j);
        }
    }
    double normA=mat[0];
    for (int i=1; i<matrixSize; i++)
    {
        if (mat[i]>normA)
            normA=mat[i];
    }
    Eigen::MatrixXd oldmatrix;
    oldmatrix.resize(matrixSize,matrixSize);
    oldmatrix=matrix;

    matrix=matrix.inverse();
    QVector<double> mat1(matrixSize);
    for (int i=0; i<matrixSize; i++)
    {
        for (int j=0; j<matrixSize; j++)
        {
            mat1[i]+=matrix(i,j);
        }
    }
    double normA1=mat1[0];
    for (int i=1; i<matrixSize; i++)
    {
        if (mat1[i]>normA1)
            normA1=mat1[i];
    }
    qDebug()<<"Число обусловленности"<<normA*normA1;
    int k=3;
    //matrix=matrix.inverse();

}

/*!
Рассчитывает значения столбца b для решения СЛАУ вида A*x=b, где А-матрица, х-искомый столбец.
\param streamVel Скорость потока
\param vortons Вектор содержащий текущие вортоны в потоке
\param normals Вектор нормалей
\param controlPoints Вектор контрольных точек
\return Столбец b
*/
Eigen::VectorXd FrameCalculations::columnCalc(const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<Vector3D> &normals, const QVector<Vector3D> controlPoints)
{
    Eigen::VectorXd column(matrixSize);
    for (int i=0; i<controlPoints.size(); i++)
        column(i)=Vector3D::dotProduct(-FrameCalculations::velocity(controlPoints[i], streamVel, vortons), normals[i]);
    column(controlPoints.size())=0.0;
    return column;
}

/*!
Рассчитывает значения столбца x путем решения СЛАУ вида A*x=b, где А-матрица, b-известный столбец.
\param column Столбец b
\return Столбец x, содержащий значения завихренностей рамок
*/
Eigen::VectorXd FrameCalculations::vorticitiesCalc(const Eigen::VectorXd &column)
{
    return Eigen::VectorXd(matrix*column);
}

/*!
Функция объединения близлежащих вортонов
\param[in,out] vortons Вектор, содержащий вортоны для объединения
\param[in] eStar Максимальное расстояние для объединения
\param[in] eDoubleStar Минимальный косинус угла для объединения
\param[in] vortonRad Радиус вортон-отрезков
*/
void FrameCalculations::unionVortons(QVector<Vorton> &vortons,const double eStar,const double eDoubleStar,const double vortonRad)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        for (int j=i-1; j>=0; j--)
        {
            if((vortons[i].getMid()-vortons[j].getMid()).length()<eStar)
            {
                double psi=Vector3D::dotProduct((vortons[i].getTail()-vortons[i].getMid()).normalized(),(vortons[j].getTail()-vortons[j].getMid()).normalized());
                if (fabs(psi)>eDoubleStar)
                {
                    if (psi<0)
                        vortons[j].turn();
                    counters.unitedNum++;
                    Vector3D newMid = (fabs(vortons[i].getVorticity())*vortons[i].getMid()+fabs(vortons[j].getVorticity())*vortons[j].getMid())/
                    (fabs(vortons[i].getVorticity())+fabs(vortons[j].getVorticity()));
                    Vector3D selfLen = (fabs(vortons[i].getVorticity())*(vortons[i].getTail()-vortons[i].getMid())+fabs(vortons[j].getVorticity())*(vortons[j].getTail()-vortons[j].getMid()))/
                    (fabs(vortons[i].getVorticity())+fabs(vortons[j].getVorticity()));
                    double newVorticity=vortons[i].getVorticity()+vortons[j].getVorticity();
                    Vorton newVorton = Vorton (newMid, newMid+selfLen, newVorticity, vortonRad);
                    vortons[i]=newVorton;
                    i--;
                    vortons.remove(j);
                    break;
                }

            }
        }
    }
    timers.unionTimer=start.elapsed()*0.01;
}

/*!
Функция удаления вортонов с малой завихренностью
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] minVorticity Минимальное значение завихренности для объединения
*/
void FrameCalculations::removeSmallVorticity(QVector<Vorton> &vortons,const double minVorticity)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (fabs(vortons[i].getVorticity())<minVorticity)
        {
            vortons.remove(i);
            counters.vorticityEliminated++;
        }
    }
    timers.removeVorticityTimer=start.elapsed()*0.01;
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра сферы
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата центра сферы
*/
void FrameCalculations::removeFarSphere(QVector<Vorton> &vortons, const double farDistance, const Vector3D bodyCenter)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if ((vortons[i].getMid()-bodyCenter).length()>farDistance)
        {
            vortons.remove(i);
            counters.tooFarNum++;
        }
    }
    timers.farTimer=start.elapsed()*0.01;
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра цилиндра
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата высота цилиндра
*/
void FrameCalculations::removeFarCylinder(QVector<Vorton> &vortons, const double farDistance, const double height)
{
    QTime start = QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if ((vortons[i].getMid().y()>height+farDistance)||(vortons[i].getMid().y()<-farDistance)||sqrt(pow(vortons[i].getMid().x(),2)+pow(vortons[i].getMid().z(),2))>farDistance)
        {
            vortons.remove(i);
            counters.tooFarNum++;
        }
    }
    timers.farTimer=start.elapsed()*0.01;
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра тела вращения
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата центра тела вращения
*/
void FrameCalculations::removeFarRotationBody(QVector<Vorton>& vortons, const double farDistance, const Vector3D bodyCenter)
{
    removeFarSphere(vortons,farDistance,bodyCenter);
}

/*!
Функция удаления вортонов удаленных на большое расстояние от центра тела вращения со срезом дна
\param[in,out] vortons Вектор, содержащий вортоны для удаления
\param[in] farDistance Максимальное значение расстояния от центра для учета влияния
\param[in] bodyCenter Координата центра тела вращения со срезом дна
*/
void FrameCalculations::removeFarRotationCutBody(QVector<Vorton> &vortons, const double farDistance, const Vector3D bodyCenter)
{
    removeFarSphere(vortons,farDistance,bodyCenter);
}

/*!
Функция расчета перемещений и удалений для вортонов с рамок и вортонов в потоке
\param[in,out] freeVortons Вектор, содержащий вортоны в потоке
\param[in,out] newVortons Вектор, содержащий вортоны с рамок
\param[in] step Размер шага
\param[in] streamVel Скорость потока
\param[in] eDelta Максимальное значение удлинения
\param[in] fiMax Максимальное значение угла поворота
\param[in] maxMove Максимальное перемещения
*/
void FrameCalculations::displacementCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove)
{
    QTime start = QTime::currentTime();
    QVector<Vorton> vortons;
    vortons.append(freeVortons);
    vortons.append(newVortons);
    QVector<Parallel> paralVec;
    for (int i=0; i<vortons.size(); i++)
    {
        Parallel element {&vortons,nullptr, nullptr, i, streamVel, step};
        paralVec.push_back(element);
    }
    QVector<Vorton> resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacement, addToVortonsVec, QtConcurrent::OrderedReduce);

    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=fabs(selfLenBef.length()-selfLenAft.length());
        if (turnAngle>fiMax)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.turnRestr++;
        }
        if (lengthChange>eDelta)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.elongationRestr++;
        }
        if (resultedVec[i].getMove().length()>maxMove)
        {
            resultedVec[i].setMove(Vector3D());
            restrictions.moveRestr++;
        }

    }

    for (int i=0; i<freeVortons.size(); i++)
    {
        freeVortons[i].setMove(resultedVec[i].getMove());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.01;
}

/*!
Функция расчета перемещений и удалений для вортонов с рамок и вортонов в потоке при решении задачи старта
\param[in,out] freeVortons Вектор, содержащий вортоны в потоке
\param[in,out] newVortons Вектор, содержащий вортоны с рамок
\param[in,out] symFreeVortons Симметричный вектор вектору, содержащему вортоны в потоке
\param[in,out] symNewVortons Симметричный вектор вектору, содержащему вортоны с рамок
\param[in] step Размер шага
\param[in] streamVel Скорость потока
\param[in] eDelta Максимальное значение удлинения
\param[in] fiMax Максимальное значение угла поворота
\param[in] maxMove Максимальное перемещения
*/
void FrameCalculations::displacementLaunchCalc(QVector<Vorton> &freeVortons, QVector<Vorton> &newVortons, QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, double step, Vector3D streamVel, double eDelta, double fiMax, double maxMove)
{
    QTime start = QTime::currentTime();
    QVector<Vorton> vortons;
    vortons.append(freeVortons);
    vortons.append(newVortons);
    vortons.append(symFreeVortons);
    vortons.append(symNewVortons);
    int phantomSize=symFreeVortons.size()+symNewVortons.size();
    QVector<Parallel> paralVec;
    for (int i=0; i<vortons.size()-phantomSize; i++)
    {
        Parallel element {&vortons,nullptr, nullptr, i, streamVel, step};
        paralVec.push_back(element);
    }
    QVector<Vorton> resultedVec=QtConcurrent::blockingMappedReduced(paralVec, parallelDisplacement, addToVortonsVec, QtConcurrent::OrderedReduce);

    for (int i=0; i<resultedVec.size(); i++)
    {
        Vector3D selfLenBef=resultedVec[i].getTail()-resultedVec[i].getMid();
        Vector3D selfLenAft=resultedVec[i].getElongation()+selfLenBef;
        double turnAngle=acos(Vector3D::dotProduct(selfLenBef.normalized(), selfLenAft.normalized()));
        double lengthChange=fabs(selfLenBef.length()-selfLenAft.length());
        if (turnAngle>fiMax)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.turnRestr++;
        }
        if (lengthChange>eDelta)
        {
            resultedVec[i].setElongation(Vector3D());
            restrictions.elongationRestr++;
        }
        if (resultedVec[i].getMove().length()>maxMove)
        {
            resultedVec[i].setMove(Vector3D());
            restrictions.moveRestr++;
        }

    }

    for (int i=0; i<freeVortons.size(); i++)
    {
        freeVortons[i].setMove(resultedVec[i].getMove());
        freeVortons[i].setElongation(resultedVec[i].getElongation());
    }

    for (int i=0; i<newVortons.size(); i++)
    {
        newVortons[i].setMove(resultedVec[i+freeVortons.size()].getMove());
        newVortons[i].setElongation(resultedVec[i+freeVortons.size()].getElongation());
    }
    timers.integrationTimer=start.elapsed()*0.01;
}

/*!
Функция установки размера матрицы
\param size Размер матрциы
*/
void FrameCalculations::setMatrixSize(int size)
{
    matrixSize=size;
}

FramesSizes FrameCalculations::calcFrameSizes(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<double> frameSizes;
    for (int i=0; i<frames.size(); i++)
        frameSizes.push_back(frames[i]->length());
    FramesSizes sizes;
    sizes.maxFrameSize=*std::max_element(frameSizes.begin(),frameSizes.end());
    sizes.minFrameSize=*std::min_element(frameSizes.begin(),frameSizes.end());
    double aver=0.0;
    for (int i=0; i<frameSizes.size();i++)
        aver+=frameSizes[i]/frameSizes.size();
    sizes.averFrameSize=aver;
    return sizes;
}

/*!
Функция установки завихренностей рамкам
\param[in,out] frames Вектор, содержащий рамки
\param[in] vorticities Вектор, содержащий завихренности для рамок
*/
void FrameCalculations::setVorticity(QVector<std::shared_ptr<MultiFrame> > frames, const Eigen::VectorXd vorticities)
{
    for (int i=0; i<vorticities.size()-1; i++)
        frames[i]->setVorticity(vorticities(i));
}

/*!
Функция, возвращающая вектор вортонов с рамок
\param frames Вектор, содержащий рамки
\return Вектор, содержащий вортоны с рамок
*/
QVector<Vorton> FrameCalculations::getFrameVortons(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<Vorton> vortons;
    for (int i=0; i<frames.size(); i++)
        vortons.append(frames[i]->getVortons());
    return vortons;
}

/*!
Функция, возвращающая вектор поднятых вортонов с рамок по нормали
\param frames Вектор, содержащий рамки
\param normals Вектор, содержащий нормали
\param deltaUp Величина подъема вортонов по нормали
\return Вектор, содержащий вортоны с рамок
*/
QVector<Vorton> FrameCalculations::getLiftedFrameVortons(QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vector3D> &normals, const double deltaUp)
{
    QVector<Vorton> vortons;
    for (int i=0; i<frames.size(); i++)
        vortons.append(frames[i]->getLiftedVortons(deltaUp*normals[i]));
    return vortons;
}

/*!
Функция расчета давления в точке
\param point Точка для расчета давления в точке
\param streamVel Скорость потока
\param streamPres Давление потока
\param density Плотность среды
\param frames Вектор, содержащий рамки
\param freeVortons Вектор, содержащий вортоны в потоке
\param tau Величина шага
\return Значение давления
*/
double FrameCalculations::pressureCalc(const Vector3D point, const Vector3D streamVel, const double streamPres, const double density, QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau)
{
    double velAdd=0.5*(Vector3D::dotProduct(streamVel, streamVel)-pow((FrameCalculations::velocity(point, streamVel, freeVortons, frames).length()),2));
    double scal = 0.0;
    double framesAdd = 0.0;
    for (int i=0; i<freeVortons.size(); i++)
        scal+=Vector3D::dotProduct(freeVortons[i].getMove(), freeVortons[i].velocity(point));
    for (int i=0; i<frames.size(); i++)
    {
        framesAdd+=frames[i]->getVorticity()*frames[i]->fi(point);
    }
    return streamPres+density*(velAdd+scal/tau-framesAdd/tau);
}

/*!
Функция расчета силы, действующей на тело
\param streamVel Скорость потока
\param streamPres Давление потока
\param density Плотность среды
\param frames Вектор, содержащий рамки
\param freeVortons Вектор, содержащий вортоны в потоке
\param tau Величина шага
\param squares Вектор, содержащий площади рамок
\param controlPointsRaised Вектор, содержащий контрольные точки для вычисления давлений
\param normals Вектор, содержащий нормали
\return Значение силы
*/
Vector3D FrameCalculations::forceCalc(const Vector3D streamVel, double streamPres, double density, QVector<std::shared_ptr<MultiFrame> > frames, const QVector<Vorton> &freeVortons, const double tau, const QVector<double> &squares, const QVector<Vector3D> &controlPointsRaised, const QVector<Vector3D> &normals)
{
    QTime start = QTime::currentTime();
    Vector3D force;
    for (int i=0; i<controlPointsRaised.size(); i++)
    {
        double pressure=FrameCalculations::pressureCalc(controlPointsRaised[i], streamVel, streamPres, density, frames, freeVortons, tau);
        force-=pressure*squares[i]*normals[i];
    }
    timers.forceTimer=start.elapsed()*0.001;
    return force;
}

/*!
Функция суммирования соответствующих значений ср
\param[in] stepNum Номер текущего шага
\param[in,out] cp Вектор для записи значений ср
\param[in] fiFragNum Количество разбиений по фи
\param[in] radius Радиус сферы
\param[in] pointsRaising Величина подъема точек для вычисления давления
\param[in] tetas Вектор значений углов тета
\param[in] streamVel Скорость потока
\param[in] streamPres Давление потока
\param[in] density Плотность среды
\param[in] frames Вектор, содержащий рамки
\param[in] freeVortons Вектор, содержащий вортоны в потоке
\param[in] tau Величина шага
\param[in] center Координата центра сферы
*/
void FrameCalculations::cpSum(const int stepNum, const int stepsQuant, QVector<double> &cp, const int fiFragNum, const double radius, const double pointsRaising, const QVector<double> &tetas, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame>> frames, QVector<Vorton> freeVortons, double tau, const Vector3D center)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double fi=M_PI*5.0/fiFragNum;
        for (int i=0; i<tetas.size();i++)
        {
            Vector3D point((radius+pointsRaising)*sin(tetas[i])*cos(fi), (radius+pointsRaising)*sin(tetas[i])*sin(fi),(radius+pointsRaising)*cos(tetas[i]));
            point+=center;
            double pres=pressureCalc(point, streamVel,streamPres,density,frames,freeVortons,tau);
            cp[i]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        }
    }
}

void FrameCalculations::cpSumCylinder(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum, const double diameter, const double height, const double pointsRaising, const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        double fi0=M_PI*2.0/fiFragNum;
        for (int i=0; i<fiFragNum;i++)
        {
            double fi=fi0*(i+0.5);
            Vector3D point((diameter/2.0+pointsRaising)*cos(fi), height*0.5, (diameter/2.0+pointsRaising)*sin(fi));
            double pres=pressureCalc(/*controlpoints[i]*/point, streamVel,streamPres,density,frames,freeVortons,tau);
            cp[i]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
        }
    }
}

void FrameCalculations::cpSumRotationBody(const int stepNum, int stepsQuant, QVector<double> &cp, const int fiFragNum,  const Vector3D streamVel, const double streamPres, const double density, const QVector<std::shared_ptr<MultiFrame> > frames, QVector<Vorton> freeVortons, double tau, const QVector<Vector3D>& controlPointsRaised)
{
    int cpQuant;
    if (stepsQuant>200)
        cpQuant=200;
    else
        cpQuant=stepsQuant;
    if (stepNum>=stepsQuant-cpQuant)
    {
        int j=0;
        for (int i=0; i<controlPointsRaised.size()-2; i=i+fiFragNum)
        {
            double pres=pressureCalc(controlPointsRaised[i],streamVel,streamPres,density,frames,freeVortons,tau);
            cp[j]+=(pres-streamPres)/(density*Vector3D::dotProduct(streamVel,streamVel)*0.5);
            j++;
        }
    }
}

/*!
Функция расчета средних значений ср
\param[in,out] cp Вектор значений ср
\param[in] stepsNum Количество шагов расчета
*/
void FrameCalculations::cpAverage(QVector<double> &cp, const int stepsNum)
{
    int divider;
    if (stepsNum>=200)
        divider=200;
    else
        divider=stepsNum;
    for (int i=0; i<cp.size(); i++)
        cp[i]/=divider;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности сферы в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] center Координата центра сферы
\param[in] radius Величина радиуса сферы
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateSphere(QVector<Vorton> &vortons, const Vector3D center, const double radius, const double layerHeight, const QVector<Vector3D>& controlPoints, const QVector<Vector3D>& normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideSphere(vortons[i],center,radius))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
//            Vector3D axis=Vector3D(vortons[i].getTail()-center).normalized();
//            vortons[i].translate(2*axis*(axis*radius-vortons[i].getMid()).length());
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideSphereLayer(vortons[i],center,radius,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности цилиндра в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] height Высота цилиндра
\param[in] diameter Диаметр цилиндра
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateCylinder(QVector<Vorton> &vortons, const double height, const double diameter, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideCylinder(vortons[i],height,diameter))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideCylinderLayer(vortons[i],height,diameter,layerHeight))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности тела вращения в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] xBeg Минимальное значение координаты х тела
\param[in] xEnd Максимальное значение координаты х тела
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateRotationBody(QVector<Vorton>& vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    for (int i=0; i<vortons.size(); i++)
    {
        if (FrameCalculations::insideRotationBody(vortons[i],bodyNose,xEnd,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationBodyLayer(vortons[i],bodyNose,xEnd,layerHeight, forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности тела вращения со срезом в слое
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] xBeg Минимальное значение координаты х тела
\param[in] xEnd Максимальное значение координаты х тела
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateRotationCutBody(QVector<Vorton> &vortons,const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, const Vector3D bodyNose, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xEnd, bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xEnd,layerHeight,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

/*!
Функция, возвращающая вортоны из тела в поток и разворачивающая вортонов относительно поверхности тела вращения со срезом в слое при решении задачи старта
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] xBeg Минимальное значение координаты х тела
\param[in] xEnd Максимальное значение координаты х тела
\param[in] layerHeight Величина слоя
\param[in] controlPoints Вектор, содержащий контрольные точки
\param[in] normals Вектор, содержащий нормали
*/
void FrameCalculations::getBackAndRotateRotationCutLaunchedBody(QVector<Vorton> &vortons, const Vector3D bodyNose, const double xEnd, const double layerHeight, const QVector<Vector3D> &controlPoints, const QVector<Vector3D> &normals, FormingParameters forming)
{
    QTime start=QTime::currentTime();
    for (int i=vortons.size()-1; i>=0; i--)
    {
        if (FrameCalculations::insideRotationCutBody(vortons[i],xEnd,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].setMid(vortons[i].getMid()+2.0*closest.first*normals[closest.second]);
            vortons[i].setTail(vortons[i].getTail()+2.0*closest.first*normals[closest.second]);
            counters.gotBackNum++;
        }
        if(FrameCalculations::insideRotationCutBodyLayer(vortons[i],xEnd,layerHeight,bodyNose,forming))
        {
            QPair<double,int> closest=BodyFragmentation::findClosest(vortons[i].getMid(),controlPoints, normals);
            vortons[i].rotateAroundNormal(normals[closest.second]);
            counters.rotatedNum++;
        }
        if (FrameCalculations::insideScreen(vortons[i]))
        {
            vortons.remove(i);
            counters.underScreenNum++;
        }
    }
    timers.getBackAndRotateTimer=start.elapsed()*0.001;
}

/*!
Функция, добавляющая вортон в вектор. Создана для совместимости с QtConcurrent
\param[in,out] vortons Вектор, содержащий вортоны для функции
\param[in] vort Вортон для добавления
*/
void FrameCalculations::addToVortonsVec(QVector<Vorton> &vortons, const Vorton vort)
{
    vortons.push_back(vort);
}

/*!
Функция расчета скорости и тензора деформации от вектора вортонов в точке.
\param point Точка расчета
\param streamVel Скорость потока
\param vortons Вектор, содержащий вортоны
\return Значение скорости и тензора деформации
*/
VelBsym FrameCalculations::velocityAndBsymm(const Vector3D point,const  Vector3D streamVel, const QVector<Vorton> &vortons)
{
    VelBsym res = VelBsym (streamVel);
    for (int i=0; i<vortons.size(); i++)
        res+=vortons[i].velAndBsym(point);
    return res;
}

/*!
Функция расчета скорости от вектора вортонов и вектора рамок в точке.
\param point Точка расчета
\param streamVel Скорость потока
\param vortons Вектор, содержащий вортоны
\param frames Вектор, содержащий рамки
\return Значение скорости
*/
Vector3D FrameCalculations::velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons, const QVector<std::shared_ptr<MultiFrame>> frames)
{
    Vector3D vel=streamVel;
    for (int i=0; i<vortons.size(); i++)
        vel+=vortons[i].velocity(point);
    for (int i=0; i<frames.size(); i++)
        vel+=frames[i]->velocity(point);
    return vel;
}

/*!
Функция расчета скорости от вектора вортонов  в точке.
\overload
\param point Точка расчета
\param streamVel Скорость потока
\param vortons Вектор, содержащий вортоны
\return Значение скорости
*/
Vector3D FrameCalculations::velocity(const Vector3D point, const Vector3D streamVel, const QVector<Vorton> &vortons)
{
    Vector3D vel=streamVel;
    for (int i=0; i<vortons.size(); i++)
        vel+=vortons[i].velocity(point);
    return vel;
}

/*!
Функция расчета перемещения и удлинения вортона. Создана для совместимости с QtConcurrent
\param el Элемент структуры для расчета
\return Вортон с рассчитанным значением перемещения и удлинения
*/
Vorton FrameCalculations::parallelDisplacement(const Parallel el)
{
    Vorton res=el.Vortons->at(el.num);
    VelBsym vb=FrameCalculations::velocityAndBsymm(el.Vortons->at(el.num).getMid(), el.streamVel, *el.Vortons);
    Vector3D selfLen=el.Vortons->at(el.num).getTail()-el.Vortons->at(el.num).getMid();
    double xElong=vb.B[0][0]*selfLen[0]+vb.B[0][1]*selfLen[1]+vb.B[0][2]*selfLen[2];
    double yElong=vb.B[1][0]*selfLen[0]+vb.B[1][1]*selfLen[1]+vb.B[1][2]*selfLen[2];
    double zElong=vb.B[2][0]*selfLen[0]+vb.B[2][1]*selfLen[1]+vb.B[2][2]*selfLen[2];
    res.setElongation(Vector3D(xElong, yElong, zElong)*el.tau);
    res.setMove(vb.Vel*el.tau);
    return res;
}

/*!
Функция для изменения положения вортона согласно рассчитанному перемещению и изменению.
\param[in,out] vortons Вектор вортонов
*/
void FrameCalculations::displace(QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size(); i++)
    {
        vortons[i].setTail(vortons[i].getTail()+vortons[i].getMove()+vortons[i].getElongation());
        vortons[i].setMid(vortons[i].getMid()+vortons[i].getMove());
    }
}

/*!
Функция для отражения положения вортонов и рамок относительно оси х
\param[in,out] symFreeVortons Вектор вортонов в слое
\param[in,out] symNewVortons Вектор вортонов с рамок
\param[in,out] symFrames Вектор рамок
*/
void FrameCalculations::reflect(QVector<Vorton> &symFreeVortons, QVector<Vorton> &symNewVortons, QVector<std::shared_ptr<MultiFrame>> symFrames)
{
    for (int i=0; i<symFreeVortons.size(); i++)
    {
        symFreeVortons[i].setMid(Vector3D(-symFreeVortons[i].getMid().x(),symFreeVortons[i].getMid().y(),symFreeVortons[i].getMid().z()));
        symFreeVortons[i].setTail(Vector3D(-symFreeVortons[i].getTail().x(),symFreeVortons[i].getTail().y(),symFreeVortons[i].getTail().z()));
        symFreeVortons[i].setVorticity(-symFreeVortons[i].getVorticity());
    }

    for (int i=0; i<symNewVortons.size(); i++)
    {
        symNewVortons[i].setMid(Vector3D(-symNewVortons[i].getMid().x(),symNewVortons[i].getMid().y(),symNewVortons[i].getMid().z()));
        symNewVortons[i].setTail(Vector3D(-symNewVortons[i].getTail().x(),symNewVortons[i].getTail().y(),symNewVortons[i].getTail().z()));
        symNewVortons[i].setVorticity(-symNewVortons[i].getVorticity());

    }

    for (int i=0; i<symFrames.size(); i++)
    {
        for (int j=0; j<symFrames[i]->getAnglesNum(); j++)
        {
            symFrames[i]->at(j).setMid(Vector3D(-symFrames[i]->at(j).getMid().x(),symFrames[i]->at(j).getMid().y(),symFrames[i]->at(j).getMid().z()));
            symFrames[i]->at(j).setTail(Vector3D(-symFrames[i]->at(j).getTail().x(),symFrames[i]->at(j).getTail().y(),symFrames[i]->at(j).getTail().z()));
            symFrames[i]->at(j).setVorticity(-symFrames[i]->at(j).getVorticity());
        }
        symFrames[i]->setCenter(Vector3D(-symFrames[i]->getCenter().x(), symFrames[i]->getCenter().y(), symFrames[i]->getCenter().z()));
    }
}

void FrameCalculations::reflectMove(QVector<Vorton> &symFreeVortons, QVector<Vorton> &freeVortons)
{
    for (int i=0; i<symFreeVortons.size(); i++)
    {
        symFreeVortons[i].setMove(Vector3D(-freeVortons[i].getMove().x(),freeVortons[i].getMove().y(),freeVortons[i].getMove().z()));
    }
}

/*!
Функция для проверки попадания вортона внутрь сферы
\param vorton Вортон
\param center Центра сферы
\param radius Радиус сферы
\return Определение попадания внутрь сферы
*/
bool FrameCalculations::insideSphere(const Vorton& vort, const Vector3D& center, const double radius)
{
    if ((vort.getMid()-center).length()<radius)
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг сферы
\param vorton Вортон
\param center Центр сферы
\param radius Радиус сферы
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг сферы
*/
bool FrameCalculations::insideSphereLayer(const Vorton& vort, const Vector3D& center, const double radius, const double layerHeight)
{
    if ((vort.getMid()-center).length()<radius+layerHeight)
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь цилиндра
\param vorton Вортон
\param height Высота цилиндра
\param diameter Диаметр цилиндра
\return Определение попадания внутрь цилиндра
*/
bool FrameCalculations::insideCylinder(const Vorton &vort, const double height, const double diameter)
{
    if ((vort.getMid().y()>0)&&(vort.getMid().y()<height)&&((pow(vort.getMid().x(),2)+pow(vort.getMid().z(),2))<0.25*pow(diameter,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг цилиндра
\param vorton Вортон
\param height Высота цилиндра
\param diameter Диаметр цилиндра
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг цилиндра
*/
bool FrameCalculations::insideCylinderLayer(const Vorton &vort, const double height, const double diameter, const double layerHeight)
{
    if ((vort.getMid().y()>-layerHeight)&&(vort.getMid().y()<height+layerHeight)&&((pow(vort.getMid().x(),2)+pow(vort.getMid().z(),2))<pow(0.5*diameter+layerHeight,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь тела вращения
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\return Определение попадания внутрь тела вращения
*/
bool FrameCalculations::insideRotationBody(const Vorton &vort, const Vector3D bodyNose, const double xEnd, FormingParameters forming)
{
    if((vort.getMid().x()>=bodyNose.x()) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getMid().x()-bodyNose.x(),forming),2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг тела вращения
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг тела вращения
*/
bool FrameCalculations::insideRotationBodyLayer(const Vorton &vort, const Vector3D bodyNose, const double xEnd, const double layerHeight, FormingParameters forming)
{
    if((vort.getMid().x()>=bodyNose.x()-layerHeight) && (vort.getMid().x()<=xEnd+layerHeight) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionF(vort.getMid().x()-bodyNose.x(),forming)+layerHeight,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь тела вращения со срезом дна
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\return Определение попадания внутрь тела вращения со срезом дна
*/
bool FrameCalculations::insideRotationCutBody(const Vorton &vort, const double xBeg, const double xEnd, FormingParameters forming)
{
    if((vort.getMid().x()>=xBeg) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z(),2)+pow(vort.getMid().y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()-xBeg,forming),2)))
        return true;
    return false;
}

bool FrameCalculations::insideRotationCutBody(const Vorton &vort,const double xEnd, const Vector3D bodyNose, FormingParameters forming)
{
    if((vort.getMid().x()>=bodyNose.x()) && (vort.getMid().x()<=xEnd) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()-bodyNose.x(),forming),2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона внутрь слоя вокруг тела вращения со срезом дна
\param vorton Вортон
\param xBeg Минимальная координата х тела
\param xEnd Максимальная координата х тела
\param layerHeight Высота слоя
\return Определение попадания внутрь слоя вокруг тела вращения со срезом дна
*/
bool FrameCalculations::insideRotationCutBodyLayer(const Vorton &vort, const double xEnd, const double layerHeight, const Vector3D bodyNose, FormingParameters forming)
{
    if((vort.getMid().x()>=bodyNose.x()-layerHeight) && (vort.getMid().x()<=xEnd+layerHeight) && ((pow(vort.getMid().z()-bodyNose.z(),2)+pow(vort.getMid().y()-bodyNose.y(),2))<pow(BodyFragmentation::presetFunctionG(vort.getMid().x()-bodyNose.x(),forming)+layerHeight,2)))
        return true;
    return false;
}

/*!
Функция для проверки попадания вортона под экран
\param vorton Вортон
\return Определение вортона под экран
*/
bool FrameCalculations::insideScreen(Vorton &vort)
{
    if (vort.getMid().x()>0.0 || vort.getTail().x()>0.0)
        return true;
    return false;
}

/*!
Функция для проверки возможного "взрыва" вортонов
\param vortons Вектор вортонов
\return Определение вортона под экран
*/
bool FrameCalculations::exploseSphere(const QVector<Vorton> &vortons)
{
    int counter=0;
    for (int i=0; i<vortons.size(); i++)
    {
       if (fabs(vortons[i].getMid().y())>2.0||fabs(vortons[i].getMid().x())>2.0||vortons[i].getMid().z()<-1.0)
           counter++;
       if (fabs(vortons[i].getTail().y())>2.0||fabs(vortons[i].getTail().x())>2.0||vortons[i].getTail().z()<-1.0)
           counter++;
    }
    if (counter>=10)
        return true;
    return false;
}

/*!
Функция для расчета дисперсии вектора С
\param cAerodynamics Вектор С
\return Значение дисперсии
*/
double FrameCalculations::calcDispersion(const QVector<Vector3D> &cAerodynamics)
{
    Vector3D cAver;
    for (int i=100; i<cAerodynamics.size(); i++)
    {
        cAver+=cAerodynamics[i];
    }
    cAver=cAver/(cAerodynamics.size()-100);

    double dispersion=0.0;

    for (int i=100; i<cAerodynamics.size(); i++)
    {
        dispersion+=(cAerodynamics[i]-cAver).lengthSquared()/(cAerodynamics.size()-100);
    }
    dispersion=sqrt(dispersion)/cAver.length();
    return dispersion;
}

/*!
Функция для копирования вектора рамок
\param frames Вектор рамок
\return Скопированный вектор рамок
*/
QVector<std::shared_ptr<MultiFrame> > FrameCalculations::copyFrames(QVector<std::shared_ptr<MultiFrame> > frames)
{
    QVector<std::shared_ptr<MultiFrame>> copyingFrames;
    for (int i=0; i<frames.size(); i++)
    {
        copyingFrames.push_back(std::make_shared<MultiFrame>(*frames[i].get()));
    }
    return copyingFrames;
}

/*!
Функция для перемещения тела при решении задачи старта
\param[in] translation Вектор переноса
\param[in,out] frames Вектор рамок
\param[in,out] controlPoints Вектор контрольных точек
\param[in,out] controlPointsRaised Вектор контрольных точек для расчета давлений
\param[in,out] center Центр тела
\param[in] xBeg Минимальная координата х тела
\param[in] xEnd Максимальная координата х тела
\param[in] fragPar Параметры разбиения
*/
void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>> &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised, Vector3D &center, Vector3D &bodyNose, double &xend, const FragmentationParameters &fragPar)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }

   //if (xbeg!=fragPar.rotationBodyXBeg)
   bodyNose+=translation;
    center+=translation;
//   if (xend-bodyNose.x()>fragPar.rotationBodyXEnd-fragPar.rotationBodyXBeg)
        xend+=translation.x();
//   if (xend!=0.0)
//       qDebug()<<"Stop";
}

/*!
Функция для перемещения тела.
\overload
\param[in] translation Вектор переноса
\param[in,out] frames Вектор рамок
\param[in,out] controlPoints Вектор контрольных точек
\param[in,out] controlPointsRaised Вектор контрольных точек для расчета давлений
\param[in,out] center Центр тела
*/
void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame>> &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised, Vector3D &center)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }
    center+=translation;
}

void FrameCalculations::translateBody(const Vector3D &translation, QVector<std::shared_ptr<MultiFrame> > &frames, QVector<Vector3D> &controlPoints, QVector<Vector3D> &controlPointsRaised)
{
    for (int i=0; i<frames.size(); i++)
        frames[i]->translate(translation);
    for (int i=0; i<controlPoints.size(); i++)
    {
        controlPoints[i].translate(translation);
        controlPointsRaised[i].translate(translation);
    }
}

void FrameCalculations::updateBoundaries(Vector3D &bodynose, Vector3D &translation, Vector3D &center)
{
    bodynose+=translation;
    center=bodynose/2;
}

/*!
Функция для перемещения вектора вортонов
\param[in] translation Вектор переноса
\param[in,out] vortons Вектор вортонов
*/
void FrameCalculations::translateVortons(const Vector3D &translation, QVector<Vorton> &vortons)
{
    for (int i=0; i<vortons.size(); i++)
        vortons[i].translate(translation);
}

/*!
Функция возвращающая значения счетчиков
\return Структура с текущими значениями счетчиков
*/
Counters FrameCalculations::getCounters() const
{
    return counters;
}

/*!
Функция возвращающая значения сработавших ограничений
\return Структура с текущими значениями сработавших ограничений
*/
Restrictions FrameCalculations::getRestrictions() const
{
    return restrictions;
}

/*!
Функция возвращающая значения таймеров
\return Структура с текущими значениями таймеров
*/
Timers FrameCalculations::getTimers() const
{
    return timers;
}

/*!
Функция обнуляющая значения счетчиков
*/
void FrameCalculations::clearCounters()
{
    counters.clear();
}

/*!
Функция обнуляющая значения сработавших ограничений
*/
void FrameCalculations::clearRestrictions()
{
    restrictions.clear();
}

/*!
Функция обнуляющая значения сработавших таймеров
*/
void FrameCalculations::clearTimers()
{
    timers.clear();
}

/*!
Функция обнуляющая значения сработавших счетчиков, сработавших ограничений, таймеров
*/
void FrameCalculations::clear()
{
    clearTimers();
    clearRestrictions();
    clearCounters();
}

