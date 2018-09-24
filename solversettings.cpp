#include "solversettings.h"
#include "ui_solversettings.h"

void SolverParameters::setData(const int i, const double value)
{
    switch (i)
    {
    case 0:
        streamPres=value;
        break;
    case 1:
        density=value;
        break;
    case 3:
        eStar=value;
        break;
    case 4:
        eDoubleStar=value;
        break;
    case 5:
        fiMax=value;
        break;
    case 6:
        eDelta=value;
        break;
    case 7:
        minVorticity=value;
        break;
    case 8:
        layerHeight=value;
        break;
    case 9:
        tau=value;
        break;
    case 10:
        stepsNum=value;
        break;
    case 11:
        deltaUp=value;
        break;
    case 12:
        farDistance=value;
        break;
    case 13:
        maxMove=value;
        break;
    }
}

void SolverParameters::setData(const Vector3D value)
{
    streamVel=value;
}

void FreeMotionParameters::setData(const Vector3D value)
{
    bodyVel=value;
}

SolverSettings::SolverSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SolverSettings)
{
    ui->setupUi(this);
    setWindowTitle(tr("Настройки расчета"));
    standartPar.maxMove=ui->maxMoveLineEdit->text().toDouble();
    standartPar.minVorticity=ui->gammaMinLineEdit->text().toDouble();
    standartPar.density=ui->densityLineEdit->text().toDouble();
    standartPar.eDelta=ui->elongationMaxLineEdit->text().toDouble();
    standartPar.eStar=ui->eStarLineEdit->text().toDouble();
    standartPar.eDoubleStar=ui->eDoubleStarLineEdit->text().toDouble();
    standartPar.deltaUp=ui->deltaUpLineEdit->text().toDouble();
    standartPar.farDistance=ui->farDistanceLineEdit->text().toDouble();
    standartPar.layerHeight=ui->layerHeightLineEdit->text().toDouble();
    standartPar.tau=ui->stepLineEdit->text().toDouble();
    standartPar.fiMax=ui->fiMaxLineEdit->text().toDouble();
    standartPar.stepsNum=ui->stepsNumLineEdit->text().toDouble();
    standartPar.streamPres=ui->pressureStreamLineEdit->text().toDouble();
    standartPar.streamVel=Vector3D(ui->xVelStreamLineEdit->text().toDouble(),ui->yVelStreamLineEdit->text().toDouble(),ui->zVelStreamLineEdit->text().toDouble());

    standartFreeMotionPar.bodyVel=Vector3D(ui->xVelBodyLineEdit->text().toDouble(),ui->yVelBodyLineEdit->text().toDouble(),ui->zVelBodyLineEdit->text().toDouble());
    solvPar=standartPar;
    freeMotionPar=standartFreeMotionPar;
}

SolverParameters SolverSettings::getSolverParameters()
{
    return solvPar;
}

FreeMotionParameters SolverSettings::getFreeMotionParameters()
{
    return freeMotionPar;
}

SolverSettings::~SolverSettings()
{
    delete ui;
}

void SolverSettings::setSolverParameters(SolverParameters &newSolvPar)
{
    ui->maxMoveLineEdit->setText(QString::number(newSolvPar.maxMove));
    ui->gammaMinLineEdit->setText(QString::number(newSolvPar.minVorticity));
    ui->densityLineEdit->setText(QString::number(newSolvPar.density));
    ui->elongationMaxLineEdit->setText(QString::number(newSolvPar.eDelta));
    ui->eStarLineEdit->setText(QString::number(newSolvPar.eStar));
    ui->eDoubleStarLineEdit->setText(QString::number(newSolvPar.eDoubleStar));
    ui->deltaUpLineEdit->setText(QString::number(newSolvPar.deltaUp));
    ui->farDistanceLineEdit->setText(QString::number(newSolvPar.farDistance));
    ui->layerHeightLineEdit->setText(QString::number(newSolvPar.layerHeight));
    ui->stepLineEdit->setText(QString::number(newSolvPar.tau));
    ui->fiMaxLineEdit->setText(QString::number(newSolvPar.fiMax));
    ui->stepsNumLineEdit->setText(QString::number(newSolvPar.stepsNum));
    ui->pressureStreamLineEdit->setText(QString::number(newSolvPar.streamPres));
    ui->xVelStreamLineEdit->setText(QString::number(newSolvPar.streamVel.x()));
    ui->yVelStreamLineEdit->setText(QString::number(newSolvPar.streamVel.y()));
    ui->zVelStreamLineEdit->setText(QString::number(newSolvPar.streamVel.z()));

    solvPar=newSolvPar;
}

void SolverSettings::on_saveSolverSettingsPushButton_clicked()
{
    if (ui->stepLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен шаг интегрирования"));
        return;
    }
    if (ui->stepsNumLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество шагов интегрирования"));
        return;
    }
    if (ui->eStarLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено максимальное расстояние для объединения вортонов"));
        return;
    }
    if (ui->eDoubleStarLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен максимальный косинус угла для объединения "));
        return;
    }

    if (ui->deltaUpLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков от рамок"));
        return;
    }
    if (ui->elongationMaxLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено максимальное значение для удлинения отрезков"));
        return;
    }
    if (ui->fiMaxLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено максимальное значение угла для расчета перемещений"));
        return;
    }

    if (ui->layerHeightLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение толщины слоя"));
        return;
    }

    if(ui->maxMoveLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено максимальное перемещение"));
        return;
    }
    if (ui->gammaMinLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено минимальное значение завихренности"));
        return;
    }
    if (ui->pressureStreamLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение давления потока"));
        return;
    }
    if (ui->densityLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение плотности"));
        return;
    }
    if (ui->xVelStreamLineEdit->text().isEmpty()||ui->yVelStreamLineEdit->text().isEmpty()||ui->zVelStreamLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение скорости потока"));
        return;
    }
    if (ui->farDistanceLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена максимальная дальность"));
        return;
    }

    solvPar.maxMove=ui->maxMoveLineEdit->text().toDouble();
    solvPar.minVorticity=ui->gammaMinLineEdit->text().toDouble();
    solvPar.density=ui->densityLineEdit->text().toDouble();
    solvPar.eDelta=ui->elongationMaxLineEdit->text().toDouble();
    solvPar.eStar=ui->eStarLineEdit->text().toDouble();
    solvPar.eDoubleStar=ui->eDoubleStarLineEdit->text().toDouble();
    solvPar.deltaUp=ui->deltaUpLineEdit->text().toDouble();
    solvPar.farDistance=ui->farDistanceLineEdit->text().toDouble();
    solvPar.layerHeight=ui->layerHeightLineEdit->text().toDouble();
    solvPar.tau=ui->stepLineEdit->text().toDouble();
    solvPar.fiMax=ui->fiMaxLineEdit->text().toDouble();
    solvPar.stepsNum=ui->stepsNumLineEdit->text().toDouble();
    solvPar.streamPres=ui->pressureStreamLineEdit->text().toDouble();
    solvPar.streamVel=Vector3D(ui->xVelStreamLineEdit->text().toDouble(),ui->yVelStreamLineEdit->text().toDouble(),ui->zVelStreamLineEdit->text().toDouble());
    hide();
}

void SolverSettings::on_resetSettingsPushButton_clicked()
{
    solvPar=standartPar;
    ui->maxMoveLineEdit->setText(QString::number(solvPar.maxMove));
    ui->gammaMinLineEdit->setText(QString::number(solvPar.minVorticity));
    ui->densityLineEdit->setText(QString::number(solvPar.density));
    ui->elongationMaxLineEdit->setText(QString::number(solvPar.eDelta));
    ui->eStarLineEdit->setText(QString::number(solvPar.eStar));
    ui->eDoubleStarLineEdit->setText(QString::number(solvPar.eDoubleStar));
    ui->deltaUpLineEdit->setText(QString::number(solvPar.deltaUp));
    ui->farDistanceLineEdit->setText(QString::number(solvPar.farDistance));
    ui->layerHeightLineEdit->setText(QString::number(solvPar.layerHeight));
    ui->stepLineEdit->setText(QString::number(solvPar.tau));
    ui->fiMaxLineEdit->setText(QString::number(solvPar.fiMax));
    ui->stepsNumLineEdit->setText(QString::number(solvPar.stepsNum));
    ui->pressureStreamLineEdit->setText(QString::number(solvPar.streamPres));
    ui->xVelStreamLineEdit->setText(QString::number(solvPar.streamVel.x()));
    ui->yVelStreamLineEdit->setText(QString::number(solvPar.streamVel.y()));
    ui->zVelStreamLineEdit->setText(QString::number(solvPar.streamVel.z()));
}

void SolverSettings::on_saveFreeMotionSettingsPushButton_clicked()
{
    if (ui->xVelBodyLineEdit->text().isEmpty()||ui->yVelBodyLineEdit->text().isEmpty()||ui->zVelBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение скорости тела"));
        return;
    }

    freeMotionPar.bodyVel=Vector3D(ui->xVelBodyLineEdit->text().toDouble(),ui->yVelBodyLineEdit->text().toDouble(),ui->zVelBodyLineEdit->text().toDouble());
    hide();
}

void SolverSettings::on_resetFreeMotionSettingsPushButton_clicked()
{
    freeMotionPar=standartFreeMotionPar;
    ui->xVelBodyLineEdit->setText(QString::number(freeMotionPar.bodyVel.x()));
    ui->yVelBodyLineEdit->setText(QString::number(freeMotionPar.bodyVel.y()));
    ui->zVelBodyLineEdit->setText(QString::number(freeMotionPar.bodyVel.z()));
}
