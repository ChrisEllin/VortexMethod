#ifndef SOLVERSETTINGS_H
#define SOLVERSETTINGS_H

#include <QDialog>
#include "vector3d.h"

/*!
    \brief Структура, хранящая параметры расчета 
*/
struct SolverParameters
{
    double streamPres; ///<Давление потока
    double density; ///< Плотность
    Vector3D streamVel; ///< Скорость потока
    double eStar; ///< Максимальное расстояние между центрами вортон-отрезков для их объединения
    double eDoubleStar; ///< Минимальный косинус угла между вортон-отрезками для их объединения
    double fiMax; ///< Максимальный угол поворота вортон-отрезка за шаг расчета
    double eDelta; ///< Максимальное удлинение вортон-отрезка за шаг расчета
    double minVorticity; ///< Минимальное значение завихренности для учета вортон-отрезка
    double layerHeight; ///< Высота слоя над телом
    double tau; ///< Шаг расчета
    int stepsNum; ///< Количество шагов расчета
    double deltaUp; ///< Высота подъема вортоно-отрезков с тела
    double farDistance; ///< Максимальная удаленность от тела для учитывания влияния вортон-отрезков
    double maxMove; ///< Максимальное значение перемещения вортон-отрезка за один шаг расчета
    int acceleratedStepsNum;
    void setData(const int i, const double value);
    void setData(const Vector3D value); ///<
};

/*!
    \brief Структура, хранящая параметры свободного движения
*/
struct FreeMotionParameters
{
    Vector3D bodyVel; ///<Скорость свободного движения тела
    void setData(const Vector3D value);
};

namespace Ui {
class SolverSettings;
}

/*!
    \brief Класс, отвечающий за интерфейс формы настроек расчета и свободного движения
*/
class SolverSettings : public QDialog
{
    Q_OBJECT

public:
    explicit SolverSettings(QWidget *parent = 0);
    SolverParameters getSolverParameters();
    FreeMotionParameters getFreeMotionParameters();
    ~SolverSettings();

public slots:
    void setSolverParameters(SolverParameters& newSolvPar);
    void calcAttributes(const QString epsilon);
    void calcAttributes(double panelLength);
private slots:
    void on_saveSolverSettingsPushButton_clicked();
    void on_resetSettingsPushButton_clicked();
    void on_saveFreeMotionSettingsPushButton_clicked();
    void on_resetFreeMotionSettingsPushButton_clicked();

    void on_xVelBodyLineEdit_editingFinished();

private:
    Ui::SolverSettings *ui; ///< Указатель на объект формы
    //bool entirety;
    SolverParameters standartPar; ///< Стандартные настройки параметров расчета
    SolverParameters solvPar; ///< Текущие настройки параметров расчета
    FreeMotionParameters freeMotionPar; ///< Стандартные настройки параметров свободного движения
    FreeMotionParameters standartFreeMotionPar; ///< Текущие настройки параметров свободного движения
};

#endif // SOLVERSETTINGS_H
