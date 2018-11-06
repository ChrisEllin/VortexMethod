#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "mainfield.h"
#include "solver.h"
#include <QFileDialog>
#include <QShortcut>
#include "variatesettings.h"
#include "preprocessorsettings.h"

namespace Ui {
class MainWindow;
}
/*!
    \brief Класс, отвечающий за интерфейс основного окна
*/
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui; ///<Указатель на интерфейс основного окна
    SolverSettings *settings; ///<Указатель на класс окна настроек расчета
    Solver *solver; ///<Указатель на класс-расчетчик
    VariateSettings *variateSettings; ///<Указатель на класс окна настроек вариации
    PreprocessorSettings *preprocessor;
    bool displaySphere; ///<Необходимость отображения трехмерной сферы
    QShortcut *keyCtrlO; ///<Указатель на хоткей (Ctrl+O)
    QShortcut *keyCtrlR; ///<Указатель на хоткей (Ctrl+R)
    QShortcut *keyCtrlH; ///<Указатель на хоткей (Ctrl+H)
    QVector<Vorton> currentVortons;
    QVector<std::shared_ptr<MultiFrame>> currentFrames;
    bool checkDrawing(const Vector3D &mid, const Vector3D &tail);
    void setParameters(SphereParameters &sphPar);
    void setParameters(CylinderParameters &cylPar);
    void setParameters(RotationBodyParameters &rotBodyPar);
private slots:

    void wheelEvent (QWheelEvent* e);
    void on_toolButtonViewXY_clicked();
    void on_toolButtonViewYX_clicked();
    void on_toolButtonViewXZ_clicked();
    void on_toolButtonViewZX_clicked();
    void on_toolButtonViewYZ_clicked();
    void on_toolButtonViewZY_clicked();
    void on_toolButtonViewReset_clicked();
    void keyCtrlRActiavated();
    void on_sphereSolverPushButton_clicked();
    void showSettings();
    void showSphere();
    void showPreprocessor();
    void showVariateSettings();
    void openPassport();
    void on_cylinderSolverPushButton_clicked();
    void on_variateSphereSolverPushButton_clicked();    
    void on_rotationBodySolverPushButton_clicked();
    void on_rotationCutBodySolverPushButton_clicked();
    void on_rotationCutBodyFreeMotionSolverPushButton_clicked();
    void on_rotationCutBodyLaunchSolverPushButton_clicked();
    void on_sphereFreeMotionSolverPushButton_clicked();
    void updateScreen();
    void on_variateCylinderSolverPushButton_clicked();

    void on_variateRotationBodySolverPushButton_clicked();

    void on_variateRotationCutBodySolverPushButton_clicked();

    void on_rotationCutBodyNearScreenPushButton_clicked();

    void on_rotationBodyFreeMotionSolverPushButton_clicked();

public slots:
    void showInfo();
    void setMaxGamma(double maxGamma);
    void recieveProgressSphere(const int percentage);
    void recieveProgressCylinder(const int percentage);
    void recieveProgressRotationBody(const int percentage);
    void recieveProgressRotationCutBody(const int percentage);
    void drawGUI(const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>>& frames);
signals:
    void setPlaneXY(); ///<Сигнал о переходе к YX отображению
    void setPlaneYX(); ///<Сигнал о переходе к XY отображению
    void setPlaneXZ(); ///<Сигнал о переходе к XZ отображению
    void setPlaneZX(); ///<Сигнал о переходе к ZX отображению
    void setPlaneYZ(); ///<Сигнал о переходе к YZ отображению
    void setPlaneZY(); ///<Сигнал о переходе к ZY отображению
    void resetPlane(); ///<Сигнал о переходе к начальному отображению
    void sendSolverParameters(SolverParameters& solvPar); ///<Сигнал, отправляющий параметры расчета
    void drawSegment(QVector3D center, QVector3D top, SArrow::vort_type); ///<Сигнал об отрисовке с выбором типа отрисовки
    void drawSegment(QVector3D center, QVector3D top); ///< \overload Сигнал об отрисовке точки
    void clearSegments(int n = -1); ///<Сигнал об очистке экрана
    void setRadius(double radius); ///<Сигнал об установке радиуса трехмерной фигуры
    void setHeight(double height); ///<Сигнал об установке высоты цилиндра
    void changeShape(MainField::shape newShape); ///<Сигнал об смене типа отображаемой фигуры
};

#endif // MAINWINDOW_H
