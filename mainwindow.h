#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "mainfield.h"
#include "solver.h"
#include <QFileDialog>
#include <QShortcut>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    SolverSettings *settings;
    Solver *solver;
    bool displaySphere;
    QShortcut *keyCtrlO;
    QShortcut *keyCtrlR;
    QShortcut *keyCtrlH;

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
    void openPassport();
    void on_cylinderSolverPushButton_clicked();

    void on_variateSphereSolverPushButton_clicked();

    void on_rotationBodySolverPushButton_clicked();

public slots:
    void recieveProgressSphere(const int percentage);
    void recieveProgressCylinder(const int percentage);
    void recieveProgressRotationBody(const int percentage);
    void drawGUI(const QVector<Vorton>& vortons, const QVector<std::shared_ptr<MultiFrame>>& frames);
signals:
    void setPlaneXY();
    void setPlaneYX();
    void setPlaneXZ();
    void setPlaneZX();
    void setPlaneYZ();
    void setPlaneZY();
    void resetPlane();
    void sendSolverParameters(SolverParameters& solvPar);
    void drawSegment(QVector3D center, QVector3D top, SArrow::vort_type);
    void drawSegment(QVector3D center, QVector3D top);
    void clearSegments(int n = -1);
    void setRadius(double radius);
    void setHeight(double height);
    void changeShape(MainField::shape newShape);
};

#endif // MAINWINDOW_H
