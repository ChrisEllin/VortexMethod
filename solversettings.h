#ifndef SOLVERSETTINGS_H
#define SOLVERSETTINGS_H

#include <QDialog>
#include "vector3d.h"

struct SolverParameters
{
    double streamPres;
    double density;
    Vector3D streamVel;
    double eStar;
    double eDoubleStar;
    double fiMax;
    double eDelta;
    double minVorticity;
    double layerHeight;    
    double tau;
    int stepsNum;
    double deltaUp;
    double farDistance;
    double maxMove;
    void setData(const int i, const double value);
    void setData(const Vector3D value);
};

namespace Ui {
class SolverSettings;
}

class SolverSettings : public QDialog
{
    Q_OBJECT

public:
    explicit SolverSettings(QWidget *parent = 0);
    SolverParameters getSolverParameters();
    ~SolverSettings();
    Ui::SolverSettings getSettingsUI() const;
public slots:
    void setSolverParameters(SolverParameters& newSolvPar);
private slots:
    void on_saveSolverSettingsPushButton_clicked();

    void on_resetSettingsPushButton_clicked();

private:
    Ui::SolverSettings *ui;
    bool entirety;
    SolverParameters standartPar;
    SolverParameters solvPar;
};

#endif // SOLVERSETTINGS_H
