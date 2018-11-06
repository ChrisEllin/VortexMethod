#include "preprocessorsettings.h"
#include "ui_preprocessorsettings.h"

PreprocessorSettings::PreprocessorSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreprocessorSettings)
{
    ui->setupUi(this);
    setWindowTitle("Настройки препроцессора");
}

Boundaries PreprocessorSettings::getBoundaries()
{
    Vector3D minBoundary;
    Vector3D maxBoundary;
    minBoundary.x(ui->xMinBoundaryLineEdit->text().toDouble());
    minBoundary.y(ui->yMinBoundaryLineEdit->text().toDouble());
    minBoundary.z(ui->zMinBoundaryLineEdit->text().toDouble());
    maxBoundary.x(ui->xMaxBoundaryLineEdit->text().toDouble());
    maxBoundary.y(ui->yMaxBoundaryLineEdit->text().toDouble());
    maxBoundary.z(ui->zMaxBoundaryLineEdit->text().toDouble());
    boundaries.maxBoundary=maxBoundary;
    boundaries.minBoundary=minBoundary;
    return boundaries;
}

PreprocessorSettings::~PreprocessorSettings()
{
    delete ui;
}
