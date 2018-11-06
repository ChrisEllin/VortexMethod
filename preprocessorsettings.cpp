#include "preprocessorsettings.h"
#include "ui_preprocessorsettings.h"

PreprocessorSettings::PreprocessorSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreprocessorSettings)
{
    ui->setupUi(this);
    setWindowTitle("Настройки препроцессора");
}

Vector3D PreprocessorSettings::getBoundaries()
{
    boundary.x(ui->xBoundaryLineEdit->text().toDouble());
    boundary.y(ui->yBoundaryLineEdit->text().toDouble());
    boundary.z(ui->zBoundaryLineEdit->text().toDouble());
    return boundary;
}

PreprocessorSettings::~PreprocessorSettings()
{
    delete ui;
}
