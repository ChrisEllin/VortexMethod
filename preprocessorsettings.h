#ifndef PREPROCESSORSETTINGS_H
#define PREPROCESSORSETTINGS_H
#include "vector3d.h"
#include <QDialog>

struct Boundaries
{
    Vector3D minBoundary;
    Vector3D maxBoundary;
};

namespace Ui {
class PreprocessorSettings;
}

class PreprocessorSettings : public QDialog
{
    Q_OBJECT

public:
    explicit PreprocessorSettings(QWidget *parent = nullptr);
    Boundaries getBoundaries();
    ~PreprocessorSettings();

private:
    Ui::PreprocessorSettings *ui;
    Boundaries boundaries;
};

#endif // PREPROCESSORSETTINGS_H
