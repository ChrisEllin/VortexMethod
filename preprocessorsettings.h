#ifndef PREPROCESSORSETTINGS_H
#define PREPROCESSORSETTINGS_H
#include "vector3d.h"
#include <QDialog>

namespace Ui {
class PreprocessorSettings;
}

class PreprocessorSettings : public QDialog
{
    Q_OBJECT

public:
    explicit PreprocessorSettings(QWidget *parent = nullptr);
    Vector3D getBoundaries();
    ~PreprocessorSettings();

private:
    Ui::PreprocessorSettings *ui;
    Vector3D boundary;
};

#endif // PREPROCESSORSETTINGS_H
