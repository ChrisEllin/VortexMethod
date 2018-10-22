#ifndef VARIATESETTINGS_H
#define VARIATESETTINGS_H

#include <QWidget>

namespace Ui {
class VariateSettings;
}

class VariateSettings : public QWidget
{
    Q_OBJECT

public:
    explicit VariateSettings(QWidget *parent = 0);
    ~VariateSettings();
    bool getInfo();
    double getScreenDistance();

private:
    Ui::VariateSettings *ui;
};

#endif // VARIATESETTINGS_H
