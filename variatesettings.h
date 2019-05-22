#ifndef VARIATESETTINGS_H
#define VARIATESETTINGS_H

#include <QWidget>

namespace Ui {
class VariateSettings;
}


/*!
    \brief Класс, реализующий форму с настройками вариации и экрана 
*/
class VariateSettings : public QWidget
{
    Q_OBJECT

public:
    explicit VariateSettings(QWidget *parent = 0);
    ~VariateSettings();
    bool getInfo();
    double getScreenDistance();

private:
    Ui::VariateSettings *ui; ///<Указатель на форму с настройками вариации и экрана
};

#endif // VARIATESETTINGS_H
