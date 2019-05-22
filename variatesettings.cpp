#include "variatesettings.h"
#include "ui_variatesettings.h"

/*!
Создает новую форму с заголовком "Настройки вариации" 
*/
VariateSettings::VariateSettings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VariateSettings)
{
    ui->setupUi(this);
    setWindowTitle(tr("Настройки вариации"));
}

/*!
Удаляет текущую форму
*/
VariateSettings::~VariateSettings()
{
    delete ui;
}

/*!
Возвращает переменную, отвечающую за необходимость вариации радиуса вортона
\return Необходимость вариации радиуса вортона
*/
bool VariateSettings::getInfo()
{
    return ui->varaiteEpsCheckBox->isChecked();
}

/*!
Возвращает расстояние до экрана
\return Значение расстояния до экрана
*/
double VariateSettings::getScreenDistance()
{
    return ui->screenDistance->text().toDouble();
}
