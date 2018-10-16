#include "variatesettings.h"
#include "ui_variatesettings.h"

VariateSettings::VariateSettings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VariateSettings)
{
    ui->setupUi(this);
    setWindowTitle(tr("Настройки вариации"));
}

VariateSettings::~VariateSettings()
{
    delete ui;
}

bool VariateSettings::getInfo()
{
    if (ui->varaiteEpsCheckBox->isChecked())
        return true;
    return false;
}
