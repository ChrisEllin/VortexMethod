#include "mainwindow.h"
#include <QApplication>
#include "vector3d.h"



int main(int argc, char *argv[])
{
    QString a="asfasdfasf";
    const char* aa=a.toStdString().c_str();
    qDebug()<<*aa;
    QApplication app(argc, argv);

    QString locale = QLocale::system().name();
    QTranslator qtTranslator;
    if (qtTranslator.load("qt_" + locale, QLibraryInfo::location(QLibraryInfo::TranslationsPath)))
         app.installTranslator(&qtTranslator);

    MainWindow w;
    w.show();
    w.setWindowTitle(QObject::tr("Подводный старт v.2.0"));
    return app.exec();
}
