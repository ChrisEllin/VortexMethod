#include "mainwindow.h"
#include <QApplication>
#include "vector3d.h"


int main(int argc, char *argv[])
{

    QApplication app(argc, argv);


    QString locale = QLocale::system().name();
    QTranslator qtTranslator;
    if (qtTranslator.load("qt_" + locale, QLibraryInfo::location(QLibraryInfo::TranslationsPath)))
         app.installTranslator(&qtTranslator);

    QVector<std::shared_ptr<MultiFrame>> frames;

    frames.push_back(std::make_shared<FourFrame>(Vector3D(0.0,1.0,0.0),Vector3D(1.0,1.0,0.0),Vector3D(1.0,0.0,0.0),Vector3D(0.0,0.0,0.0),0.1));

    Vorton a1(Vector3D (0.0,0.5,0.0),Vector3D(0.0,1.0,0.0),1,0.1);
    Vorton a2(Vector3D (0.5,0.0,0.0),Vector3D(0.0,0.0,0.0),1,0.1);
    Vorton a3(Vector3D (1.0,0.5,0.0),Vector3D(1.0,0.0,0.0),1,0.1);
    Vorton a4(Vector3D (0.5,1.0,0.0),Vector3D(1.0,1.0,0.0),1,0.1);

    Vector3D n1(0.0,0.0,-1.0);
    Vector3D c1(0.5,0.5,0.0);

    //frames.push_back(std::make_shared<FourFrame>(Vector3D(1.0,1.0,0.0),Vector3D(1.0,0.0,0.0),Vector3D(1.0,1.0,1.0),Vector3D(1.0,0.0,1.0),0.1));
    frames.push_back(std::make_shared<FourFrame>(Vector3D(1.0,1.0,0.0),Vector3D(1.0,1.0,1.0),Vector3D(1.0,0.0,1.0),Vector3D(1.0,0.0,0.0),0.1));
    Vorton a5(Vector3D (1.0,0.5,0.0),Vector3D(1.0,1.0,0.0),0,0.1);
    Vorton a6(Vector3D (1.0,0.0,0.5),Vector3D(1.0,0.0,0.0),0,0.1);
    Vorton a7(Vector3D (1.0,1.0,0.5),Vector3D(1.0,1.0,1.0),0,0.1);
    Vorton a8(Vector3D (1.0,0.5,1.0),Vector3D(1.0,0.0,1.0),0,0.1);

    Vector3D n2(1.0,0.0,0.0);
    Vector3D c2(1.0,0.5,0.5);

    frames.push_back(std::make_shared<FourFrame>(Vector3D(0.0,1.0,0.0),Vector3D(0.0,1.0,1.0),Vector3D(1.0,1.0,1.0),Vector3D(1.0,1.0,0.0),0.1));
    Vorton a9(Vector3D (0.5,1.0,0.0),Vector3D(0.0,1.0,0.0),0,0.1);
    Vorton a10(Vector3D (0.0,1.0,0.5),Vector3D(0.0,1.0,1.0),0,0.1);
    Vorton a11(Vector3D (0.5,1.0,1.0),Vector3D(1.0,1.0,1.0),0,0.1);
    Vorton a12(Vector3D (1.0,1.0,0.5),Vector3D(1.0,1.0,0.0),0,0.1);

    Vector3D n3(0.0,1.0,0.0);
    Vector3D c3(0.5,1.0,0.5);

    //frames.push_back(std::make_shared<FourFrame>(Vector3D (0.0,0.0,0.0),Vector3D (0.0,0.0,1.0),Vector3D (0.0,1.0,1.0),Vector3D (0.0,1.0,0.0),0.1));
    frames.push_back(std::make_shared<MultiFrame>(4,Vector3D(0.0,0.5,0.5),Vector3D(0.0,0.0,0.0),Vector3D(0.0,0.0,1.0),0.1));
    Vorton a13(Vector3D (0.0,0.5,0.0),Vector3D (0.0,0.0,0.0),0,0.1);
    Vorton a14(Vector3D (0.0,0.0,0.5),Vector3D (0.0,0.0,1.0),0,0.1);
    Vorton a15(Vector3D (0.0,0.5,1.0),Vector3D (0.0,1.0,1.0),0,0.1);
    Vorton a16(Vector3D (0.0,1.0,0.5),Vector3D (0.0,1.0,0.0),0,0.1);

    Vector3D n4(-1.0,0.0,0.0);
    Vector3D c4(0.0,0.5,0.5);

    frames.push_back(std::make_shared<FourFrame>(Vector3D(0.0,1.0,1.0),Vector3D (0.0,0.0,1.0),Vector3D (1.0,0.0,1.0),Vector3D (1.0,1.0,1.0),0.1));
    Vorton a17(Vector3D (0.5,1.0,1.0), Vector3D(0.0,1.0,1.0),0,0.1);
    Vorton a18(Vector3D (0.0,0.5,1.0),Vector3D (0.0,0.0,1.0),0,0.1);
    Vorton a19(Vector3D (0.5,0.0,1.0),Vector3D (1.0,0.0,1.0),0,0.1);
    Vorton a20(Vector3D (1.0,0.5,1.0),Vector3D (1.0,1.0,1.0),0,0.1);

    Vector3D n5(0.0,0.0,1.0);
    Vector3D c5(0.5,0.5,1.0);

    frames.push_back(std::make_shared<FourFrame>(Vector3D (1.0,0.0,0.0),Vector3D (1.0,0.0,1.0),Vector3D (0.0,0.0,1.0),Vector3D (0.0,0.0,0.0),0.1));
    Vorton a21(Vector3D (0.5,0.0,0.0),Vector3D (1.0,0.0,0.0),0,0.1);
    Vorton a22(Vector3D (1.0,0.0,0.5),Vector3D (1.0,0.0,1.0),0,0.1);
    Vorton a23(Vector3D (0.5,0.0,1.0),Vector3D (0.0,0.0,1.0),0,0.1);
    Vorton a24(Vector3D (0.0,0.0,0.5),Vector3D (0.0,0.0,0.0),0,0.1);

    Vector3D n6(0.0,-1.0,0.0);
    Vector3D c6(0.5,0.0,0.5);

    QVector<Vector3D> normals;
    normals.append(n1);
    normals.append(n2);
    normals.append(n3);
    normals.append(n4);
    normals.append(n5);
    normals.append(n6);

    QVector<Vector3D> cont;
    cont.append(c1);
    cont.append(c2);
    cont.append(c3);
    cont.append(c4);
    cont.append(c5);
    cont.append(c6);

    QVector<Vorton> vorts;

//    vorts.append(a1);
//    vorts.append(a2);
//    vorts.append(a3);
//    vorts.append(a4);
//    vorts.append(a5);
//    vorts.append(a6);
//    vorts.append(a7);
//    vorts.append(a8);
//    vorts.append(a9);
//    vorts.append(a10);
//    vorts.append(a11);
//    vorts.append(a12);
//    vorts.append(a13);
//    vorts.append(a14);
//    vorts.append(a15);
//    vorts.append(a16);
//    vorts.append(a17);
//    vorts.append(a18);
//    vorts.append(a19);
//    vorts.append(a20);
//    vorts.append(a21);
//    vorts.append(a22);
//    vorts.append(a23);
//    vorts.append(a24);

    Vorton b(Vector3D(0.5,0.5,0.5),Vector3D(0.1,0.1,0.1),0,0.1);
    b.setMove(Vector3D(2.0,0.0,0.0));
//    Vorton c(Vector3D(1.3,0.3,0),Vector3D(1.4,0.8,-0.2),0,0.1);
//    c.setMove(Vector3D(3,0.0,0.0));
    vorts.push_back(b);
  //  vorts.push_back(c);
   // Vorton aa(Vector3D(-0.2,-0.2,-0.2),Vector3D(0.0,0.0,0.0),0,0.1);
    //aa.setMove(Vector3D(0.1,0.1,0.3));
    //vorts.push_back(aa);
    //QVector<Vorton> oldVorts=vorts;
    FrameCalculations functions;
    QVector<Vorton> v=functions.getFrameVortons(frames);
    QVector<std::pair<double, double> > paral=functions.makeParalllepiped(v);
    QVector<int> res=functions.universalGetBackTriangle(vorts,paral,0.0,cont,normals,frames);
/*    functions.universalRotateTriangle(vorts,res,paral,0.4,cont,normals,frames)*/;



    MainWindow w;
    w.drawGUI(vorts,frames);
    w.show();
    w.setWindowTitle(QObject::tr("Подводный старт v.2.0"));
    return app.exec();
}
