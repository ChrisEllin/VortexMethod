#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStyleFactory>


/*!
Создает экземмпляр главного окна; настраивает сигнально-слотовую связь.
*/
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<SolverParameters>("SolverParameters");

    acceleratedGroup= new QActionGroup(this);
    acceleratedGroup->addAction(ui->withoutAccelerateMotionAction);
    acceleratedGroup->addAction(ui->acceleratedMotionAction);

    keyCtrlO = new QShortcut(this);
    keyCtrlO->setKey(Qt::CTRL + Qt::Key_O);

    keyCtrlR = new QShortcut(this);
    keyCtrlR->setKey(Qt::CTRL + Qt::Key_R);
    keyCtrlH = new QShortcut(this);
    keyCtrlH->setKey(Qt::CTRL + Qt::Key_H);
    keyCtrlS=new QShortcut(this);
    keyCtrlS->setKey(Qt::CTRL + Qt::Key_S);

    keyCtrlE = new QShortcut(this);
    keyCtrlE->setKey(Qt::CTRL + Qt::Key_E);

    QSurfaceFormat format;
    format.setMajorVersion(3);
    format.setMinorVersion(3);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setSamples(8);
    ui->openGLWidget->setFormat(format);
    solver=new Solver();
    waitForOpen=new Logger();
    settings =new SolverSettings();
    variateSettings = new VariateSettings();
    preprocessor = new PreprocessorSettings();
    connect (this, SIGNAL(setPlaneXY()), ui->openGLWidget, SLOT(setPlaneXY()));
    connect (this, SIGNAL(setPlaneYX()), ui->openGLWidget, SLOT(setPlaneYX()));
    connect (this, SIGNAL(setPlaneXZ()), ui->openGLWidget, SLOT(setPlaneXZ()));
    connect (this, SIGNAL(setPlaneZX()), ui->openGLWidget, SLOT(setPlaneZX()));
    connect (this, SIGNAL(setPlaneYZ()), ui->openGLWidget, SLOT(setPlaneYZ()));
    connect (this, SIGNAL(setPlaneZY()), ui->openGLWidget, SLOT(setPlaneZY()));
    connect (this, SIGNAL(resetPlane()), ui->openGLWidget, SLOT(resetPlane()));
    connect (this, SIGNAL(drawSegment(QVector3D,QVector3D)), ui->openGLWidget, SLOT(addVorton(QVector3D,QVector3D)));
    connect (this, SIGNAL(drawSegment(QVector3D,QVector3D, SArrow::vort_type)), ui->openGLWidget, SLOT(addVorton(QVector3D,QVector3D,SArrow::vort_type)));
    connect (this, SIGNAL(clearSegments(int)), ui->openGLWidget, SLOT(clearCylinders(int)));
    connect (ui->toolButtonViewZoomIn, SIGNAL(clicked(bool)), ui->openGLWidget, SLOT(zoomIn()));
    connect (ui->toolButtonViewZoomOut, SIGNAL(clicked(bool)), ui->openGLWidget, SLOT(zoomOut()));
    connect (this, SIGNAL(changeShape(MainField::shape)), ui->openGLWidget, SLOT(changeModel(MainField::shape)));
    connect (this, SIGNAL(setHeight(double)), ui->openGLWidget, SLOT(setHeight(double)));
    connect (this, SIGNAL(setRadius(double)), ui->openGLWidget, SLOT(setRadius(double)));
    connect (ui->settingsOpenAction, SIGNAL(triggered(bool)), this, SLOT(showSettings()));
    connect (ui->preprocessorSettingsOpenAction, SIGNAL(triggered(bool)), this, SLOT(showPreprocessor()));
    connect (ui->variationSettingsOpenAction, SIGNAL(triggered(bool)), this, SLOT(showVariateSettings()));
    connect (solver, SIGNAL(sendProgressSphere(const int)), this, SLOT(recieveProgressSphere(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(sendProgressCylinder(const int)), this, SLOT(recieveProgressCylinder(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(sendProgressRotationBody(const int)), this, SLOT(recieveProgressRotationBody(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(sendProgressRotationCutBody(const int)), this, SLOT(recieveProgressRotationCutBody(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL (repaintGUI(const QVector<Vorton>&, const QVector<std::shared_ptr<MultiFrame>>&)), this,
             SLOT(drawGUI(const QVector<Vorton>&, const QVector<std::shared_ptr<MultiFrame>>&)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(updateSphereMaximum(const int)), ui->sphereProgressBar, SLOT(setMaximum(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(updateCylinderMaximum(const int)), ui->cylinderProgressBar, SLOT(setMaximum(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(updateRotationBodyMaximum(const int)), ui->rotationBodyProgressBar, SLOT(setMaximum(const int)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(updateRotationCutBodyMaximum(const int)), ui->rotationCutBodyProgressBar, SLOT(setMaximum(const int)), Qt::BlockingQueuedConnection);
    connect (ui->displaySphereAction, SIGNAL(triggered(bool)), this, SLOT(showSphere()));
    connect (ui->openPassportAction, SIGNAL(triggered(bool)), this, SLOT(openPassport()));
    connect (this, SIGNAL(sendSolverParameters(SolverParameters&)), settings, SLOT(setSolverParameters(SolverParameters&)));
    connect (keyCtrlO, SIGNAL(activated()), this, SLOT(openPassport()));
    connect (keyCtrlR, SIGNAL(activated()), this, SLOT(keyCtrlRActiavated()));
    connect (keyCtrlH, SIGNAL(activated()), this, SLOT(showSphere()));
    connect (keyCtrlS, SIGNAL(activated()), this, SLOT(stop()));
    connect (keyCtrlE, SIGNAL(activated()), this, SLOT(close()));
    connect (solver, SIGNAL(variatingFinished()), this, SLOT(showInfo()));
    connect (solver, SIGNAL(sendMaxGamma(double)), this, SLOT(setMaxGamma(double)));
    connect (ui->updateScreenAction, SIGNAL(triggered(bool)), this, SLOT(updateScreen()));
    connect (solver, SIGNAL(sendNormalsVis(QVector<Vector3D>&,QVector<Vector3D>&)), this, SLOT(setNormalsVis(QVector<Vector3D>&,QVector<Vector3D>&)), Qt::BlockingQueuedConnection);
    connect (solver, SIGNAL(finishSolver()), this, SLOT(solverFinished()));
    connect (ui->openVortonsDirAction, SIGNAL(triggered()),this, SLOT(openDirectory()));
    connect (waitForOpen, SIGNAL(sendVortons(const QVector<Vorton>&, const QVector<Vorton>&)), this , SLOT(drawGUI(const QVector<Vorton>&, const QVector<Vorton>&)),Qt::BlockingQueuedConnection);
    connect (ui->exitAction, SIGNAL(triggered()), this, SLOT(close()));
    connect (ui->stopAction, SIGNAL(triggered()), this, SLOT(stop()));
    connect (ui->getBackGAAction, SIGNAL(triggered()),this, SLOT(changeGetBack()));
    connect (solver, SIGNAL(sendReguliser(double)),this, SLOT(setReguliser(double)));
    connect (ui->epsilonCylinderLineEdit, SIGNAL(textChanged(const QString)),settings,SLOT(calcAttributes(const QString)));
    connect (ui->epsilonSphereLineEdit, SIGNAL(textChanged(const QString)),settings,SLOT(calcAttributes(const QString)));
    connect (ui->epsilonRotationBodyLineEdit, SIGNAL(textChanged(const QString)),settings,SLOT(calcAttributes(const QString)));
    connect (ui->epsilonRotationCutBodyLineEdit, SIGNAL(textChanged(const QString)),settings,SLOT(calcAttributes(const QString)));
    connect (ui->fiRotationBodyLineEdit, SIGNAL(textChanged(const QString)), this, SLOT(calcAverLength()));
    connect (ui->fiRotationCutBodyLineEdit, SIGNAL(textChanged(const QString)), this, SLOT(calcAverLength()));
    connect (ui->epsilonCylinderLineEdit, SIGNAL(textChanged(const QString)),this,SLOT(calcAverLength()));
    connect (ui->epsilonSphereLineEdit, SIGNAL(textChanged(const QString)),this,SLOT(calcAverLength()));
    connect (ui->epsilonRotationBodyLineEdit, SIGNAL(textChanged(const QString)),this,SLOT(calcAverLength()));
    connect (ui->epsilonRotationCutBodyLineEdit, SIGNAL(textChanged(const QString)),this,SLOT(calcAverLength()));

    connect (ui->dirLoadingAction, SIGNAL(triggered()), this, SLOT(loadKadrDir()));
    connect (this, SIGNAL(sendPanelLength(double)),settings,SLOT(calcAttributes(double)));

    connect (this, SIGNAL(sendPanelLength(double)), this, SLOT(calcEpsilonLength(double)));
    displaySphere=true;
    showSphere();
    ui->openGLWidget->backgroundColor = Qt::white;
   // ui->nextCommandLinkButton->setFixedSize(ui->nextCommandLinkButton->icon());
    QApplication::setStyle(QStyleFactory::create("fusion"));
    exitState=ExitState::None;
    closeBox.setWindowTitle("Завершение работы");
    closeBox.setLabelText("Ожидайте завершения работы программы");
    closeBox.setMaximum(0);
    closeBox.setMinimum(0);
    closeBox.setValue(0);
    closeBox.setCancelButton(0);
    closeBox.setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    closeBox.close();
    solving=false;

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    Solver::interrupted=true;
    if (solving)
    {
        closeBox.open();
        closeBox.show();
        event->ignore();
    }
    exitState=ExitState::Closed;
}

/*!
Переводит изображение в XY вид
*/
void MainWindow::on_toolButtonViewXY_clicked()
{
    setPlaneXY();
}

/*!
Переводит изображение в YX вид
*/
void MainWindow::on_toolButtonViewYX_clicked()
{
    setPlaneYX();
}

/*!
Переводит изображение в XZ вид
*/
void MainWindow::on_toolButtonViewXZ_clicked()
{
    setPlaneXZ();
}

/*!
Переводит изображение в ZX вид
*/
void MainWindow::on_toolButtonViewZX_clicked()
{
    setPlaneZX();
}

/*!
Переводит изображение в YZ вид
*/
void MainWindow::on_toolButtonViewYZ_clicked()
{
    setPlaneYZ();
}

/*!
Переводит изображение в ZY вид
*/
void MainWindow::on_toolButtonViewZY_clicked()
{
    setPlaneZY();
}

/*!
Переводит изображение в изначальный вид
*/
void MainWindow::on_toolButtonViewReset_clicked()
{
    resetPlane();
}

/*!
Запускает соответствующий расчет по нажатию (Ctrl+R).
*/
void MainWindow::keyCtrlRActiavated()
{
    if (ui->tabWidget->currentIndex()==0)
    {
        on_sphereSolverPushButton_clicked();
    }
    if (ui->tabWidget->currentIndex()==1)
    {
        on_cylinderSolverPushButton_clicked();
    }
    if (ui->tabWidget->currentIndex()==2)
    {
        on_rotationBodySolverPushButton_clicked();
    }
    if (ui->tabWidget->currentIndex()==3)
    {
        on_rotationCutBodySolverPushButton_clicked();
    }
}

/*!
Изменяет масштаб картинки при повороте колеса мыши.
*/
void MainWindow::wheelEvent(QWheelEvent* e)
{
    if (e->angleDelta().y() > 0)
        ui->toolButtonViewZoomIn->clicked();
    else
        ui->toolButtonViewZoomOut->clicked();
}

/*!
Удаляет главное и дополнительные окна.
*/
MainWindow::~MainWindow()
{
    delete ui;
    delete settings;
    delete solver;
    delete preprocessor;
    delete keyCtrlO;
    delete keyCtrlH;
    delete keyCtrlR;
    delete waitForOpen;
    delete variateSettings;
}

bool MainWindow::checkDrawing(const Vector3D &mid,const Vector3D &tail)
{
    Boundaries boundaries=preprocessor->getBoundaries();
    if (mid.x()<boundaries.minBoundary.x())
        return false;
    if (mid.y()<boundaries.minBoundary.y())
        return false;
    if (mid.z()<boundaries.minBoundary.z())
        return false;
    if (mid.x()>boundaries.maxBoundary.x())
        return false;
    if (mid.y()>boundaries.maxBoundary.y())
        return false;
    if (mid.z()>boundaries.maxBoundary.z())
        return false;
    if (tail.x()<boundaries.minBoundary.x())
        return false;
    if (tail.y()<boundaries.minBoundary.y())
        return false;
    if (tail.z()<boundaries.minBoundary.z())
        return false;
    if (tail.x()>boundaries.maxBoundary.x())
        return false;
    if (tail.y()>boundaries.maxBoundary.y())
        return false;
    if (tail.z()>boundaries.maxBoundary.z())
        return false;
    return true;

}

/*!
Устанавливает параметры разбиения сферы в интерфейс основного окна
\param sphPar Параметры разбиения сферы
*/
void MainWindow::setParameters(SphereParameters& sphPar)
{
    ui->pointsRaisingSphereLineEdit->setText(QString::number(sphPar.raise));
    ui->epsilonSphereLineEdit->setText(QString::number(sphPar.vortonsRad));
    ui->deltaSphereFragmLineEdit->setText(QString::number(sphPar.delta));
    ui->radSphereLineEdit->setText(QString::number(sphPar.radius));
    ui->tetaSphereLineEdit->setText(QString::number(sphPar.tetaFragNum));
    ui->fiSphereLineEdit->setText(QString::number(sphPar.fiFragNum));
    ui->tabWidget->setCurrentIndex(0);
}

/*!
Устанавливает параметры разбиения цилиндра в интерфейс основного окна
\param sphPar Параметры разбиения цилиндра
*/
void MainWindow::setParameters(CylinderParameters &cylPar)
{
    ui->pointsRaisingCylinderLineEdit->setText(QString::number(cylPar.raise));
    ui->epsilonCylinderLineEdit->setText(QString::number(cylPar.vortonsRad));
    ui->deltaCylinderFragmLineEdit->setText(QString::number(cylPar.delta));
    ui->radiusCylinderLineEdit->setText(QString::number(cylPar.radFragNum));
    ui->fiCylinderLineEdit->setText(QString::number(cylPar.fiFragNum));
    ui->heightCylinderLineEdit->setText(QString::number(cylPar.height));
    ui->heightFragmCylinderLineEdit->setText(QString::number(cylPar.heightFragNum));
    ui->diameterCylinderLineEdit->setText(QString::number(cylPar.diameter));
    ui->tabWidget->setCurrentIndex(1);
}

/*!
Устанавливает параметры разбиения тела вращения в интерфейс основного окна
\param sphPar Параметры разбиения тела вращения
*/
void MainWindow::setParameters(RotationBodyParameters &rotBodyPar)
{
    ui->pointsRaisingRotationBodyLineEdit->setText(QString::number(rotBodyPar.raise));
    ui->epsilonRotationBodyLineEdit->setText(QString::number(rotBodyPar.vortonsRad));
    ui->deltaRotationBodyFragmLineEdit->setText(QString::number(rotBodyPar.delta));
    ui->partRotationBodyLineEdit->setText(QString::number(rotBodyPar.partFragNum));
    ui->fiRotationBodyLineEdit->setText(QString::number(rotBodyPar.fiFragNum));
    ui->xBegRotationBodyLineEdit->setText(QString::number(rotBodyPar.xBeg));
    ui->xEndRotationBodyLineEdit->setText(QString::number(rotBodyPar.xEnd));
    ui->sectionDistanceRotationBodyLineEdit->setText(QString::number(rotBodyPar.sectionDistance));
    ui->tabWidget->setCurrentIndex(2);
}

void MainWindow::openDirectory()
{
    QString vortonsDir=QFileDialog::getExistingDirectory(this, tr("Выберите папку"), "/home", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);
    QFuture<void> sphereFuture=QtConcurrent::run(waitForOpen, &Logger::openVortonFiles,vortonsDir);
}

void MainWindow::changeGetBack()
{
    if (ui->getBackGAAction->isChecked())
        Solver::getBackGA=true;
    else
        Solver::getBackGA=false;
}

/*!
Запускает расчет сферы с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_sphereSolverPushButton_clicked()
{
    if(ui->fiSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }
    if(ui->tetaSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по тета"));
        return;
    }
    if(ui->radSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен радиус сферы"));
        return;
    }
    if (ui->deltaSphereFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }


    FragmentationParameters fragPar;
    fragPar.sphereFiFragNum=ui->fiSphereLineEdit->text().toInt();
    fragPar.sphereTetaFragNum=ui->tetaSphereLineEdit->text().toInt();
    fragPar.sphereRad=ui->radSphereLineEdit->text().toDouble();

    fragPar.vortonsRad=ui->epsilonSphereLineEdit->text().toDouble();
    fragPar.delta=ui->deltaSphereFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingSphereLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);
    if (ui->acceleratedMotionAction->isChecked())
        solver->setMotionType(MotionType::ACCELERATED);
    else
        solver->setMotionType(MotionType::NOACCELERATE);
    QFuture<void> sphereFuture=QtConcurrent::run(solver,&Solver::sphereSolver, fragPar);
    solving=true;
    ui->pointsRaisingSphereLineEdit->setDisabled(true);
    ui->epsilonSphereLineEdit->setDisabled(true);
    ui->deltaSphereFragmLineEdit->setDisabled(true);
    ui->radSphereLineEdit->setDisabled(true);
    ui->tetaSphereLineEdit->setDisabled(true);
    ui->fiSphereLineEdit->setDisabled(true);
    ui->sphereSolverPushButton->setDisabled(true);
    ui->sphereFreeMotionSolverPushButton->setDisabled(true);
    ui->variateSphereSolverPushButton->setDisabled(true);
}

/*!
Запускает вариацию параметров сферы с начальными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_variateSphereSolverPushButton_clicked()
{
    if(ui->fiSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }
    if(ui->tetaSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по тета"));
        return;
    }
    if(ui->radSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен радиус сферы"));
        return;
    }
    if (ui->deltaSphereFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }


    FragmentationParameters fragPar;
    fragPar.sphereFiFragNum=ui->fiSphereLineEdit->text().toInt();
    fragPar.sphereTetaFragNum=ui->tetaSphereLineEdit->text().toInt();
    fragPar.sphereRad=ui->radSphereLineEdit->text().toDouble();

    fragPar.vortonsRad=ui->epsilonSphereLineEdit->text().toDouble();
    fragPar.delta=ui->deltaSphereFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingSphereLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);
    ui->sphereProgressBar->setEnabled(false);
    QFuture<void> sphereFuture=QtConcurrent::run(solver,&Solver::variateSphereParameters, fragPar,variateSettings->getInfo());
    solving=true;
}

/*!
Запускает расчет цилиндра с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_cylinderSolverPushButton_clicked()
{
    if(ui->fiCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->radiusCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }
    if(ui->heightFragmCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по высоте"));
        return;
    }

    if (ui->deltaCylinderFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->diameterCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен диаметр цилиндра"));
        return;
    }
    if (ui->heightCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота цилиндра"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.cylinderFiFragNum=ui->fiCylinderLineEdit->text().toInt();
    fragPar.cylinderRadFragNum=ui->radiusCylinderLineEdit->text().toInt();
    fragPar.cylinderHeight=ui->heightCylinderLineEdit->text().toDouble();
    fragPar.cylinderHeightFragNum=ui->heightFragmCylinderLineEdit->text().toInt();
    fragPar.cylinderDiameter=ui->diameterCylinderLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonCylinderLineEdit->text().toDouble();
    fragPar.delta=ui->deltaCylinderFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingCylinderLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);
    QFuture<void> cylinderFuture=QtConcurrent::run(solver,&Solver::cylinderSolver, fragPar);

    solving=true;
    ui->pointsRaisingCylinderLineEdit->setDisabled(true);
    ui->epsilonCylinderLineEdit->setDisabled(true);
    ui->deltaCylinderFragmLineEdit->setDisabled(true);
    ui->radiusCylinderLineEdit->setDisabled(true);
    ui->fiCylinderLineEdit->setDisabled(true);
    ui->heightCylinderLineEdit->setDisabled(true);
    ui->heightFragmCylinderLineEdit->setDisabled(true);
    ui->diameterCylinderLineEdit->setDisabled(true);
    ui->cylinderSolverPushButton->setDisabled(true);
    ui->variateCylinderSolverPushButton->setDisabled(true);
}

/*!
Запускает расчет тела вращения с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_rotationBodySolverPushButton_clicked()
{
    if(ui->fiRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }
    if(ui->xBegRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }
    if (ui->sectionDistanceRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionEndDistance=ui->sectionEndDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodyFormingType=ui->comboBox->currentIndex();
    switch (fragPar.rotationBodyFormingType)
    {
    case 0:
    {
        fragPar.formingDiameter=ui->formingRBDiameterLineEdit->text().toDouble();
        fragPar.formingLengthSectorTwo=ui->formingRBSectorTwoLength->text().toDouble();
        fragPar.formingLengthSectorOne=ui->formingRBSectorOneLength->text().toDouble();
        break;
    }
    case 1:
    {
        fragPar.formingDiameter=ui->formingConeRBDiameterLineEdit->text().toDouble();
        fragPar.formingAngle=ui->formingConeRBAngleLineEdit->text().toDouble();
        fragPar.formingLengthSectorOne=ui->formingConeRBLengthLineEdit->text().toDouble();
        break;
    }
    case 2:
    {
       fragPar.formingDiameter=ui->formingEllipsoidRBDiameterLineEdit->text().toDouble();
       fragPar.formingLengthSectorOne=ui->formingEllipsoidRBLengthLineEdit->text().toDouble();
       break;
    }
    }

    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);
    if (ui->acceleratedMotionAction->isChecked())
        solver->setMotionType(MotionType::ACCELERATED);
    else
        solver->setMotionType(MotionType::NOACCELERATE);

    QFuture<void> rotationBodyFuture=QtConcurrent::run(solver,&Solver::rotationBodySolver, fragPar);
    solving=true;

    ui->pointsRaisingRotationBodyLineEdit->setDisabled(true);
    ui->epsilonRotationBodyLineEdit->setDisabled(true);
    ui->deltaRotationBodyFragmLineEdit->setDisabled(true);
    ui->partRotationBodyLineEdit->setDisabled(true);
    ui->fiRotationBodyLineEdit->setDisabled(true);
    ui->sectionDistanceRotationBodyLineEdit->setDisabled(true);
    ui->xBegRotationBodyLineEdit->setDisabled(true);
    ui->xEndRotationBodyLineEdit->setDisabled(true);
    ui->rotationBodySolverPushButton->setDisabled(true);
    ui->rotationBodySolverPushButton->setDisabled(true);
    ui->rotationBodyFreeMotionSolverPushButton->setDisabled(true);
}

/*!
Выводит дополнительное окно о завершении вариации
*/
void MainWindow::showInfo()
{
    QMessageBox::information(this, tr("Информация"),tr("Вариация параметров завершена"));
}

void MainWindow::stop()
{
    Solver::interrupted=true;
    if (solving)
    {
        closeBox.open();
        closeBox.show();
    }
    exitState=ExitState::Stopped;
}

void MainWindow::solverFinished()
{
    closeBox.close();
    solving=false;
    if (exitState==ExitState::Closed)
        close();
    if (exitState==ExitState::Stopped)
    {
        Solver::interrupted=false;
        if (ui->tabWidget->currentIndex()==0)
        {
            ui->pointsRaisingSphereLineEdit->setDisabled(false);
            ui->epsilonSphereLineEdit->setDisabled(false);
            ui->deltaSphereFragmLineEdit->setDisabled(false);
            ui->radSphereLineEdit->setDisabled(false);
            ui->tetaSphereLineEdit->setDisabled(false);
            ui->fiSphereLineEdit->setDisabled(false);
            ui->sphereSolverPushButton->setDisabled(false);
            ui->variateSphereSolverPushButton->setDisabled(false);
            ui->sphereFreeMotionSolverPushButton->setDisabled(false);
        }

        if (ui->tabWidget->currentIndex()==1)
        {
            ui->pointsRaisingCylinderLineEdit->setDisabled(false);
            ui->epsilonCylinderLineEdit->setDisabled(false);
            ui->deltaCylinderFragmLineEdit->setDisabled(false);
            ui->radiusCylinderLineEdit->setDisabled(false);
            ui->heightCylinderLineEdit->setDisabled(false);
            ui->heightFragmCylinderLineEdit->setDisabled(false);
            ui->fiCylinderLineEdit->setDisabled(false);
            ui->diameterCylinderLineEdit->setDisabled(false);
            ui->cylinderSolverPushButton->setDisabled(false);
            ui->variateCylinderSolverPushButton->setDisabled(false);
        }

        if (ui->tabWidget->currentIndex()==2)
        {
            ui->pointsRaisingRotationBodyLineEdit->setDisabled(false);
            ui->epsilonRotationBodyLineEdit->setDisabled(false);
            ui->deltaRotationBodyFragmLineEdit->setDisabled(false);
            ui->partRotationBodyLineEdit->setDisabled(false);
            ui->fiRotationBodyLineEdit->setDisabled(false);
            ui->sectionDistanceRotationBodyLineEdit->setDisabled(false);
            ui->xBegRotationBodyLineEdit->setDisabled(false);
            ui->xEndRotationBodyLineEdit->setDisabled(false);
            ui->rotationBodySolverPushButton->setDisabled(false);
            ui->variateRotationBodySolverPushButton->setDisabled(false);
            ui->rotationBodyFreeMotionSolverPushButton->setDisabled(false);
        }

        if (ui->tabWidget->currentIndex()==3)
        {
            ui->pointsRaisingRotationCutBodyLineEdit->setDisabled(false);
            ui->epsilonRotationCutBodyLineEdit->setDisabled(false);
            ui->deltaRotationCutBodyFragmLineEdit->setDisabled(false);
            ui->partRotationCutBodyLineEdit->setDisabled(false);
            ui->fiRotationCutBodyLineEdit->setDisabled(false);
            ui->sectionDistanceRotationCutBodyLineEdit->setDisabled(false);
            ui->xBegRotationCutBodyLineEdit->setDisabled(false);
            ui->radRotationCutBodyLineEdit->setDisabled(false);
            ui->xEndRotationCutBodyLineEdit->setDisabled(false);
            ui->rotationCutBodySolverPushButton->setDisabled(false);
            ui->rotationCutBodyFreeMotionSolverPushButton->setDisabled(false);
            ui->rotationCutBodyLaunchSolverPushButton->setDisabled(false);
            ui->rotationCutBodyLaunchSolverPushButton->setDisabled(false);
            ui->rotationCutBodyNearScreenPushButton->setDisabled(false);
            ui->variateRotationCutBodySolverPushButton->setDisabled(false);
        }
    }
}

void MainWindow::setMaxGamma(double maxGamma)
{
    ui->maxGammaLineEdit->setText(QString::number(maxGamma));
}

/*!
Выводит дополнительное окно с настройками расчета
*/
void MainWindow::showSettings()
{
    settings->show();
}

/*!
Отображает трехмерную сферу
*/
void MainWindow::showSphere()
{
    displaySphere=!displaySphere;
    if (displaySphere)
        emit changeShape(MainField::Sphere);
    else
        emit changeShape(MainField::None);
}

void MainWindow::showPreprocessor()
{
    preprocessor->show();
}

/*!
Выводит дополнительное окно с настройками вариации
*/
void MainWindow::showVariateSettings()
{
    variateSettings->show();
}

/*!
Открывает файл с начальными данными расчета для последующего произведения расчета с ними
*/
void MainWindow::openPassport()
{
    QString passportPath=QFileDialog::getOpenFileName(this, tr("Открыть паспорт-файл"),"", tr("Текстовые файлы(*.txt)"));
    QFile passport(passportPath);
    QStringList passportData;
    if (passport.open(QIODevice::ReadOnly))
    {
        QTextStream passportTextStream(&passport);
        while (!passportTextStream.atEnd())
        {
            passportData.push_back(passportTextStream.readLine());
        }

        int currentStrNum=6;

        if(passportData[4][10]==1057) // код буквы С
        {
            SphereParameters sphPar;

            for (int i=currentStrNum; i<currentStrNum+6; i++)
            {
                QStringList strData=passportData[i].split(' ');
                bool ok=false;
                int j=0;
                double value;
                do
                {
                    value=strData[j].toDouble(&ok);
                    j++;
                }
                while(!ok);
                sphPar.setData(i-currentStrNum,value);
            }

            setParameters(sphPar);
            currentStrNum+=6;
        }

        if(passportData[4][10]==1062)  // код буквы Ц
        {
            CylinderParameters cylPar;

            for (int i=currentStrNum; i<currentStrNum+8; i++)
            {
                QStringList strData=passportData[i].split(' ');
                bool ok=false;
                int j=0;
                double value;
                do
                {
                    value=strData[j].toDouble(&ok);
                    j++;
                }
                while(!ok);
                cylPar.setData(i-currentStrNum,value);
            }
            setParameters(cylPar);
            currentStrNum+=8;
        }

        if(passportData[4][10]==1058 && passportData[4].size()<34)  // код буквы Т
        {
            RotationBodyParameters rotBodyPar;
            for (int i=currentStrNum; i<currentStrNum+8; i++)
            {
                QStringList strData=passportData[i].split(' ');
                bool ok=false;
                int j=0;
                double value;
                do
                {
                    value=strData[j].toDouble(&ok);
                    j++;
                }
                while(!ok);
                rotBodyPar.setData(i-currentStrNum,value);
            }
            currentStrNum+=8;
        }

        if(passportData[4][10]==1058 && passportData[4].size()==34)  // код буквы Т
        {
            RotationCutBodyParameters rotBodyPar;
            for (int i=currentStrNum; i<currentStrNum+9; i++)
            {
                QStringList strData=passportData[i].split(' ');
                bool ok=false;
                int j=0;
                double value;
                do
                {
                    value=strData[j].toDouble(&ok);
                    j++;
                }
                while(!ok);
                rotBodyPar.setData(i-currentStrNum,value);
            }
            currentStrNum+=9;
        }


        SolverParameters solvPar;
        for (int i=currentStrNum; i<currentStrNum+14; i++)
        {
            QStringList strData=passportData[i].split(' ');

            bool ok=false;
            int j=0;
            double value;
            QVector<double> values;
            do
            {
                value=strData[j].toDouble(&ok);
                if ((ok)&&(j!=strData.size()-1))
                {
                   ok=false;
                   values.push_back(value);
                }
                j++;
            }
            while(!ok);
            if (values.isEmpty())
                solvPar.setData(i-currentStrNum,value);
            else
            {
                solvPar.setData(Vector3D(values[0],values[1],value));
                values.clear();
            }
        }
        emit sendSolverParameters(solvPar);
    }
    passport.close();
}

void MainWindow::loadKadrDir()
{
    QString vortonsDir=QFileDialog::getExistingDirectory(this, tr("Выберите папку"), "/home", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);
    QFuture<std::shared_ptr<QPair<QVector<QVector<Vorton>>,QVector<QVector<Vorton>>>>> resultFuture=QtConcurrent::run(waitForOpen, &Logger::loadKadrDir,vortonsDir);
    resultFuture.waitForFinished();
    allVortons=std::make_shared<QVector<QVector<Vorton>>>(resultFuture.result().get()->first);
    allFrames=std::make_shared<QVector<QVector<Vorton>>>(resultFuture.result().get()->second);
    emit drawGUI(allVortons.get()->at(0),allFrames.get()->at(0));
    ui->currentNumberLineEdit->setText(QString::number(0));
    currentNumber=0;
}

/*!
Выводит текущий прогресс расчета сферы
\param percentage Текущий шаг
*/
void MainWindow::recieveProgressSphere(const int percentage)
{
    ui->sphereProgressBar->setValue(percentage);
    if (percentage==ui->sphereProgressBar->maximum())
    {
        ui->pointsRaisingSphereLineEdit->setDisabled(false);
        ui->epsilonSphereLineEdit->setDisabled(false);
        ui->deltaSphereFragmLineEdit->setDisabled(false);
        ui->radSphereLineEdit->setDisabled(false);
        ui->tetaSphereLineEdit->setDisabled(false);
        ui->fiSphereLineEdit->setDisabled(false);
        ui->sphereSolverPushButton->setDisabled(false);
        ui->variateSphereSolverPushButton->setDisabled(false);
        ui->sphereFreeMotionSolverPushButton->setDisabled(false);
        solving=false;
    }
}

/*!
Выводит текущий прогресс расчета цилиндра
\param percentage Текущий шаг
*/
void MainWindow::recieveProgressCylinder(const int percentage)
{
    ui->cylinderProgressBar->setValue(percentage);
    if (percentage==ui->cylinderProgressBar->maximum())
    {
        ui->pointsRaisingCylinderLineEdit->setDisabled(false);
        ui->epsilonCylinderLineEdit->setDisabled(false);
        ui->deltaCylinderFragmLineEdit->setDisabled(false);
        ui->radiusCylinderLineEdit->setDisabled(false);
        ui->heightCylinderLineEdit->setDisabled(false);
        ui->heightFragmCylinderLineEdit->setDisabled(false);
        ui->fiCylinderLineEdit->setDisabled(false);
        ui->diameterCylinderLineEdit->setDisabled(false);
        ui->cylinderSolverPushButton->setDisabled(false);
        ui->variateCylinderSolverPushButton->setDisabled(false);
        solving=false;
    }
}

/*!
Выводит текущий прогресс расчета тела вращения
\param percentage Текущий шаг
*/
void MainWindow::recieveProgressRotationBody(const int percentage)
{
    ui->rotationBodyProgressBar->setValue(percentage);
    if (percentage==ui->rotationBodyProgressBar->maximum())
    {
        ui->pointsRaisingRotationBodyLineEdit->setDisabled(false);
        ui->epsilonRotationBodyLineEdit->setDisabled(false);
        ui->deltaRotationBodyFragmLineEdit->setDisabled(false);
        ui->partRotationBodyLineEdit->setDisabled(false);
        ui->fiRotationBodyLineEdit->setDisabled(false);
        ui->sectionDistanceRotationBodyLineEdit->setDisabled(false);
        ui->xBegRotationBodyLineEdit->setDisabled(false);
        ui->xEndRotationBodyLineEdit->setDisabled(false);
        ui->rotationBodySolverPushButton->setDisabled(false);
        ui->variateRotationBodySolverPushButton->setDisabled(false);
        ui->rotationBodyFreeMotionSolverPushButton->setDisabled(false);
        solving=false;
    }
}

/*!
Выводит текущий прогресс расчета тела вращения со срезом
\param percentage Текущий шаг
*/
void MainWindow::recieveProgressRotationCutBody(const int percentage)
{
    ui->rotationCutBodyProgressBar->setValue(percentage);
    if (percentage==ui->rotationCutBodyProgressBar->maximum())
    {
        ui->pointsRaisingRotationCutBodyLineEdit->setDisabled(false);
        ui->epsilonRotationCutBodyLineEdit->setDisabled(false);
        ui->deltaRotationCutBodyFragmLineEdit->setDisabled(false);
        ui->partRotationCutBodyLineEdit->setDisabled(false);
        ui->fiRotationCutBodyLineEdit->setDisabled(false);
        ui->sectionDistanceRotationCutBodyLineEdit->setDisabled(false);
        ui->xBegRotationCutBodyLineEdit->setDisabled(false);
        ui->radRotationCutBodyLineEdit->setDisabled(false);
        ui->xEndRotationCutBodyLineEdit->setDisabled(false);
        ui->rotationCutBodySolverPushButton->setDisabled(false);
        ui->rotationCutBodyFreeMotionSolverPushButton->setDisabled(false);
        ui->rotationCutBodyLaunchSolverPushButton->setDisabled(false);
        ui->rotationCutBodyLaunchSolverPushButton->setDisabled(false);
        ui->rotationCutBodyNearScreenPushButton->setDisabled(false);
        ui->variateRotationCutBodySolverPushButton->setDisabled(false);
        solving=false;
    }
}

/*!
Перерисовывает визуализацию расчета
\param vortons Вектор вортонов(отображается точками)
\param frames Вектор рамок(отображается стрелками)
*/
void MainWindow::drawGUI(const QVector<Vorton> &vortons, const QVector<std::shared_ptr<MultiFrame> > &frames)
{
    clearSegments();
    currentVortons=vortons;
    currentFrames=frames;
    for (int i=0; i<frames.size(); i++)
    {
        for (int j = 0; j < frames[i]->getAnglesNum(); j++)
        {
            QVector3D mid = Vector3D::toQVector3D(frames[i]->at(j).getMid());
            QVector3D tail = Vector3D::toQVector3D(frames[i]->at(j).getTail());
            emit drawSegment(2.0*mid - tail, tail, SArrow::Grid);
        }
    }
    ui->freeVortonsQuantityLineEdit->setText(QString::number(vortons.size()));

    if(!frames.isEmpty())
    {
        for (int i=0; i<vortons.size(); i++)
    {
        QVector3D mid=Vector3D::toQVector3D(vortons[i].getMid());
        QVector3D tail=Vector3D::toQVector3D(vortons[i].getTail());
        if (checkDrawing(vortons[i].getMid(),vortons[i].getTail()))
            emit drawSegment(2.0*mid-tail, tail);
    }
    }
    else
    {
        for (int i=0; i<vortons.size(); i++)
    {
        QVector3D mid=Vector3D::toQVector3D(vortons[i].getMid());
        QVector3D tail=Vector3D::toQVector3D(vortons[i].getTail());
        if (checkDrawing(vortons[i].getMid(),vortons[i].getTail()))
            emit drawSegment(2.0*mid-tail, tail,SArrow::Grid);
    }
    }

    if (!currentControlPoints.isEmpty() && !currentNormals.isEmpty() && preprocessor->normalsDrawing())
    {
        for (int i=0; i<currentNormals.size();i++)
            emit drawSegment(Vector3D::toQVector3D(currentControlPoints[i]), Vector3D::toQVector3D(currentControlPoints[i]+currentNormals[i]), SArrow::Grid);
    }
}


void MainWindow::drawGUI(const QVector<Vorton> &vortons, const QVector<Vorton> &frames)
{
    clearSegments();
    for (int i=0; i<frames.size(); i++)
    {
        QVector3D mid=Vector3D::toQVector3D(frames[i].getMid());
        QVector3D tail=Vector3D::toQVector3D(frames[i].getTail());
         emit drawSegment(2.0*mid-tail, tail,SArrow::Grid);
    }
    ui->freeVortonsQuantityLineEdit->setText(QString::number(vortons.size()));
    QVector<double> vorticities;
    for (int i=0; i<vortons.size(); i++)
        vorticities.push_back(vortons[i].getVorticity());
    ui->maxGammaLineEdit->setText(QString::number(*std::max_element(vorticities.begin(),vorticities.end(),Vector3D::fabsCompare)));
    for (int i=0; i<vortons.size(); i++)
    {
        QVector3D mid=Vector3D::toQVector3D(vortons[i].getMid());
        QVector3D tail=Vector3D::toQVector3D(vortons[i].getTail());
        if (checkDrawing(vortons[i].getMid(),vortons[i].getTail()))
            emit drawSegment(2.0*mid-tail, tail);
    }

    if (!currentControlPoints.isEmpty() && !currentNormals.isEmpty() && preprocessor->normalsDrawing())
    {
        for (int i=0; i<currentNormals.size();i++)
            emit drawSegment(Vector3D::toQVector3D(currentControlPoints[i]), Vector3D::toQVector3D(currentControlPoints[i]+currentNormals[i]), SArrow::Grid);
    }
}

void MainWindow::setNormalsVis(QVector<Vector3D> &controlPoints, QVector<Vector3D> &normals)
{
    currentControlPoints=controlPoints;
    currentNormals=normals;
}

/*!
Запускает расчет тела вращения со срезом с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_rotationCutBodySolverPushButton_clicked()
{
    if(ui->fiRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }

    if(ui->radRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }

    if(ui->xBegRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationCutBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }

    if (ui->sectionDistanceRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyRFragNum=ui->radRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionEndDistance=ui->sectionEndDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationCutBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationCutBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationCutBodyLineEdit->text().toDouble();
    fragPar.formingEllipsoidDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
    fragPar.formingLengthSectorOne=ui->formingRBCSectorOneLength->text().toDouble();
    fragPar.formingLengthSectorTwo=ui->formingRBCSectorTwoLength->text().toDouble();
    fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
    ui->formRotationCutBodyComboBox->currentIndex()==0 ? fragPar.rotationBodyRBCFormingType=ELLIPSOID_CYLINDER :
            (ui->formRotationCutBodyComboBox->currentIndex()==1 ? fragPar.rotationBodyRBCFormingType=ELLIPSOID_CONE :fragPar.rotationBodyRBCFormingType=ELLIPSOID_CYLINDER_CONE);
    switch (fragPar.rotationBodyRBCFormingType) {
    case ELLIPSOID_CONE:
    {
        fragPar.formingEllipsoidDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
        fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->formingRBCSectorOneLength->text().toDouble();
        fragPar.formingConeLength=ui->formingRBCSectorTwoLength->text().toDouble();
        fragPar.formingFullLength=fragPar.formingEllisoidLength+fragPar.formingConeLength;
        break;
    }
    case ELLIPSOID_CYLINDER:
    {
        fragPar.formingEllipsoidDiameter=ui->cylDiameterRotationCutBodyLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->ellipsoidLengthRotationCutBodyLineEdit->text().toDouble();
        fragPar.formingFullLength=ui->lengthRotationCutBodyLineEdit->text().toDouble();
        break;
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        fragPar.formingEllipsoidDiameter=ui->formingRBC3FormDiameterLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->formingRBC3FormSectorOneLength->text().toDouble();
        fragPar.formingConeLength=ui->formingRBC3FormSectorThreeLength->text().toDouble();
        fragPar.formingFullLength=ui->formingRBC3FormSectorTwoLength->text().toDouble()+fragPar.formingConeLength+fragPar.formingEllisoidLength;
        fragPar.formingTailDiameter=ui->formingRBC3FormTailDiameterLineEdit->text().toDouble();
        break;
    }
    }

    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);

    if (ui->acceleratedMotionAction->isChecked())
        solver->setMotionType(MotionType::ACCELERATED);
    else
        solver->setMotionType(MotionType::NOACCELERATE);
    QFuture<void> rotationCutBodyFuture=QtConcurrent::run(solver,&Solver::rotationCutBodySolver, fragPar);
    solving=true;
    ui->pointsRaisingRotationCutBodyLineEdit->setDisabled(true);
    ui->radRotationCutBodyLineEdit->setDisabled(true);
    ui->epsilonRotationCutBodyLineEdit->setDisabled(true);
    ui->deltaRotationCutBodyFragmLineEdit->setDisabled(true);
    ui->partRotationCutBodyLineEdit->setDisabled(true);
    ui->fiRotationCutBodyLineEdit->setDisabled(true);
    ui->sectionDistanceRotationCutBodyLineEdit->setDisabled(true);
    ui->xBegRotationCutBodyLineEdit->setDisabled(true);
    ui->xEndRotationCutBodyLineEdit->setDisabled(true);
    ui->rotationCutBodySolverPushButton->setDisabled(true);
    ui->rotationCutBodyFreeMotionSolverPushButton->setDisabled(true);
    ui->rotationCutBodyLaunchSolverPushButton->setDisabled(true);
    ui->rotationCutBodyNearScreenPushButton->setDisabled(true);
    ui->variateRotationCutBodySolverPushButton->setDisabled(true);
}

/*!
Запускает расчет движущегося тела вращения с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_rotationCutBodyFreeMotionSolverPushButton_clicked()
{
    if(ui->fiRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }

    if(ui->radRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }

    if(ui->xBegRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationCutBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }

    if (ui->sectionDistanceRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyRFragNum=ui->radRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionEndDistance=ui->sectionEndDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationCutBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationCutBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationCutBodyLineEdit->text().toDouble();
    fragPar.formingDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
    fragPar.formingLengthSectorOne=ui->formingRBCSectorOneLength->text().toDouble();
    fragPar.formingLengthSectorTwo=ui->formingRBCSectorTwoLength->text().toDouble();
    fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
    ui->formRotationCutBodyComboBox->currentIndex()==0 ? fragPar.rotationBodyRBCFormingType=ELLIPSOID_CYLINDER :
            (ui->formRotationCutBodyComboBox->currentIndex()==1 ? fragPar.rotationBodyRBCFormingType=ELLIPSOID_CONE :fragPar.rotationBodyRBCFormingType=ELLIPSOID_CYLINDER_CONE);
    switch (fragPar.rotationBodyRBCFormingType) {
    case ELLIPSOID_CONE:
    {
        fragPar.formingEllipsoidDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
        fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->formingRBCSectorOneLength->text().toDouble();
        fragPar.formingConeLength=ui->formingRBCSectorTwoLength->text().toDouble();
        fragPar.formingFullLength=fragPar.formingEllisoidLength+fragPar.formingConeLength;
        break;
    }
    case ELLIPSOID_CYLINDER:
    {
        fragPar.formingEllipsoidDiameter=ui->cylDiameterRotationCutBodyLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->ellipsoidLengthRotationCutBodyLineEdit->text().toDouble();
        fragPar.formingFullLength=ui->lengthRotationCutBodyLineEdit->text().toDouble();
        break;
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        fragPar.formingEllipsoidDiameter=ui->formingRBC3FormDiameterLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->formingRBC3FormSectorOneLength->text().toDouble();
        fragPar.formingConeLength=ui->formingRBC3FormSectorThreeLength->text().toDouble();
        fragPar.formingFullLength=ui->formingRBC3FormSectorTwoLength->text().toDouble()+fragPar.formingConeLength+fragPar.formingEllisoidLength;
        fragPar.formingTailDiameter=ui->formingRBC3FormTailDiameterLineEdit->text().toDouble();
        break;
    }
    }
    SolverParameters solvPar=settings->getSolverParameters();
    FreeMotionParameters freeMotionPar=settings->getFreeMotionParameters();
    *solver=Solver(solvPar,freeMotionPar);
    QFuture<void> rotationCutBodyFuture=QtConcurrent::run(solver,&Solver::rotationCutBodyFreeMotionSolver, fragPar);
    solving=true;
    ui->pointsRaisingRotationCutBodyLineEdit->setDisabled(true);
    ui->radRotationCutBodyLineEdit->setDisabled(true);
    ui->epsilonRotationCutBodyLineEdit->setDisabled(true);
    ui->deltaRotationCutBodyFragmLineEdit->setDisabled(true);
    ui->partRotationCutBodyLineEdit->setDisabled(true);
    ui->fiRotationCutBodyLineEdit->setDisabled(true);
    ui->sectionDistanceRotationCutBodyLineEdit->setDisabled(true);
    ui->xBegRotationCutBodyLineEdit->setDisabled(true);
    ui->xEndRotationCutBodyLineEdit->setDisabled(true);
    ui->rotationCutBodySolverPushButton->setDisabled(true);
    ui->rotationCutBodyFreeMotionSolverPushButton->setDisabled(true);
    ui->rotationCutBodyLaunchSolverPushButton->setDisabled(true);
}

/*!
Запускает расчет задачи старта для тела вращения с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_rotationCutBodyLaunchSolverPushButton_clicked()
{
    if(ui->fiRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }

    if(ui->radRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }

    if(ui->xBegRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationCutBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }

    if (ui->sectionDistanceRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyRFragNum=ui->radRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionEndDistance=ui->sectionEndDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationCutBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationCutBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationCutBodyLineEdit->text().toDouble();
    fragPar.formingDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
    fragPar.formingLengthSectorOne=ui->formingRBCSectorOneLength->text().toDouble();
    fragPar.formingLengthSectorTwo=ui->formingRBCSectorTwoLength->text().toDouble();
    fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
    ui->formRotationCutBodyComboBox->currentIndex()==0 ? fragPar.rotationBodyRBCFormingType=ELLIPSOID_CYLINDER :
            (ui->formRotationCutBodyComboBox->currentIndex()==1 ? fragPar.rotationBodyRBCFormingType=ELLIPSOID_CONE :fragPar.rotationBodyRBCFormingType=ELLIPSOID_CYLINDER_CONE);
    switch (fragPar.rotationBodyRBCFormingType) {
    case ELLIPSOID_CONE:
    {
        fragPar.formingEllipsoidDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
        fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->formingRBCSectorOneLength->text().toDouble();
        fragPar.formingConeLength=ui->formingRBCSectorTwoLength->text().toDouble();
        fragPar.formingFullLength=fragPar.formingEllisoidLength+fragPar.formingConeLength;
        break;
    }
    case ELLIPSOID_CYLINDER:
    {
        fragPar.formingEllipsoidDiameter=ui->cylDiameterRotationCutBodyLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->ellipsoidLengthRotationCutBodyLineEdit->text().toDouble();
        fragPar.formingFullLength=ui->lengthRotationCutBodyLineEdit->text().toDouble();
        break;
    }
    case ELLIPSOID_CYLINDER_CONE:
    {
        fragPar.formingEllipsoidDiameter=ui->formingRBC3FormDiameterLineEdit->text().toDouble();
        fragPar.formingEllisoidLength=ui->formingRBC3FormSectorOneLength->text().toDouble();
        fragPar.formingConeLength=ui->formingRBC3FormSectorThreeLength->text().toDouble();
        fragPar.formingFullLength=ui->formingRBC3FormSectorTwoLength->text().toDouble()+fragPar.formingConeLength+fragPar.formingEllisoidLength;
        fragPar.formingTailDiameter=ui->formingRBC3FormTailDiameterLineEdit->text().toDouble();
        break;
    }
    }
    SolverParameters solvPar=settings->getSolverParameters();
    FreeMotionParameters freeMotionPar=settings->getFreeMotionParameters();
    *solver=Solver(solvPar,freeMotionPar);
    QFuture<void> rotationCutBodyFuture=QtConcurrent::run(solver,&Solver::rotationCutBodyLaunchSolver, fragPar);
    solving=true;

    ui->pointsRaisingRotationCutBodyLineEdit->setDisabled(true);
    ui->radRotationCutBodyLineEdit->setDisabled(true);
    ui->epsilonRotationCutBodyLineEdit->setDisabled(true);
    ui->deltaRotationCutBodyFragmLineEdit->setDisabled(true);
    ui->partRotationCutBodyLineEdit->setDisabled(true);
    ui->fiRotationCutBodyLineEdit->setDisabled(true);
    ui->sectionDistanceRotationCutBodyLineEdit->setDisabled(true);
    ui->xBegRotationCutBodyLineEdit->setDisabled(true);
    ui->xEndRotationCutBodyLineEdit->setDisabled(true);
    ui->rotationCutBodySolverPushButton->setDisabled(true);
    ui->rotationCutBodyFreeMotionSolverPushButton->setDisabled(true);
    ui->rotationCutBodyLaunchSolverPushButton->setDisabled(true);
    ui->rotationCutBodyNearScreenPushButton->setDisabled(true);
    ui->variateRotationCutBodySolverPushButton->setDisabled(true);
}

/*!
Запускает расчет задачи старта для сферы с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_sphereFreeMotionSolverPushButton_clicked()
{
    if(ui->fiSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }
    if(ui->tetaSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по тета"));
        return;
    }
    if(ui->radSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен радиус сферы"));
        return;
    }
    if (ui->deltaSphereFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingSphereLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }


    FragmentationParameters fragPar;
    fragPar.sphereFiFragNum=ui->fiSphereLineEdit->text().toInt();
    fragPar.sphereTetaFragNum=ui->tetaSphereLineEdit->text().toInt();
    fragPar.sphereRad=ui->radSphereLineEdit->text().toDouble();

    fragPar.vortonsRad=ui->epsilonSphereLineEdit->text().toDouble();
    fragPar.delta=ui->deltaSphereFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingSphereLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();

    FreeMotionParameters freeMotionPar=settings->getFreeMotionParameters();
    *solver=Solver(solvPar,freeMotionPar);
    QFuture<void> sphereFuture=QtConcurrent::run(solver,&Solver::sphereFreeMotionSolver, fragPar);
    solving=true;
    ui->pointsRaisingSphereLineEdit->setDisabled(true);
    ui->epsilonSphereLineEdit->setDisabled(true);
    ui->deltaSphereFragmLineEdit->setDisabled(true);
    ui->radSphereLineEdit->setDisabled(true);
    ui->tetaSphereLineEdit->setDisabled(true);
    ui->fiSphereLineEdit->setDisabled(true);
    ui->sphereSolverPushButton->setDisabled(true);
    ui->sphereFreeMotionSolverPushButton->setDisabled(true);
    ui->variateSphereSolverPushButton->setDisabled(true);
}

void MainWindow::updateScreen()
{
    drawGUI(currentVortons,currentFrames);
}

void MainWindow::calcAverLength()
{
    if (ui->autoParametersAction->isChecked())
    {
    double panelLength;
    switch (ui->tabWidget->currentIndex())
    {
    case 0:
        panelLength = (M_PI/ui->fiSphereLineEdit->text().toInt()*ui->radSphereLineEdit->text().toDouble()*2.0)
                > (M_PI/ui->tetaSphereLineEdit->text().toInt()*ui->radSphereLineEdit->text().toDouble()*2.0) ? M_PI/ui->fiSphereLineEdit->text().toInt()*ui->radSphereLineEdit->text().toDouble()*2.0
                : M_PI/ui->tetaSphereLineEdit->text().toInt()*ui->radSphereLineEdit->text().toDouble()*2.0;

        ui->pointsRaisingSphereLineEdit->setText(QString::number(panelLength*0.5));
        break;
    case 1:
        panelLength = (M_PI/ui->fiCylinderLineEdit->text().toInt()*ui->diameterCylinderLineEdit->text().toDouble())
                > (ui->heightCylinderLineEdit->text().toDouble()/ui->heightFragmCylinderLineEdit->text().toInt())
                ? (M_PI/ui->fiCylinderLineEdit->text().toInt()*ui->diameterCylinderLineEdit->text().toDouble())
                : (ui->heightCylinderLineEdit->text().toDouble()/ui->heightFragmCylinderLineEdit->text().toInt());
        ui->pointsRaisingCylinderLineEdit->setText(QString::number(panelLength*0.5));
        break;
    case 2:
        switch (ui->stackedWidget->currentIndex())
        {
        case 0:
//            panelLength = (M_PI/ui->fiRotationBodyLineEdit->text().toInt()*ui->formingRBDiameterLineEdit->text().toDouble())
//                    > ((ui->formingRBDiameterLineEdit->text().toDouble()*0.5*M_PI + ui->formingRBSectorOneLength->text().toDouble()+
//                        ui->formingRBSectorTwoLength->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt())
//                    ? (M_PI/ui->fiRotationBodyLineEdit->text().toInt()*ui->formingRBDiameterLineEdit->text().toDouble())
//                    :((ui->formingRBDiameterLineEdit->text().toDouble()*0.5*M_PI + ui->formingRBSectorOneLength->text().toDouble()+
//                       ui->formingRBSectorTwoLength->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt());
             ui->partRotationBodyLineEdit->setText(QString::number(static_cast<int>((ui->fiRotationBodyLineEdit->text().toInt()*(0.5*M_PI*ui->formingRBDiameterLineEdit->text().toDouble()+
                                                                                                         ui->formingRBSectorOneLength->text().toDouble()+ui->formingRBSectorTwoLength->text().toDouble()))/M_PI/ui->formingRBDiameterLineEdit->text().toDouble())));
             panelLength = M_PI/ui->fiRotationBodyLineEdit->text().toInt()*ui->formingRBDiameterLineEdit->text().toDouble();
            break;
        case 1:
            ui->partRotationBodyLineEdit->setText(QString::number(static_cast<int>(ui->fiRotationBodyLineEdit->text().toInt()*(ui->formingConeRBLengthLineEdit->text().toDouble())/M_PI/ui->formingConeRBDiameterLineEdit->text().toDouble())));
             panelLength = (M_PI/ui->fiRotationBodyLineEdit->text().toInt()*ui->formingConeRBDiameterLineEdit->text().toDouble())
                    > ((ui->formingConeRBLengthLineEdit->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt())
                    ? ((ui->formingConeRBLengthLineEdit->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt())
                    : ((ui->formingConeRBLengthLineEdit->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt());
            break;
        case 2:
            ui->partRotationBodyLineEdit->setText(QString::number(static_cast<int>(ui->fiRotationBodyLineEdit->text().toInt()*(ui->formingEllipsoidRBLengthLineEdit->text().toDouble())/M_PI/ui->formingEllipsoidRBDiameterLineEdit->text().toDouble())));
            panelLength = (M_PI/ui->fiRotationBodyLineEdit->text().toInt()*ui->formingEllipsoidRBDiameterLineEdit->text().toDouble())
                    > ((ui->formingEllipsoidRBLengthLineEdit->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt())
                    ? (M_PI/ui->fiRotationBodyLineEdit->text().toInt()*ui->formingEllipsoidRBDiameterLineEdit->text().toDouble())
                    : ((ui->formingEllipsoidRBLengthLineEdit->text().toDouble())/ui->partRotationBodyLineEdit->text().toInt());
            break;
        }
        ui->pointsRaisingRotationBodyLineEdit->setText(QString::number(panelLength*0.5));
        break;
    case 3:
        switch(ui->stackedWidget_2->currentIndex())
        {
        case 1:
        ui->partRotationCutBodyLineEdit->setText(QString::number(static_cast<int>(ui->fiRotationCutBodyLineEdit->text().toInt()*(ui->formingRBCSectorOneLength->text().toDouble()+ui->formingRBCSectorTwoLength->text().toDouble())/M_PI/ui->formingRBCDiameterLineEdit->text().toDouble())));

        panelLength = (M_PI/ui->fiRotationCutBodyLineEdit->text().toInt()*ui->formingRBCDiameterLineEdit->text().toDouble())
                > ((ui->formingRBCSectorOneLength->text().toDouble()+ui->formingRBCSectorTwoLength->text().toDouble())/ui->partRotationCutBodyLineEdit->text().toInt())
                ? (M_PI/ui->fiRotationCutBodyLineEdit->text().toInt()*ui->formingRBCDiameterLineEdit->text().toDouble())
                : ((ui->formingRBCSectorOneLength->text().toDouble()+ui->formingRBCSectorTwoLength->text().toDouble())/ui->partRotationCutBodyLineEdit->text().toInt());
        ui->radRotationCutBodyLineEdit->setText(QString::number(static_cast<int>(ui->formingRBCTailDiameterLineEdit->text().toDouble()/panelLength)));
        if (ui->radRotationCutBodyLineEdit->text().toInt()==0)
            ui->radRotationCutBodyLineEdit->setText("1");
        ui->pointsRaisingRotationCutBodyLineEdit->setText(QString::number(panelLength*0.5));
        break;
        case 0:
            ui->partRotationCutBodyLineEdit->setText(QString::number(static_cast<int>(ui->fiRotationCutBodyLineEdit->text().toInt()*ui->lengthRotationCutBodyLineEdit->text().toDouble()/M_PI/ui->cylDiameterRotationCutBodyLineEdit->text().toDouble())));
            panelLength=ui->lengthRotationCutBodyLineEdit->text().toDouble()/ ui->partRotationCutBodyLineEdit->text().toInt();
            ui->radRotationCutBodyLineEdit->setText(QString::number(static_cast<int>(ui->cylDiameterRotationCutBodyLineEdit->text().toDouble()/panelLength)));
            if (ui->radRotationCutBodyLineEdit->text().toInt()==0)
                ui->radRotationCutBodyLineEdit->setText("1");
            ui->pointsRaisingRotationCutBodyLineEdit->setText(QString::number(panelLength*0.5));
            break;
        case 2:
            ui->partRotationCutBodyLineEdit->setText(QString::number(static_cast<int>(ui->fiRotationCutBodyLineEdit->text().toInt()*(ui->formingRBC3FormSectorOneLength->text().toDouble()+ui->formingRBC3FormSectorTwoLength->text().toDouble()+ui->formingRBC3FormSectorThreeLength->text().toDouble())/M_PI/ui->formingRBC3FormDiameterLineEdit->text().toDouble())));
            panelLength=(ui->formingRBC3FormSectorThreeLength->text().toDouble()+ui->formingRBC3FormSectorOneLength->text().toDouble()+ui->formingRBC3FormSectorTwoLength->text().toDouble())/
                    ui->partRotationCutBodyLineEdit->text().toInt();
            qDebug()<<panelLength;
            ui->radRotationCutBodyLineEdit->setText(QString::number(static_cast<int>(ui->formingRBC3FormTailDiameterLineEdit->text().toDouble()/panelLength)));
            if (ui->radRotationCutBodyLineEdit->text().toInt()==0)
                ui->radRotationCutBodyLineEdit->setText("1");
            ui->pointsRaisingRotationCutBodyLineEdit->setText(QString::number(panelLength*0.5));
            break;
        }
    }
    emit sendPanelLength(panelLength);
    }
}

/*!
Запускает вариацию параметров цилиндра с начальными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_variateCylinderSolverPushButton_clicked()
{
    if(ui->fiCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->radiusCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }
    if(ui->heightFragmCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по высоте"));
        return;
    }

    if (ui->deltaCylinderFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->diameterCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введен диаметр цилиндра"));
        return;
    }
    if (ui->heightCylinderLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота цилиндра"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.cylinderFiFragNum=ui->fiCylinderLineEdit->text().toInt();
    fragPar.cylinderRadFragNum=ui->radiusCylinderLineEdit->text().toInt();
    fragPar.cylinderHeight=ui->heightCylinderLineEdit->text().toDouble();
    fragPar.cylinderHeightFragNum=ui->heightFragmCylinderLineEdit->text().toInt();
    fragPar.cylinderDiameter=ui->diameterCylinderLineEdit->text().toDouble();

    fragPar.vortonsRad=ui->epsilonCylinderLineEdit->text().toDouble();
    fragPar.delta=ui->deltaCylinderFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingCylinderLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();
    solving=true;
    *solver=Solver(solvPar);
    ui->cylinderProgressBar->setEnabled(false);
    QFuture<void> cylinderFuture=QtConcurrent::run(solver,&Solver::variateCylinderParameters, fragPar,variateSettings->getInfo());
}

/*!
Запускает вариацию параметров тела вращения с начальными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_variateRotationBodySolverPushButton_clicked()
{
    if(ui->fiRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }
    if(ui->xBegRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }
    if (ui->sectionDistanceRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }


    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationBodyLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();
    solving=true;
    *solver=Solver(solvPar);
    QFuture<void> rotationBodyFuture=QtConcurrent::run(solver,&Solver::variateRotationBodyParameters, fragPar,variateSettings->getInfo());
}

/*!
Запускает вариацию параметров тела вращения со срезом с начальными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_variateRotationCutBodySolverPushButton_clicked()
{
    if(ui->fiRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }

    if(ui->radRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }

    if(ui->xBegRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationCutBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }

    if (ui->sectionDistanceRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyRFragNum=ui->radRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationCutBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationCutBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationCutBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationCutBodyLineEdit->text().toDouble();

    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);
    QFuture<void> rotationCutBodyFuture=QtConcurrent::run(solver,&Solver::variateRotationCutBodyParameters, fragPar, variateSettings->getInfo());
    solving=true;
}

/*!
Запускает расчет тела вращения со срезом вблизи экрана с заданными параметрами. Проверяет полноту данных для этого.
*/
void MainWindow::on_rotationCutBodyNearScreenPushButton_clicked()
{
    if(ui->fiRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }

    if(ui->radRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по радиусу"));
        return;
    }

    if(ui->xBegRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationCutBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }

    if (ui->sectionDistanceRotationCutBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyRFragNum=ui->radRotationCutBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationCutBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionEndDistance=ui->sectionEndDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationCutBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationCutBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationCutBodyLineEdit->text().toDouble();
    fragPar.formingDiameter=ui->formingRBCDiameterLineEdit->text().toDouble();
    fragPar.formingLengthSectorOne=ui->formingRBCSectorOneLength->text().toDouble();
    fragPar.formingLengthSectorTwo=ui->formingRBCSectorTwoLength->text().toDouble();
    fragPar.formingTailDiameter=ui->formingRBCTailDiameterLineEdit->text().toDouble();
    SolverParameters solvPar=settings->getSolverParameters();

    *solver=Solver(solvPar);
    QFuture<void> rotationCutBodyFuture=QtConcurrent::run(solver,&Solver::rotationCutBodySolverNearScreen, fragPar, variateSettings->getScreenDistance());
    solving=true;
    ui->pointsRaisingRotationCutBodyLineEdit->setDisabled(true);
    ui->radRotationCutBodyLineEdit->setDisabled(true);
    ui->epsilonRotationCutBodyLineEdit->setDisabled(true);
    ui->deltaRotationCutBodyFragmLineEdit->setDisabled(true);
    ui->partRotationCutBodyLineEdit->setDisabled(true);
    ui->fiRotationCutBodyLineEdit->setDisabled(true);
    ui->sectionDistanceRotationCutBodyLineEdit->setDisabled(true);
    ui->xBegRotationCutBodyLineEdit->setDisabled(true);
    ui->xEndRotationCutBodyLineEdit->setDisabled(true);
    ui->rotationCutBodySolverPushButton->setDisabled(true);
    ui->rotationCutBodyFreeMotionSolverPushButton->setDisabled(true);
    ui->rotationCutBodyLaunchSolverPushButton->setDisabled(true);
    ui->rotationCutBodyNearScreenPushButton->setDisabled(true);
    ui->variateRotationCutBodySolverPushButton->setDisabled(true);
}

void MainWindow::on_rotationBodyFreeMotionSolverPushButton_clicked()
{
    if(ui->fiRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по фи"));
        return;
    }

    if(ui->partRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено количество разбиений по частям"));
        return;
    }
    if(ui->xBegRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена начальная координата по Х"));
        return;
    }

    if (ui->deltaRotationBodyFragmLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота для подъема отрезков при разбиении"));
        return;
    }
    if (ui->epsilonRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введено значение радиуса вортона"));
        return;
    }
    if (ui->pointsRaisingRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота подъема точек для вычисления давления"));
        return;
    }

    if (ui->xEndRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена конечная координата по X"));
        return;
    }
    if (ui->sectionDistanceRotationBodyLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, tr("Ошибка"), tr("Не введена высота среза"));
        return;
    }

    FragmentationParameters fragPar;
    fragPar.rotationBodyFiFragNum=ui->fiRotationBodyLineEdit->text().toInt();
    fragPar.rotationBodyPartFragNum=ui->partRotationBodyLineEdit->text().toInt();
    fragPar.rotationBodyXBeg=ui->xBegRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodyXEnd=ui->xEndRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionDistance=ui->sectionDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.rotationBodySectionEndDistance=ui->sectionEndDistanceRotationBodyLineEdit->text().toDouble();
    fragPar.vortonsRad=ui->epsilonRotationBodyLineEdit->text().toDouble();
    fragPar.delta=ui->deltaRotationBodyFragmLineEdit->text().toDouble();
    fragPar.pointsRaising=ui->pointsRaisingRotationBodyLineEdit->text().toDouble();
    fragPar.formingDiameter=ui->formingRBDiameterLineEdit->text().toDouble();
    fragPar.formingLengthSectorOne=ui->formingRBSectorOneLength->text().toDouble();
    fragPar.formingLengthSectorTwo=ui->formingRBSectorTwoLength->text().toDouble();

    fragPar.rotationBodyFormingType=ui->comboBox->currentIndex();
    switch (fragPar.rotationBodyFormingType)
    {
    case 0:
    {
        fragPar.formingDiameter=ui->formingRBDiameterLineEdit->text().toDouble();
        fragPar.formingLengthSectorTwo=ui->formingRBSectorTwoLength->text().toDouble();
        fragPar.formingLengthSectorOne=ui->formingRBSectorOneLength->text().toDouble();
        break;
    }
    case 1:
    {
        fragPar.formingDiameter=ui->formingConeRBDiameterLineEdit->text().toDouble();
        fragPar.formingAngle=ui->formingConeRBAngleLineEdit->text().toDouble();
        fragPar.formingLengthSectorOne=ui->formingConeRBLengthLineEdit->text().toDouble();
        break;
    }
    case 2:
    {
       fragPar.formingDiameter=ui->formingEllipsoidRBDiameterLineEdit->text().toDouble();
       fragPar.formingLengthSectorOne=ui->formingEllipsoidRBLengthLineEdit->text().toDouble();
       break;
    }
    }

    SolverParameters solvPar=settings->getSolverParameters();
    FreeMotionParameters freeMotionPar=settings->getFreeMotionParameters();

    *solver=Solver(solvPar,freeMotionPar);
    if (ui->acceleratedMotionAction->isChecked())
        solver->setMotionType(MotionType::ACCELERATED);
    else
        solver->setMotionType(MotionType::NOACCELERATE);
    QFuture<void> rotationBodyFuture=QtConcurrent::run(solver,&Solver::rotationBodyFreeMotionSolver, fragPar);
    solving=true;
    ui->pointsRaisingRotationBodyLineEdit->setDisabled(true);
    ui->epsilonRotationBodyLineEdit->setDisabled(true);
    ui->deltaRotationBodyFragmLineEdit->setDisabled(true);
    ui->partRotationBodyLineEdit->setDisabled(true);
    ui->fiRotationBodyLineEdit->setDisabled(true);
    ui->sectionDistanceRotationBodyLineEdit->setDisabled(true);
    ui->xBegRotationBodyLineEdit->setDisabled(true);
    ui->xEndRotationBodyLineEdit->setDisabled(true);
    ui->rotationBodySolverPushButton->setDisabled(true);
    ui->rotationBodySolverPushButton->setDisabled(true);
    ui->rotationBodyFreeMotionSolverPushButton->setDisabled(true);
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    ui->stackedWidget->setCurrentIndex(index);
}

void MainWindow::setReguliser(double reguliser)
{
    ui->reguliserLineEdit->setText(QString::number(reguliser));
}

void MainWindow::on_nextCommandLinkButton_clicked()
{
    if (currentNumber<allVortons.get()->size()-1)
    {
        currentNumber=ui->currentNumberLineEdit->text().toInt()+1;
        ui->currentNumberLineEdit->setText(QString::number(currentNumber));
        emit drawGUI(allVortons.get()->at(currentNumber),allFrames.get()->at(currentNumber));
    }
}

void MainWindow::on_previousCommandLinkButton_clicked()
{
    if (currentNumber>0)
    {
        currentNumber=ui->currentNumberLineEdit->text().toInt()-1;
        ui->currentNumberLineEdit->setText(QString::number(currentNumber));
        emit drawGUI(allVortons.get()->at(currentNumber),allFrames.get()->at(currentNumber));
    }
}

void MainWindow::on_currentNumberLineEdit_editingFinished()
{
    if (ui->currentNumberLineEdit->text().toInt()>=0 && ui->currentNumberLineEdit->text().toInt()<=allVortons.get()->size()-1)
    {
        currentNumber=ui->currentNumberLineEdit->text().toInt();
        emit drawGUI(allVortons.get()->at(currentNumber),allFrames.get()->at(currentNumber));
    }
}

void MainWindow::calcEpsilonLength(double panel)
{

    if (ui->autoParametersAction->isChecked())
    {
        ui->epsilonRotationBodyLineEdit->setText(QString::number(panel*0.5));
        ui->epsilonRotationCutBodyLineEdit->setText(QString::number(panel*0.5));
    }
}

void MainWindow::on_formRotationCutBodyComboBox_currentIndexChanged(int index)
{
    ui->stackedWidget_2->setCurrentIndex(index);
}

void MainWindow::on_ovalPushButton_clicked()
{
    QFuture<void> oval=QtConcurrent::run(solver,&Solver::ovalSolver);
}

void MainWindow::on_ringsPushButton_clicked()
{
    QFuture<void> ring=QtConcurrent::run(solver,&Solver::ringsSolver);
}
