#ifndef BODYFRAGMENTATION_H
#define BODYFRAGMENTATION_H
#include "fourframe.h"
#include "vector2d.h"
#include <QVariant>
enum BodyType {SPHERE, CYLINDER, ROTATIONBODY, ROTATIONBOTTOMCUT};

struct FragmentationParameters
{
    //для сферы
    int sphereFiFragNum;
    int sphereTetaFragNum;
    double sphereRad;


    //для цилиндра
    double cylinderDiameter;
    double cylinderHeight;
    int cylinderFiFragNum;
    int cylinderRadFragNum;
    int cylinderHeightFragNum;

    //для тела вращения
    int rotationBodyFiFragNum;
    int rotationBodyPartFragNum;
    double rotationBodyXBeg;
    double rotationBodyXEnd;
    double rotationBodySectionDistance;

    //для тела вращения со срезом
    int rotationBodyRFragNum;
    //общие
    double vortonsRad;
    double delta;
    double pointsRaising;
};

struct SphereParameters
{
    int fiFragNum;
    int tetaFragNum;
    double radius;
    double delta;
    double raise;
    double vortonsRad;
    void setData(const int i, const double value);
};

struct CylinderParameters
{
    int fiFragNum;
    int radFragNum;
    int heightFragNum;
    double diameter;
    double height;
    double delta;
    double raise;
    double vortonsRad;
    void setData(const int i, const double value);
};

struct RotationBodyParameters
{
    int fiFragNum;
    int partFragNum;
    double xBeg;
    double xEnd;
    double sectionDistance;
    double delta;
    double raise;
    double vortonsRad;
    void setData(const int i, const double value);
};

struct RotationCutBodyParameters
{
    int fiFragNum;
    int partFragNum;
    int rFragNum;
    double xBeg;
    double xEnd;
    double sectionDistance;
    double delta;
    double raise;
    double vortonsRad;
    void setData(const int i, const double value);
};

class BodyFragmentation: public QObject
{
    Q_OBJECT
private:
    QVector<std::shared_ptr<MultiFrame>> frames;
    QVector<Vector3D> controlPoints;
    QVector<Vector3D> normals;
    QVector<double> squares;
    QVector<Vector3D> controlPointsRaised;

    SphereParameters sphere;
    CylinderParameters cylinder;
    RotationBodyParameters rotationBody;
    RotationCutBodyParameters rotationBottomCutBody;
    bool launch;
public:
    BodyFragmentation(BodyType body, const FragmentationParameters& param, bool launch=false);
//    BodyFragmentation(const FragmentationParameters& param, const int i, const Vector3D &bodyVel, const double tau);
    void sphereFragmentation();
    void cylinderFragmentation();
    void rotationBodyFragmantation();
    void rotationCutBodyFragmantation();
    void rotationCutBodyLaunchFragmentation(const int i, const Vector3D &bodyVel, const double tau);
    void clearVectors();

    static double presetFunctionF(double x);
    static double presetDeriveFunctionF(double x);
    static double presetFunctionG(double x);
    static double presetDeriveFunctionG(double x);
    QVector<Vector3D> getControlPoints() const;
    QVector<Vector3D> getNormals() const;
    QVector<double> getSquares() const;
    QVector<Vector3D> getControlPointsRaised() const;
    QVector<std::shared_ptr<MultiFrame>> getFrames() const;
    static QPair<double, int> findClosest(const Vector3D point, const QVector<Vector3D>& controlPoints, const QVector<Vector3D>& normals);
    static int findClosetElementFromArray(const QVector<double> arr, const double point);
};

#endif // BODYFRAGMENTATION_H
