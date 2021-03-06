﻿#ifndef BODYFRAGMENTATION_H
#define BODYFRAGMENTATION_H
#include "fourframe.h"
#include "vector2d.h"
#include <QVariant>

/** \file bodyfragmentation.h 
    \brief Заголовочный файл для описания классов, структур и перечислений для разбиения исходного тела на рамки
*/

/**
 * Типы расчитываемых тел.
*/
enum BodyType {
    SPHERE, ///<Сфера
    CYLINDER, ///<Цилиндр
    ROTATIONBODY, ///<Тело вращения
    ROTATIONBOTTOMCUT, ///<Тело вращения со срезом дна
    ROTATIONTWOBOTTOM
};


enum FormingTypeRBC {ELLIPSOID_CYLINDER, ELLIPSOID_CONE, ELLIPSOID_CYLINDER_CONE};

/*!
    \defgroup fragmentationParameters Параметры разбиения
    \brief Структура, хранящая данные, необходимые для разбиения
*/
///@{
/*!
    \brief Структура, хранящая параметры разбиения
*/
struct FragmentationParameters
{

    /*!
        \defgroup sphereParameters Параметры сферы
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся параметры, необходимые для разбиения сферы.
    */
    ///@{
    int sphereFiFragNum=0; ///<Количество разбиений по fi
    int sphereTetaFragNum=0; ///<Количество разбиений по teta
    double sphereRad=0.0; ///<Радиус сферы
    ///@}

    /*!
        \defgroup cylinderParameters Параметры цилиндра
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся параметры, необходимые для разбиения цилиндра.
    */
    ///@{
    double cylinderDiameter=0.0; ///<Диаметр цилиндра
    double cylinderHeight=0.0; ///<Высота цилиндра
    int cylinderFiFragNum=0; ///<Количество разбиений по fi
    int cylinderRadFragNum=0; ///<Количество разбиений по радиусу
    int cylinderHeightFragNum=0; ///<Количество разбиений по высоте
    ///@}

    /*!
        \defgroup rotationBodyParameters Параметры тела вращения
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся параметры, необходимые для разбиения тела вращения.
    */
    ///@{
    int rotationBodyFiFragNum=0; ///<Количество разбиений по fi
    int rotationBodyPartFragNum=0; ///<Количество разбиений по длине
    double rotationBodyXBeg=0.0; ///<Координата начала тела по X
    double rotationBodyXEnd=0.0; ///<Координата конца тела по Y
    double rotationBodySectionDistance=0.0; ///< Величина среза
    double rotationBodySectionEndDistance=0.0;
    int rotationBodyFormingType=0;
    ///@}

    /*!
        \defgroup rotationBottomCutParameters Дополнительные параметры тела вращения со срезом
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся дополнительные параметры, необходимые для разбиения тела вращения.
    */
    ///@{
    int rotationBodyRFragNum=0; ///<Количество разбиений по радиусу

    int rotationBodyR2FragNum=0;


    FormingTypeRBC rotationBodyRBCFormingType;
    ///@}

    double formingDiameter=0.0;
    double formingLengthSectorOne=0.0;
    double formingLengthSectorTwo=0.0;
    double formingTailDiameter=0.0;
    double formingAngle=0.0;


    double formingEllipsoidDiameter=0.0;

    double formingEllisoidLength=0.0;
    double formingConeLength=0.0;
    double formingFullLength=0.0;


    double formingRBB_R1=0.0;
    double formingRBB_R2=0.0;
    double formingRBB_length=0.0;
    /*!
        \defgroup commonParameters Общие параметры
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся Общие параметры, необходимые для разбиения тела.
    */
    ///@{
    double vortonsRad=0.0; ///<Радиус вортона
    double delta=0.0; ///<Подъем рамок над телом
    double pointsRaising=0.0; ///<Подъем контрольных точек для подсчета давления
    ///@}



};
///@}

/*!
    \brief Структура для хранения параметров сферы
*/
struct SphereParameters
{
    int fiFragNum; ///<Количество разбиений по fi
    int tetaFragNum; ///<Количество разбиений по teta
    double radius; ///<Радиус сферы
    double delta; ///<Подъем рамок над телом
    double raise; ///<Подъем контрольных точек для подсчета давления
    double vortonsRad; ///<Радиус вортона
    void setData(const int i, const double value);
};

/*!
    \brief Структура для хранения параметров цилиндра
*/
struct CylinderParameters
{
    int fiFragNum; ///<Количество разбиений по fi
    int radFragNum; ///<Количество разбиений по радиусу
    int heightFragNum; ///<Количество разбиений по высоте
    double diameter; ///<Диаметр цилиндра
    double height; ///<Высота цилиндра
    double delta; ///<Подъем рамок над телом
    double raise; ///<Подъем контрольных точек для подсчета давления
    double vortonsRad;  ///<Радиус вортона
    void setData(const int i, const double value);
};

/*!
    \brief Структура для хранения параметров тела вращения
*/
struct RotationBodyParameters
{
    int fiFragNum;  ///<Количество разбиений по fi
    int partFragNum;  ///<Количество разбиений по длине
    double xBeg;  ///<Координата начала тела по X
    double xEnd;  ///<Координата конца тела по Y
    double sectionDistance;  ///< Величина среза
    double sectionEndDistance;
    double delta;  ///<Подъем рамок над телом
    double raise;  ///<Подъем контрольных точек для подсчета давления
    double vortonsRad;  ///<Радиус вортона
    void setData(const int i, const double value);
};

/*!
    \brief Структура для хранения параметров тела вращения со срезом
*/
struct RotationCutBodyParameters
{
    int fiFragNum;  ///<Количество разбиений по fi
    int partFragNum;  ///<Количество разбиений по длине
    int rFragNum; ///<Количество разбиений по радиусу
    double xBeg;  ///<Координата начала тела по X
    double xEnd;  ///<Координата конца тела по Y
    double sectionDistance;  ///< Величина среза
    double sectionEndDistance;
    double delta;  ///<Подъем рамок над телом
    double raise;  ///<Подъем контрольных точек для подсчета давления
    double vortonsRad; ///<Радиус вортона
    void setData(const int i, const double value);
};

struct RotationBodyWithTwoBottomsParameters
{
    int fiFragNum;  ///<Количество разбиений по fi
    int partFragNum;  ///<Количество разбиений по длине
    int r1FragNum; ///<Количество разбиений по радиусу
    int r2FragNum;
    double xBeg;  ///<Координата начала тела по X
    double xEnd;  ///<Координата конца тела по Y

    double delta;  ///<Подъем рамок над телом
    double raise;  ///<Подъем контрольных точек для подсчета давления
    double vortonsRad; ///<Радиус вортона
    void setData(const int i, const double value);
};

struct FormingParameters
{
    double diameter;
    double tailDiameter;
    double sectorOneLength;
    double sectorTwoLength;
    double angle;

    int typeNum;
};


struct FormingParametersRBC
{
    double ellipsoidDiameter;
    double tailDiameter;
    double ellipsoidLength;
    double coneLength;
    double fullLength;

    FormingTypeRBC type;
};

struct FormingParametersRBB
{
    double radius1;
    double radius2;
    double length;
};

/*!
    \brief Класс, описывающие разбиение тела

    Класс разбивает тело на рамки, рассчитывает контрольные точки, нормали, контрольные точки для подсчета давления и площади рамок.
*/
class BodyFragmentation: public QObject
{
    Q_OBJECT
private:
    QVector<std::shared_ptr<MultiFrame>> frames; ///<Вектор рамок, составленных из вортон-отрезков
    QVector<Vector3D> controlPoints; ///<Вектор котнрольных точек
    QVector<Vector3D> normals; ///<Вектор нормалей
    QVector<double> squares; ///<Векторй площадей рамок
    QVector<Vector3D> controlPointsRaised; ///<Вектор контрольных точек для подсчета давления

    SphereParameters sphere; ///<Параметры разбиения сферы
    CylinderParameters cylinder; ///<Параметры разбиения цилиндра
    RotationBodyParameters rotationBody; ///<Параметры разбиения тела вращения
    RotationCutBodyParameters rotationBottomCutBody; ///<Параметры разбиения тела вращения со срезом дна
    RotationBodyWithTwoBottomsParameters rotationBodyTwoBot;
    FormingParameters forming;
    QPair<int,int> streamLinesSize;
    FormingParametersRBC formingRBC;
    FormingParametersRBB formingRBB;
    BodyType exisetingType;
    QVector<QVector<Vector3D>> graphNodesX;
    QVector<QVector<Vector3D>> graphNodesY;
    QVector<QVector<Vector3D>> graphNodesZ;
    //bool launch;
public:
    BodyFragmentation(BodyType body, const FragmentationParameters& param, bool launch=false);
    BodyFragmentation(const BodyFragmentation& frag);
    BodyFragmentation(BodyType body, const FragmentationParameters& param, int a, bool launch=false);
//    BodyFragmentation(const FragmentationParameters& param, const int i, const Vector3D &bodyVel, const double tau);
    void sphereFragmentation();
    void cylinderFragmentation();
    void calculateBoundaries(Vector3D& bodyNose, Vector3D& center, double& xend, Vector3D bodyVel, double tau, double &fullLength, bool underwater);


     QPair<int,int> getStreamLinesSizes();
    void rotationBodyFragmantation();
    void rotationCutBodyFragmantation();
    void rotationCutBodyFragmantationWithConcentration();
    void rotationCutBodyLaunchFragmentation(const int i, const Vector3D &bodyVel, const double tau, const double fullLength);
    void rotationBodyWithTwoBottoms();
    void clearVectors();
    void prepareGraphsX0(QVector<std::shared_ptr<MultiFrame>>& xFrames,  FormingParameters pars);
    void prepareGraphsY0(QVector<std::shared_ptr<MultiFrame>>& yFrames,  FormingParameters pars);
    void prepareGraphsZ0(QVector<std::shared_ptr<MultiFrame>>& zFrames, FormingParameters pars);
    FormingParameters getForming();
    FormingParametersRBC getFormingRBC();
    static double presetFunctionF(double x, FormingParameters parameters);
    static double presetDeriveFunctionF(double x, FormingParameters parameters);
    static double presetFunctionG(double x, FormingParametersRBC parameters);
    static double presetFunctionRBB(double x, FormingParametersRBB parameters);
    static double presetDeriveFunctionRBB(double x, FormingParametersRBB parameters);
    //static double presetFunctionG(double x, double xBeg, FormingParameters parameters);
    static double presetDeriveFunctionG(double x, FormingParametersRBC parameters);
    QVector<Vector3D> getControlPoints() const;
    QVector<Vector3D> getNormals() const;
    QVector<double> getSquares() const;
    QVector<Vector3D> getControlPointsRaised() const;
    QVector<std::shared_ptr<MultiFrame>> getFrames() const;
    static QPair<double, int> findClosest(const Vector3D point, const QVector<Vector3D>& controlPoints, const QVector<Vector3D>& normals);
    static QPair<double, int> findClosestTriangle(const Vector3D point, const QVector<std::shared_ptr<MultiFrame> > &frames, const QVector<Vector3D>& normals);
    static int findClosetElementFromArray(const QVector<double> arr, const double point);

    QVector<QVector<Vector3D>> getGraphNodesX();
    QVector<QVector<Vector3D>> getGraphNodesY();
    QVector<QVector<Vector3D>> getGraphNodesZ();
    void clearGraphNodes();
    static bool coDirectionallyCheck(const Vector3D a, const Vector3D b);
};

#endif // BODYFRAGMENTATION_H
