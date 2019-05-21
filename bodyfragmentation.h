#ifndef BODYFRAGMENTATION_H
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
    ROTATIONBOTTOMCUT ///<Тело вращения со срезом дна
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
    int sphereFiFragNum; ///<Количество разбиений по fi
    int sphereTetaFragNum; ///<Количество разбиений по teta
    double sphereRad; ///<� адиус сферы
    ///@}

    /*!
        \defgroup cylinderParameters Параметры цилиндра
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся параметры, необходимые для разбиения цилиндра.
    */
    ///@{
    double cylinderDiameter; ///<Диаметр цилиндра
    double cylinderHeight; ///<Высота цилиндра
    int cylinderFiFragNum; ///<Количество разбиений по fi
    int cylinderRadFragNum; ///<Количество разбиений по радиусу
    int cylinderHeightFragNum; ///<Количество разбиений по высоте
    ///@}

    /*!
        \defgroup rotationBodyParameters Параметры тела вращения
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся параметры, необходимые для разбиения тела вращения.
    */
    ///@{
    int rotationBodyFiFragNum; ///<Количество разбиений по fi
    int rotationBodyPartFragNum; ///<Количество разбиений по длине
    double rotationBodyXBeg; ///<Координата начала тела по X
    double rotationBodyXEnd; ///<Координата конца тела по Y
    double rotationBodySectionDistance; ///< Величина среза
    double rotationBodySectionEndDistance;
    int rotationBodyFormingType;
    ///@}

    /*!
        \defgroup rotationBottomCutParameters Дополнительные параметры тела вращения со срезом
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся дополнительные параметры, необходимые для разбиения тела вращения.
    */
    ///@{
    int rotationBodyRFragNum; ///<Количество разбиений по радиусу
    FormingTypeRBC rotationBodyRBCFormingType;
    ///@}

    double formingDiameter;
    double formingLengthSectorOne;
    double formingLengthSectorTwo;
    double formingTailDiameter;
    double formingAngle;


    double formingEllipsoidDiameter;

    double formingEllisoidLength;
    double formingConeLength;
    double formingFullLength;

    /*!
        \defgroup commonParameters Общие параметры
        \ingroup fragmentationParameters
        \brief В данном модуле хранятся Общие параметры, необходимые для разбиения тела.
    */
    ///@{
    double vortonsRad; ///<� адиус вортона
    double delta; ///<Подъем рамок над телом
    double pointsRaising; ///<Подъем контрольных точек для подсчета давления
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
    double radius; ///<� адиус сферы
    double delta; ///<Подъем рамок над телом
    double raise; ///<Подъем контрольных точек для подсчета давления
    double vortonsRad; ///<� адиус вортона
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
    double vortonsRad;  ///<� адиус вортона
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
    double vortonsRad;  ///<� адиус вортона
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
    double vortonsRad; ///<� адиус вортона
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
    FormingParameters forming;
    QPair<int,int> streamLinesSize;
    FormingParametersRBC formingRBC;
    //bool launch;
public:
    BodyFragmentation(BodyType body, const FragmentationParameters& param, bool launch=false);
//    BodyFragmentation(const FragmentationParameters& param, const int i, const Vector3D &bodyVel, const double tau);
    void sphereFragmentation();
    void cylinderFragmentation();
    QPair<int,int> getStreamLinesSizes();
    void rotationBodyFragmantation();
    void rotationCutBodyFragmantation();
    void rotationCutBodyLaunchFragmentation(const int i, const Vector3D &bodyVel, const double tau, const double fullLength);
    void clearVectors();
    void prepareGraphsX0(QVector<std::shared_ptr<MultiFrame>>& xFrames,  FormingParameters pars);
    void prepareGraphsY0(QVector<std::shared_ptr<MultiFrame>>& yFrames,  FormingParameters pars);
    void prepareGraphsZ0(QVector<std::shared_ptr<MultiFrame>>& zFrames, FormingParameters pars);
    FormingParameters getForming();
    FormingParametersRBC getFormingRBC();
    static double presetFunctionF(double x, FormingParameters parameters);
    static double presetDeriveFunctionF(double x, FormingParameters parameters);
    static double presetFunctionG(double x, FormingParametersRBC parameters);
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

    static bool coDirectionallyCheck(const Vector3D a, const Vector3D b);
};

#endif // BODYFRAGMENTATION_H
