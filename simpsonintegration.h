#ifndef SIMPSONINTEGRATION_H
#define SIMPSONINTEGRATION_H
#include "bodyfragmentation.h"
#include <eigen-eigen-5a0156e40feb/Eigen/Dense>
#include <eigen-eigen-5a0156e40feb/Eigen/LU>

struct IntegrationResults
{
    double mass;
    Vector3D massCenter;
    Eigen::Matrix3d inertiaTensor;
};
 
class SimpsonIntegration
{
private:
    double length;
    double bodyDensity;
    bool rotationBottomCutFlag;
    FormingParameters parametersRB;
    FormingParametersRBC parametersRBC;
    IntegrationResults results;

    void massCalculator();
    void massCenterCalculator();
    void inertiaTensorCalculator();
    double massCenterIntegralRB(int n);
    double massCenterIntegralRBC(int n);
    double massCenterFuncRB(double x);
    double massCenterFuncRBC(double x);
    double massIntegralRB(int n);
    double massIntegralRBC(int n);
    double massFuncRB(double x);
    double massFuncRBC(double x);


    double inertiaZZIntegralRB(int n);
    double inertiaZZIntegralRBC(int n);
    double inertiaZZFuncRB(double x);
    double inertiaZZFuncRBC(double x);

    double inertiaXXIntegralRB(int n);
    double inertiaXXIntegralRBC(int n);
    double inertiaXXFuncRB(double x);
    double inertiaXXFuncRBC(double x);
public:
    SimpsonIntegration(double _length, double _bodyDensity, FormingParameters parameters);
    SimpsonIntegration(double _length, double _bodyDensity, FormingParametersRBC parameters);
    IntegrationResults integralsCalculator();
};

#endif // SIMPSONINTEGRATION_H
