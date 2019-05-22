#include "simpsonintegration.h"

SimpsonIntegration::SimpsonIntegration(double _length, double _bodyDensity, FormingParameters parameters)
{
    length=_length;
    bodyDensity=_bodyDensity;
    parametersRB=parameters;
    rotationBottomCutFlag=false;
}

SimpsonIntegration::SimpsonIntegration(double _length, double _bodyDensity, FormingParametersRBC parameters)
{
    length=_length;
    bodyDensity=_bodyDensity;
    parametersRBC=parameters;
    rotationBottomCutFlag=true;
}

IntegrationResults SimpsonIntegration::integralsCalculator()
{
    massCalculator();
    massCenterCalculator();
    inertiaTensorCalculator();
    return results;
}
 
void SimpsonIntegration::massCalculator()
{
    if(rotationBottomCutFlag)
    {
        int n=1;
        double simpson1=massIntegralRBC(n);
        double simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=massIntegralRBC(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        results.mass=simpson1;
    }
    else
    {
        int n=1;
        double simpson1=massIntegralRB(n);
        double simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=massIntegralRB(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        results.mass=simpson1;
    }
}

void SimpsonIntegration::massCenterCalculator()
{
    if(rotationBottomCutFlag)
    {
        int n=1;
        double simpson1=massCenterIntegralRBC(n);
        double simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=massCenterIntegralRBC(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        results.massCenter=Vector3D(simpson1/results.mass,0.0,0.0);
    }
    else
    {
        int n=1;
        double simpson1=massCenterIntegralRB(n);
        double simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=massCenterIntegralRB(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        results.massCenter=Vector3D(simpson1/results.mass,0.0,0.0);
    }
}

void SimpsonIntegration::inertiaTensorCalculator()
{
    double inertiaZZ;
    double inertiaXX;
    if(rotationBottomCutFlag)
    {

        int n=1;
        double simpson1=inertiaZZIntegralRBC(n);
        double simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=inertiaZZIntegralRBC(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        inertiaZZ=simpson1;

        n=1;
        simpson1=inertiaXXIntegralRBC(n);
        simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=inertiaXXIntegralRBC(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        inertiaXX=simpson1;

        results.inertiaTensor<<inertiaXX,0.0,0.0,0.0,inertiaXX,0.0,0.0,0.0,inertiaZZ;

    }
    else
    {
        int n=1;
        double simpson1=inertiaZZIntegralRB(n);
        double simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=inertiaZZIntegralRB(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        inertiaZZ=simpson1;

        n=1;
        simpson1=inertiaXXIntegralRB(n);
        simpson=0.0;
        do {
            simpson=simpson1;
            n=2*n;
            simpson1=inertiaXXIntegralRB(n);
        }
        while (fabs(simpson1-simpson)>0.0001);
        inertiaXX=simpson1;

        results.inertiaTensor<<inertiaXX,0.0,0.0,0.0,inertiaXX,0.0,0.0,0.0,inertiaZZ;
    }
}

double SimpsonIntegration::massCenterIntegralRB(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=massCenterFuncRB(h*i);
        f2+=massCenterFuncRB(h*(i+1));
    }
    return h/3*(massCenterFuncRB(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::massCenterIntegralRBC(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=massCenterFuncRBC(h*i);
        f2+=massCenterFuncRBC(h*(i+1));
    }
    return h/3*(massCenterFuncRBC(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::massCenterFuncRB(double x)
{
    return bodyDensity*x*M_PI*pow(BodyFragmentation::presetFunctionF(x,parametersRB),2);
}

double SimpsonIntegration::massCenterFuncRBC(double x)
{
    return bodyDensity*x*M_PI*pow(BodyFragmentation::presetFunctionG(x,parametersRBC),2);
}


double SimpsonIntegration::massIntegralRB(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=massFuncRB(h*i);
        f2+=massFuncRB(h*(i+1));
    }
    return h/3*(massFuncRB(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::massIntegralRBC(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=massFuncRBC(h*i);
        f2+=massFuncRBC(h*(i+1));
    }
    return h/3*(massFuncRBC(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::massFuncRB(double x)
{
    return bodyDensity*M_PI*pow(BodyFragmentation::presetFunctionF(x,parametersRB),2);
}

double SimpsonIntegration::massFuncRBC(double x)
{
    return bodyDensity*M_PI*pow(BodyFragmentation::presetFunctionG(x,parametersRBC),2);
}

double SimpsonIntegration::inertiaZZIntegralRB(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=inertiaZZFuncRB(h*i);
        f2+=inertiaZZFuncRB(h*(i+1));
    }
    return h/3*(inertiaZZFuncRB(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::inertiaZZIntegralRBC(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=inertiaZZFuncRBC(h*i);
        f2+=inertiaZZFuncRBC(h*(i+1));
    }
    return h/3*(inertiaZZFuncRBC(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::inertiaZZFuncRB(double x)
{
    return bodyDensity*M_PI*pow(BodyFragmentation::presetFunctionF(x,parametersRB),4)*0.5;
}

double SimpsonIntegration::inertiaZZFuncRBC(double x)
{
    return bodyDensity*M_PI*pow(BodyFragmentation::presetFunctionG(x,parametersRBC),4)*0.5;
}

double SimpsonIntegration::inertiaXXIntegralRB(int n)
{
    double h=(length-0.0)/n;
    double f1=0.0;
    double f2=0.0;
    for (int i=1; i<n; i+=2)
    {
        f1+=inertiaXXFuncRB(h*i);
        f2+=inertiaXXFuncRB(h*(i+1));
    }
    return h/3*(inertiaXXFuncRB(0.0)+4*f1+2*f2);
}

double SimpsonIntegration::inertiaXXIntegralRBC(int n)
{
    double h=(length-0.0)/n;
        double f1=0.0;
        double f2=0.0;
        for (int i=1; i<n; i+=2)
        {
            f1+=inertiaXXFuncRBC(h*i);
            f2+=inertiaXXFuncRBC(h*(i+1));
        }
        return h/3*(inertiaXXFuncRBC(0.0)+4*f1+2*f2);
}


double SimpsonIntegration::inertiaXXFuncRB(double x)
{
    return bodyDensity*(pow(BodyFragmentation::presetFunctionF(x,parametersRB),2)*0.25+pow(x-results.massCenter.x(),2))*M_PI*pow(BodyFragmentation::presetFunctionF(x,parametersRB),2);
}

double SimpsonIntegration::inertiaXXFuncRBC(double x)
{
     return bodyDensity*(pow(BodyFragmentation::presetFunctionG(x,parametersRBC),2)*0.25+pow(x-results.massCenter.x(),2))*M_PI*pow(BodyFragmentation::presetFunctionG(x,parametersRBC),2);
}
