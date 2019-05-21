#include "vorton.h"

/*!
Создает элемент класса, где все поля проинициализрованы нулем
*/
VelBsym:: VelBsym ()
{
    Vel = Vector3D();
    B[0][0]=0.0;
    B[0][1]=0.0;
    B[0][2]=0.0;
    B[1][0]=0.0;
    B[1][1]=0.0;
    B[1][2]=0.0;
    B[2][0]=0.0;
    B[2][1]=0.0;
    B[2][2]=0.0;
}

/*!
Создает элемент класса с заданным значением скорости и нулевым значение тензора деформации
\param velocity заданное значение скорости
*/
VelBsym::VelBsym(Vector3D velocity)
{
    Vel = velocity;
    B[0][0]=0.0;
    B[0][1]=0.0;
    B[0][2]=0.0;
    B[1][0]=0.0;
    B[1][1]=0.0;
    B[1][2]=0.0;
    B[2][0]=0.0;
    B[2][1]=0.0;
    B[2][2]=0.0;
}

/*!
Создает полностью заполненный элемент класса
\param velocity заданное значение скорости
\param B00, B01, B02, B10, B11, B12, B20, B21, B22 значения элемнтов тензора деформации с соответсвующим номером
*/
VelBsym::VelBsym (Vector3D velocity, double B00, double B01, double B02, double B10, double B11, double B12, double B20, double B21, double B22)
{
    Vel = velocity;
    B[0][0]=B00;
    B[0][1]=B01;
    B[0][2]=B02;
    B[1][0]=B10;
    B[1][1]=B11;
    B[1][2]=B12;
    B[2][0]=B20;
    B[2][1]=B21;
    B[2][2]=B22;
}

/*!
Оператор копирования
\param C Новое значение вектора
*/
VelBsym VelBsym::operator = (VelBsym C)
{
    Vel = C.Vel;
    B[0][0]=C.B[0][0];
    B[0][1]=C.B[0][1];
    B[0][2]=C.B[0][2];
    B[1][0]=C.B[1][0];
    B[1][1]=C.B[1][1];
    B[1][2]=C.B[1][2];
    B[2][0]=C.B[2][0];
    B[2][1]=C.B[2][1];
    B[2][2]=C.B[2][2];
    return *this;
}

/*!
Оператор сложения
\param C слагаемое
\result Покомпонентное сложение структуры
*/
VelBsym VelBsym:: operator + (VelBsym C)
{
    VelBsym ans = VelBsym();
    ans.Vel = Vel+C.Vel;
    ans.B[0][0]=B[0][0]+C.B[0][0];
    ans.B[0][1]=B[0][1]+C.B[0][1];
    ans.B[0][2]=B[0][2]+C.B[0][2];
    ans.B[1][0]=B[1][0]+C.B[1][0];
    ans.B[1][1]=B[1][1]+C.B[1][1];
    ans.B[1][2]=B[1][2]+C.B[1][2];
    ans.B[2][0]=B[2][0]+C.B[2][0];
    ans.B[2][1]=B[2][1]+C.B[2][1];
    ans.B[2][2]=B[2][2]+C.B[2][2];
    return ans;
}

/*!
Оператор копирования в изначальный контейнер
\param C слагаемое
\result Покомпонентное сложение структуры
*/
VelBsym VelBsym::operator +=(VelBsym C)
{
    Vel+=C.Vel;
    B[0][0]=B[0][0]+C.B[0][0];
    B[0][1]=B[0][1]+C.B[0][1];
    B[0][2]=B[0][2]+C.B[0][2];
    B[1][0]=B[1][0]+C.B[1][0];
    B[1][1]=B[1][1]+C.B[1][1];
    B[1][2]=B[1][2]+C.B[1][2];
    B[2][0]=B[2][0]+C.B[2][0];
    B[2][1]=B[2][1]+C.B[2][1];
    B[2][2]=B[2][2]+C.B[2][2];
    return *this;
}

/*!
Создает пустой вортон-отрезок
*/
Vorton::Vorton()
{

}

/*!
Создает вортон-отрезок с заданными параметрами
\param _mid Значение центра вортон-отрезка
\param _tail Значение хвоста вортон-отрезка
\param _vorticity Значение завихренности вортон-отрезка
\param _radius Значение радиуса вортон-отрезка
*/
Vorton::Vorton(const Vector3D _mid, const Vector3D _tail, const double _vorticity, const double _radius)
{
    mid=_mid;
    tail=_tail;
    vorticity=_vorticity;
    radius=_radius;
}

//Vorton::Vorton(const Vorton &a)
//{
//    mid=a.mid;
//    tail=a.tail;
//    vorticity=a.vorticity;
//    radius=a.radius;
//    move=a.move;
//    elongation=a.elongation;
//}

/*!
� ассчитывает значение интенсивности вортон-отрезка в заданной точке
\param point Координаты точки расчета
\return Значение интенсивности
*/
Vector3D Vorton::q(const Vector3D &point) const
{
    Vector3D h0 = tail-mid;
    Vector3D s0 = point-mid;
    double R = (Vector3D::crossProduct(h0,s0)).length()/h0.length();
    if (qFuzzyIsNull(R))
        return Vector3D(0,0,0);
    if (R>=radius)
    {
        Vector3D s1 = s0-h0;
        Vector3D s2 = s0+h0;
        Vector3D av = Vector3D::crossProduct(h0,s0);
        double cv = Vector3D::dotProduct((s2/s2.length()-s1/s1.length()),h0)/av.lengthSquared();
        return (1/(4*M_PI))*cv*av;
    }
    else
    {
        Vector3D re = point-(radius/R-1)*(h0*Vector3D::dotProduct(s0,h0)/h0.lengthSquared()-s0);
        Vector3D ae = Vector3D::crossProduct(h0,(re-mid));
        double ce = Vector3D::dotProduct(((re-mid+h0).normalized()-(re-mid-h0).normalized()),h0)/ae.lengthSquared();
        return (1/(4*M_PI))*R/radius*ce*ae;
    }
}

/*!
� ассчитывает значение единичной интенсивности вортон-отрезка в заданной точке
\param point Координаты точки расчета
\return Значение единичной интенсивности
*/
Vector3D Vorton::qHelp(const Vector3D& point) const
{
    Vector3D  h0 = tail-mid;
    Vector3D  s0 = point-mid;
    Vector3D  s1 = s0-h0;
    Vector3D  s2 = s0+h0;
    Vector3D  av = Vector3D::crossProduct(h0,s0);
    double  cv = Vector3D::dotProduct((s2.normalized()-s1.normalized()),h0)/av.lengthSquared();
    return ((1/(4*M_PI))*cv*av);
}

/*!
� ассчитывает значение скорости вортон-отрезка в заданной точке
\param point Координаты точки расчета
\return Значение скорости
*/
Vector3D Vorton::velocity(const Vector3D &point) const
{
    return q(point)*vorticity;
}

/*!
� ассчитывает значение скорости и тензора деформации вортон-отрезка в заданной точке
\param point Координаты точки расчета
\return Значение скорости и тензора деформаций
*/
VelBsym Vorton::velAndBsym(const Vector3D& point) const
{
    Vector3D h0 = tail-mid;
    Vector3D s0 = point-mid;
    double R = (Vector3D::crossProduct(h0,s0)).length()/h0.length();
    if (qFuzzyIsNull(R))
    {
        VelBsym ans;
        ans.Vel = Vector3D (0,0,0);
        ans.B[0][0] = ans.B[0][1] =ans.B[0][2] =ans.B[1][0] = ans.B[1][1] = ans.B[1][2] =ans.B[2][0] =ans.B[2][1]=ans.B[2][2]=0;
        return ans;
    }
    if (R>=radius)
    {
        VelBsym ans;
        Vector3D s1 = s0-h0;
        Vector3D s2 = s0+h0;
        Vector3D av = Vector3D::crossProduct(h0,s0);
        double cv = Vector3D::dotProduct((s2.normalized()-s1.normalized()),h0)/av.lengthSquared();
        ans.Vel = (vorticity/(4.0*M_PI))*cv*av;
        for (int m=0; m<3; m++)
        {
            for (int n=0; n<3; n++)
            {
                double apn = levi(m,0,n)*h0.x()+levi(m,1,n)*h0.y()+levi(m,2,n)*h0.z();
                double cpn=((1.0/s2.length()-1.0/s1.length()+2.0*cv*Vector3D::dotProduct(s0,h0))*h0[n]
                            -(Vector3D::dotProduct((s2[n]*s2/pow(s2.length(),3)-s1[n]*s1/pow(s1.length(),3)+2*cv*s0[n]*h0),h0)))/av.lengthSquared();
                ans.B[m][n]=(vorticity/(4*M_PI))*(cpn*av[m]+cv*apn);
            }
        }
        double q = 0.5*(ans.B[0][1]+ans.B[1][0]);
        double w = 0.5*(ans.B[0][2]+ans.B[2][0]);
        double z  = 0.5*(ans.B[1][2]+ans.B[2][1]);
        ans.B[0][1]=ans.B[1][0]=q;
        ans.B[0][2]=ans.B[2][0]=w;
        ans.B[1][2]=ans.B[2][1]=z;
        return ans;
    }
    else
    {
        VelBsym ans;
        Vector3D re = point-(radius/R-1.0)*(h0*Vector3D::dotProduct(s0,h0)/h0.lengthSquared()-s0);
        Vector3D s0e = re-mid;
        Vector3D  s1e = s0e-h0;
        Vector3D s2e = s0e+h0;
        Vector3D ae = Vector3D::crossProduct(h0,s0e);
        double ce = Vector3D::dotProduct((s2e.normalized()-s1e.normalized()),h0)/ae.lengthSquared();
        ans.Vel=vorticity/(4*M_PI)*(R/radius)*ce*ae;
        for (int m=0; m<3; m++)
            for (int n=0; n<3; n++)
            {
                double apn = levi(m,0,n)*h0.x()+levi(m,1,n)*h0.y()+levi(m,2,n)*h0.z();
                double cpn=((1.0/s2e.length()-1.0/s1e.length()+2*ce*Vector3D::dotProduct(s0e,h0))*h0[n]
                            -(Vector3D::dotProduct((s2e[n]*s2e/pow(s2e.length(),3)-s1e[n]*s1e/pow(s1e.length(),3)+2.0*ce*s0e[n]*h0),h0)))/Vector3D::dotProduct(ae,ae);
                ans.B[m][n]=(R/radius*(vorticity/(4.0*M_PI))*(cpn*ae[m]+ce*apn));
            }
        double q = 0.5*(ans.B[0][1]+ans.B[1][0]);
        double w = 0.5*(ans.B[0][2]+ans.B[2][0]);
        double z = 0.5*(ans.B[1][2]+ans.B[2][1]);
        ans.B[0][1]=ans.B[1][0]=q;
        ans.B[0][2]=ans.B[2][0]=w;
        ans.B[1][2]=ans.B[2][1]=z;
        return ans;
    }
}

VelBsym Vorton::velAndBsymGauss3(const Vector3D &point, const Vector3D &deltar) const
{
    VelBsym ans;
    VelBsym ans1;
    VelBsym ans2;
    ans=velAndBsym(point);
    ans1=velAndBsym(point+sqrt(0.6)*deltar);
    ans2=velAndBsym(point-sqrt(0.6)*deltar);
    ans.Vel=0.5*(8.0/9.0*ans.Vel+5.0/9.0*(ans1.Vel+ans2.Vel));
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            ans.B[i][j]=0.5*(8.0/9.0*ans.B[i][j]+5.0/9.0*(ans1.B[i][j]+ans2.B[i][j])); 
        }
    }
    return ans;
}

/*!
� ассчитывает значение тензора Леви-Чивитты
\param firstComponent Значение первой компоненты тензора Леви-Чивитты
\param secondComponent Значение второй компоненты тензора Леви-Чивитты
\param thirdComponent Значение третьей компоненты тензора Леви-Чивитты
\return Значение тензора
*/
double Vorton::levi(int firstComponent, int secondComponent, int thirdComponent)
{
    if ((firstComponent==secondComponent)||(firstComponent==thirdComponent)||(secondComponent==thirdComponent))
        return 0.0;
    if ((firstComponent==0)&&(secondComponent==1)&&(thirdComponent==2))
        return 1.0;
    if ((firstComponent==0)&&(secondComponent==2)&&(thirdComponent==1))
        return -1.0;
    if ((firstComponent==1)&&(secondComponent==2)&&(thirdComponent==0))
        return 1.0;
    if ((firstComponent==1)&&(secondComponent==0)&&(thirdComponent==2))
        return -1.0;
    if ((firstComponent==2)&&(secondComponent==1)&&(thirdComponent==0))
        return -1.0;
    if ((firstComponent==2)&&(secondComponent==0)&&(thirdComponent==1))
        return 1.0;
    QMessageBox::critical(new QWidget(), "Ошибка", "Неверные параметры для тензора Леви-Чивитты");
    exit(1);
}


//Vector3D Vorton::rotated(const Vector3D &vec, const Vector3D &axis, const double theta)
//{
//    Vector3D newaxis = axis/axis.length();
//    double a = cos(theta/2.0);
//    Vector3D secondary = -newaxis*sin(theta/2.0);
//    double b = secondary.x();
//    double c = secondary.y();
//    double d = secondary.z();
//    double aa = a*a;
//    double bb = b*b;
//    double cc = c*c;
//    double dd = d*d;
//    double bc = b*c;
//    double ad = a*d;
//    double ac = a*c;
//    double ab = a*b;
//    double bd = b*d;
//    double cd = c*d;
//    return Vector3D((aa+bb-cc-dd)*vec.x()+2*(bc+ad)*vec.y()+2*(bd-ac)*vec.z(),2*(bc-ad)*vec.x()+(aa+cc-bb-dd)*vec.y()+2*(cd+ab)*vec.z(), 2*(bd+ac)*vec.x()+2*(cd-ab)*vec.y()+(aa+dd-bb-cc)*vec.z());
//}

/*!
Поворот вортон-отрезка вокруг нормали
\param normal Координата нормали
*/
void Vorton::rotateAroundNormal(const Vector3D &normal)
{
    double len=(tail-mid).length();
    Vector3D rTilda=tail-Vector3D::dotProduct(tail-mid,normal)*normal;
    if ((rTilda-mid)!=Vector3D())
    {
        tail=(rTilda-mid).normalized()*len+mid;
    }
    else
    {
        Vector3D direction;
        if (!(qFuzzyIsNull(normal.z())&&qFuzzyIsNull(normal.y())))
            direction=Vector3D(0.0,normal.z(),-normal.y());
        else
            direction=Vector3D(-normal.z(),0.0,normal.x());
        tail=mid+direction.normalized()*len;
    }
}

/*!
Задает значение центра вортон-отрезка
\param _mid Координата центра
*/
void Vorton::setMid(const Vector3D& _mid)
{
    mid=_mid;
}

/*!
Задает значение хвоста вортон-отрезка
\param _tail Координата хвоста
*/
void Vorton::setTail(const Vector3D& _tail)
{
    tail=_tail;
}

/*!
Задает значение завихренности вортон-отрезка
\param _vorticity Значение завихренности
*/
void Vorton::setVorticity(const double _vorticity)
{
    vorticity=_vorticity;
}

/*!
Задает значение радиуса вортон-отрезка
\param _radius Значение радиуса
*/
void Vorton::setRadius(const double _radius)
{
    radius=_radius;
}

/*!
Задает значение удлинения вортон-отрезка
\param _elongation Значение удлинения
*/
void Vorton::setElongation(const Vector3D& _elongation)
{
    elongation=_elongation;
}

/*!
Задает значение перемещения вортон-отрезка
\param _move Значение перемещения
*/
void Vorton::setMove(const Vector3D& _move)
{
    move=_move;
}

/*!
Возвращает значение центра вортон-отрезка
\return Значение центра
*/
Vector3D Vorton::getMid() const
{
    return mid;
}

/*!
Возвращает значение хвоста вортон-отрезка
\return Значение хвоста
*/
Vector3D Vorton::getTail() const
{
    return tail;
}

/*!
Возвращает значение завихренности вортон-отрезка
\return Значение завихренности
*/
double Vorton::getVorticity() const
{
    return vorticity;
}

/*!
Возвращает значение радиуса вортон-отрезка
\return Значение радиуса
*/
double Vorton::getRadius() const
{
    return radius;
}

/*!
Возвращает значение удлинения вортон-отрезка
\return Значение удлинения
*/
Vector3D Vorton::getElongation() const
{
    return elongation;
}

/*!
Возвращает значение перемещения вортон-отрезка
\return Значение перемещения
*/
Vector3D Vorton::getMove() const
{
    return move;
}

/*!
� азворачивает вортон-отрезок отнносительно центра
*/
void Vorton::turn ()
{
    tail = 2*mid-tail;
    vorticity = -vorticity;
}

/*!
Переносит вортон-отрезок на заданный трехмерный вектор
\param translation Вектор направления
*/
void Vorton::translate(const Vector3D& translation)
{
    mid+=translation;
    tail+=translation;
}

void Vorton::translateWithMove(const Vector3D &translation)
{
    mid+=translation;
    tail+=translation;
    move+=translation;
}

/*!
Оператор копирования
\param vort2 Новое значение вектора
*/
Vorton Vorton::operator =(const Vorton &vort2)
{
        mid = vort2.mid;
        tail = vort2.tail;
        radius  =vort2.radius;
        vorticity = vort2.vorticity;
        move = vort2.move;
        elongation = vort2.elongation;
        return Vorton(*this);
}
