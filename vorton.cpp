#include "vorton.h"

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

Vorton::Vorton()
{

}

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

Vector3D Vorton::velocity(const Vector3D &point) const
{
    return q(point)*vorticity;
}

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
        ans.Vel = (vorticity/(4*M_PI))*cv*av;
        for (int m=0; m<3; m++)
            for (int n=0; n<3; n++)
            {
                double apn = levi(m,0,n)*h0.x()+levi(m,1,n)*h0.y()+levi(m,2,n)*h0.z();
                double cpn=((1/s2.length()-1/s1.length()+2*cv*Vector3D::dotProduct(s0,h0))*h0[n]-(Vector3D::dotProduct((s2[n]*s2/pow(s2.length(),3)-s1[n]*s1/pow(s1.length(),3)+2*cv*s0[n]*h0),h0)))/av.lengthSquared();
                ans.B[m][n]=(vorticity/(4*M_PI))*(cpn*av[m]+cv*apn);
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
        Vector3D re = point-(radius/R-1)*(h0*Vector3D::dotProduct(s0,h0)/h0.lengthSquared()-s0);
        Vector3D s0 = re-mid;
        Vector3D  s1 = s0-h0;
        Vector3D s2 = s0+h0;
        Vector3D ae = Vector3D::crossProduct(h0,(re-mid));
        double ce = Vector3D::dotProduct(((re-mid+h0).normalized()-(re-mid-h0).normalized()),h0)/ae.lengthSquared();
        ans.Vel=vorticity/(4*M_PI)*(R/radius)*ce*ae;
        for (int m=0; m<3; m++)
            for (int n=0; n<3; n++)
            {
                double apn = levi(m,0,n)*h0.x()+levi(m,1,n)*h0.y()+levi(m,2,n)*h0.z();
                double cpn=((1/s2.length()-1/s1.length()+2*ce*Vector3D::dotProduct(s0,h0))*h0[n]-(Vector3D::dotProduct((s2[n]*s2/pow(s2.length(),3)-s1[n]*s1/pow(s1.length(),3)+2*ce*s0[n]*h0),h0)))/Vector3D::dotProduct(ae,ae);
                ans.B[m][n]=(R/radius*(vorticity/(4*M_PI))*(cpn*ae[m]+ce*apn));
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
        return 1.0;
    if ((firstComponent==2)&&(secondComponent==0)&&(thirdComponent==1))
        return -1.0;
    QMessageBox::critical(new QWidget(), "Ошибка", "Неверные параметры для тензора Леви-Чивитты");
    exit(1);
}

Vector3D Vorton::rotated(const Vector3D &vec, const Vector3D &axis, const double theta)
{
    Vector3D newaxis = axis/axis.length();
    double a = cos(theta/2.0);
    Vector3D secondary = -newaxis*sin(theta/2.0);
    double b = secondary.x();
    double c = secondary.y();
    double d = secondary.z();
    double aa = a*a;
    double bb = b*b;
    double cc = c*c;
    double dd = d*d;
    double bc = b*c;
    double ad = a*d;
    double ac = a*c;
    double ab = a*b;
    double bd = b*d;
    double cd = c*d;
    return Vector3D((aa+bb-cc-dd)*vec.x()+2*(bc+ad)*vec.y()+2*(bd-ac)*vec.z(),2*(bc-ad)*vec.x()+(aa+cc-bb-dd)*vec.y()+2*(cd+ab)*vec.z(), 2*(bd+ac)*vec.x()+2*(cd-ab)*vec.y()+(aa+dd-bb-cc)*vec.z());
}

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

void Vorton::setMid(const Vector3D& _mid)
{
    mid=_mid;
}

void Vorton::setTail(const Vector3D& _tail)
{
    tail=_tail;
}

void Vorton::setVorticity(const double _vorticity)
{
    vorticity=_vorticity;
}

void Vorton::setRadius(const double _radius)
{
    radius=_radius;
}

void Vorton::setElongation(const Vector3D& _elongation)
{
    elongation=_elongation;
}

void Vorton::setMove(const Vector3D& _move)
{
    move=_move;
}

Vector3D Vorton::getMid() const
{
    return mid;
}

Vector3D Vorton::getTail() const
{
    return tail;
}

double Vorton::getVorticity() const
{
    return vorticity;
}

double Vorton::getRadius() const
{
    return radius;
}

Vector3D Vorton::getElongation() const
{
    return elongation;
}

Vector3D Vorton::getMove() const
{
    return move;
}

void Vorton::turn ()
{
    tail = 2*mid-tail;
    vorticity = -vorticity;
}

void Vorton::translate(const Vector3D& translation)
{
    mid+=translation;
    tail+=translation;
}

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
