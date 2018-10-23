#include "math.h"
#include <QLineF>
#include <QVector>

#define D_ERROR 0.05

double e_distance(QPointF a,QPointF b){
    return sqrt(pow(a.x()-b.x(),2) + pow(a.y()-b.y(),2));
}

double point_to_line(QPointF a, QLineF b){
    QPointF p1 = b.p1();
    QPointF p2 = b.p2();
    double num = abs( (p2.y() - p1.y())*a.x() - (p2.x() - p1.x())*a.y() + p2.x()*p1.y() - p2.y()*p1.x() );
    double den = sqrt( pow(p2.y() - p1.y(),2) + pow(p2.x() - p1.x(),2));
    return num/den;
}

QPointF around_circle(QPointF start, QPointF circle, double radius, QPointF closest ){

    double x = start.x() - circle.x();
    double y = start.y() - circle.y();

    double sqrt_quad;
    if( pow(x,2) + pow(y,2) - pow(radius,2) < 0){
        sqrt_quad = 0;
    }
    else{
        sqrt_quad = sqrt(pow(x,2) + pow(y,2) - pow(radius,2));
    }
    double den = pow(x,2) + pow(y,2);

    double x_tan1 = (pow(radius,2)*x + radius*y*sqrt_quad)/den + circle.x();
    double y_tan1 = (pow(radius,2)*y - radius*x*sqrt_quad)/den + circle.y();
    double x_tan2 = (pow(radius,2)*x - radius*y*sqrt_quad)/den + circle.x();
    double y_tan2 = (pow(radius,2)*y + radius*x*sqrt_quad)/den + circle.y();

    double dist1 = e_distance(closest,QPointF(x_tan1,y_tan1));
    double dist2 = e_distance(closest,QPointF(x_tan2,y_tan2));
    if( dist2 > dist1 ){
        return QPointF(x_tan1,y_tan1);
    }
    return QPointF(x_tan2,y_tan2);
}

bool half_pi(double angle1, double angle2){

    //Fit angles to range
    while( angle1 > M_PI ){
        angle1 -= 2*M_PI;
    }
    while( angle1 < -M_PI ){
        angle1 += 2*M_PI;
    }
    while( angle2 > M_PI ){
        angle2 -= 2*M_PI;
    }
    while( angle2 < -M_PI ){
        angle2 += 2*M_PI;
    }

    //Do opposite end corrections
    if( angle1 - angle2 > 3*M_PI/2 ){
        angle2 += 2*M_PI;
    }
    if( angle1 - angle2 < -3*M_PI/2 ){
        angle2 -= 2*M_PI;
    }

    //Final angle check
    if( fabs(angle1 - angle2) < M_PI/2 ){
        return true;
    }
    return false;
}

//Gets the normal angle of an edge that is within pi/2 of an incident angle
double norm_angle(double incident, QLineF edge){

    //Fit incident angle to range
    while( incident > M_PI ){
        incident -= 2*M_PI;
    }
    while( incident < -M_PI ){
        incident += 2*M_PI;
    }

    double norm = atan2(edge.p2().y() - edge.p1().y(), edge.p2().x() - edge.p1().x()) + M_PI/2;

    if( half_pi(norm,incident) ){
        return norm;
    }

    if( norm > 0 ){
        return norm - M_PI;
    }
    return norm + M_PI;

}

void replace_point(QPointF replace, QPointF use, QVector<QLineF> * s_lines){

    for( unsigned i = 0 ; i < s_lines->size() ; i++ ){

        if((*s_lines)[i].p1() == replace){
            (*s_lines)[i].setP1(use);
        }
        else if((*s_lines)[i].p2() == replace){
            (*s_lines)[i].setP2(use);
        }
    }
}


