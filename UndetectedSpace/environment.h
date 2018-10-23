#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

//QT shape implementations
#include <QPolygonF>
#include <QLineF>
#include <QRectF>

//Readin headers
#include <QTextStream>
#include "shapes.h"
#include "graphics.h"

#define D_ERROR 0.05
double e_distance(QPointF,QPointF);

class Environment{
private:
    //Space variables
    QPolygonF border;
    QVector<QPolygonF> obstacles;
    QVector<QLineF> lines;

    //Robot variables
    QPointF robot_start;
    unsigned objective; // 0 - detect all space , 1 - go towards known goal , 2 - go towards unkown goal
    QPointF robot_goal; //If applicable

public:
    Environment(unsigned);

    //Declarations for environment constructor
    void read_in_robot_vals(QTextStream&);
    QPolygonF read_in_polygon(QString);
    void get_all_lines();

    //Collision detection for rays
    double ray_detect(Graphics *, QPointF , double);

    //Transform different things from robot space
    QPointF transform(QPointF);

    QPolygonF * get_border(){
        return &border;
    }

    QVector<QPolygonF> * get_obstacles(){
        return &obstacles;
    }

    unsigned get_objective(){
        return objective;
    }

    QPointF get_destination(){
        double x = robot_goal.x() - robot_start.x();
        double y = robot_start.y() - robot_goal.y();
        return QPointF(x,y);
    }

    QPointF get_robot_start(){
        return robot_start;
    }

    ~Environment();

};

#endif // ENVIRONMENT_H
