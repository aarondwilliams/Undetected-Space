#include "mainwindow.h"
#include "ui_mainwindow.h"

//Additional painting headers
#include <QPainter>
#include <QPicture>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

void MainWindow::paintEvent(QPaintEvent*){
    QPainter painter(this);

    painter.setBrush(Qt::black);
    painter.drawRect(QRect(0,0,900,600));

    //Paint Environment

    painter.setPen(Qt::NoPen);

    painter.setBrush(Qt::white);
    painter.drawPolygon(*env->get_border());

    painter.setBrush(Qt::darkGray);
    unsigned num_obstacles = env->get_obstacles()->size();
    for( unsigned i = 0 ; i < num_obstacles ; i++ ){
        painter.drawPolygon((*env->get_obstacles())[i]);
    }

    //Paint Graphics

        //Points for point detection

    if( graphics->are_there_points() ){
        painter.setPen(Qt::NoPen);
        QVector<QPointF> points = graphics->get_points();
        painter.setBrush(Qt::red);
        for( unsigned i = 0 ; i < points.size() ; i++ ){
            painter.drawEllipse(points[i],3,3);
        }
    }

    if( graphics->are_there_verts() ){
        QPen pen;
        pen.setWidth(2);
        pen.setBrush(Qt::cyan);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        QVector<Vertex> verts = graphics->get_verts();
        for( unsigned i = 0 ; i < verts.size() ; i++ ){
            painter.drawEllipse(env->transform(verts[i].get_loc()),verts[i].get_var(),verts[i].get_var());
        }
    }

    if( graphics->are_there_polys() ){
        QPen pen;
        pen.setWidth(2);
        pen.setBrush(Qt::green);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        QVector<QPolygonF> polys = graphics->get_polys();
        for( unsigned i = 0 ; i < polys.size() ; i++ ){
            QPolygonF curr_poly;
            for( unsigned j = 0 ; j < polys[i].size() ; j++ ){
                curr_poly << env->transform(polys[i][j]);
            }
            painter.drawConvexPolygon(curr_poly);
        }
    }

    if( graphics->are_there_lines() ){
        QPen pen;
        pen.setWidth(4);
        pen.setBrush(Qt::yellow);
        pen.setCapStyle(Qt::RoundCap);
        painter.setPen(pen);
        QVector<QLineF> lines = graphics->get_lines();
        for( unsigned i = 0 ; i < lines.size() ; i++ ){
            painter.drawLine(env->transform(lines[i].p1()),env->transform(lines[i].p2()));
        }
    }

    if( graphics->is_there_path() ){
        QPen pen;
        pen.setWidth(2*robot->get_radius());
        pen.setBrush(Qt::magenta);
        pen.setCapStyle(Qt::RoundCap);
        painter.setPen(pen);
        QVector<Path> path = graphics->get_path();
        for( unsigned i = 0 ; i < path.size() ; i++ ){
            painter.drawLine(env->transform(path[i].get_straight().p1()),env->transform(path[i].get_straight().p2()));
            if( !path[i].is_straight() ){
                QPointF cent = env->transform(path[i].get_center());
                QRectF bounding = QRectF(cent.x(),cent.y(),2*path[i].get_radius(),2*path[i].get_radius());
                double angle1 = path[i].get_angle1()*16*180/M_PI;
                double angle2 = (path[i].get_angle2()*16*180/M_PI) - angle1;
                painter.drawArc(bounding,angle1,angle2);
            }
        }
    }

    //Paint Robot
    painter.setPen(Qt::SolidLine);
    painter.setBrush(Qt::darkGray);

    QPointF robot_loc = env->transform(robot->get_location()) ;
    double robot_radius = robot->get_radius();
    painter.drawEllipse(robot_loc,robot_radius,robot_radius);

    if( env->get_objective() == 1 ){
        //Paint objective
        painter.setBrush(Qt::blue);
        QPointF goal = env->transform(env->get_destination());
        painter.drawEllipse(goal,10,10);
    }

    return;
}

void MainWindow::add_pointers(Environment * new_env, Robot * new_rob, Graphics * new_graph){
    env = new_env;
    robot = new_rob;
    graphics = new_graph;
    return;
}

MainWindow::~MainWindow()
{
    delete ui;
}
