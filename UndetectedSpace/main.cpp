//Personal Headers
#include "environment.h"
#include "shapes.h"
#include "robot.h"
#include "graphics.h"
#include "mainwindow.h"

//Necessary QT Headers
#include <QApplication>
#include <QMessageBox>
#include <QTime>

void delay(int length)
{
    QTime die_time = QTime::currentTime().addMSecs(length);
    while (QTime::currentTime() < die_time){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

void animate(MainWindow *w,Graphics *g){

    if( !g->get_pathing() ){
        for( unsigned i = 0 ; i < 9 ; i++ ){
            g->update_animation(i);
            w->update();
            delay(500);
        }
    }
    else{
        for( unsigned i = 9 ; i < 11 ; i++ ){
            g->update_animation(i);
            w->update();
            delay(1200);
        }
    }
}

void print_message(Robot *r){

    //Prints a message regarding the outcome of the robot's movements
    QMessageBox msgBox;
    if( r->get_state() == 3 && r->get_objective() == 0 ){
        msgBox.setText("Robot identified all pathable space.");
    }
    else if( r->get_state() == 3 && r->get_objective() == 1 ){
        msgBox.setText("Robot could not find a path to the goal.");
    }
    else if( r->get_state() == 4 && r->get_objective() == 1 ){
        msgBox.setText("Robot has reached the goal");
    }
    msgBox.exec();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow *w = new MainWindow();

    //Initialize necessary elements
    Environment *e = new Environment(7);
    Robot *r = new Robot(e);
    Graphics *g = new Graphics();
    w->add_pointers(e,r,g);
    w->show();

    bool objective_completed = false;

    while( !objective_completed ){

        objective_completed = r->take_action(g,e);
        if( r->get_state() != 3 ){
            animate(w,g);
        }
    }

    print_message(r);

    return a.exec();
}
