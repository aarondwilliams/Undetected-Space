#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//Necessary QT headers
#include <QMainWindow>
#include <QPainter>
#include "ui_mainwindow.h"

//Personal Headers
#include "environment.h"
#include "robot.h"
#include "shapes.h"
#include "graphics.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    Ui::MainWindow *ui;

    Environment * env;
    Robot * robot;
    Graphics * graphics;

public:

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void paintEvent(QPaintEvent*);

    void add_pointers(Environment*,Robot*,Graphics*);

};

#endif // MAINWINDOW_H
