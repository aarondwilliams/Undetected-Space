//My includes
#include "environment.h"

//Read in headers
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <math.h>

//Initialization of environment by reading in file
Environment::Environment(unsigned env){

    //Find the correct filename based
    QString *filename = new QString(":/environments/env");
    char which = env + '0';
    filename->append(which);
    filename->append(".txt");

    QFile file(*filename);
    if(!file.open(QIODevice::ReadOnly)){
        qDebug() << "File was not opened";
    }
    QTextStream in(&file);

    //Adding in robot variables
    read_in_robot_vals(in);
    if(in.atEnd()){
        qDebug() << "File was not correctly configured";
    }

    //create border from file
    border = read_in_polygon(in.readLine());

    //add obstacles
    obstacles = QVector<QPolygonF>();
    while(!in.atEnd()){
        QPolygonF obstacle = read_in_polygon(in.readLine());
        obstacles.append(obstacle);
    }
    file.close();

    //Generate list of lines using border and obstacles
    get_all_lines();

    return;
}

//Reads the first line of the environment declaration and converts it to values
void Environment::read_in_robot_vals(QTextStream& in){

    QString line = in.readLine();
    QStringList robot_vars = line.split(" ");

    objective = robot_vars[0].toUInt();

    QStringList start = robot_vars[1].split(",");
    robot_start = QPointF(start[0].toFloat(),start[1].toFloat());

    if( objective != 0 ){
        QStringList end = robot_vars[2].split(",");
        robot_goal = QPointF(end[0].toFloat(),end[1].toFloat());
    }

    return;
}

//Converts a string of text into a polygon
//Space seperated points, comma seperated floats
QPolygonF Environment::read_in_polygon(QString line){

    QStringList points = line.split(" ");
    unsigned numofpoints = points.count();
    if(numofpoints < 3){
        qDebug() << "Not enough points for polygon";
    }

    QPolygonF output = QPolygonF();
    for(unsigned i = 0 ; i < numofpoints ; i++){
        QStringList point_string = points[i].split(",");
        QPointF point = QPointF(point_string[0].toFloat(),point_string[1].toFloat());
        output.append(point);
    }

    return output;
}

//Gets all existing object lines from border and obstacles for collision detections
void Environment::get_all_lines(){

    lines = QVector<QLineF>();

    //Get lines from border
    unsigned border_size = border.size() - 1;
    lines.append(QLineF(border.first(),border.last()));
    for( unsigned i = 0 ; i < border_size ; i++ ){
        lines.append(QLineF(border[i],border[i+1]));
    }

    //Get lines from obstacles
    unsigned num_obstacles = obstacles.size();
    for( unsigned i = 0 ; i < num_obstacles ; i++ ){

        //per obstacle
        unsigned obstacle_size = obstacles[i].size() - 1;
        lines.append(QLineF(obstacles[i].first(),obstacles[i].last()));
        for ( unsigned j = 0 ; j < obstacle_size ; j++ ){
            lines.append(QLineF(obstacles[i][j],obstacles[i][j+1]));
        }

    }
    return;
}

//Transforms an individual point from robot space to environment space
QPointF Environment::transform(QPointF point){
    double new_x = robot_start.x() + point.x();
    double new_y = robot_start.y() - point.y();
    return QPointF(new_x,new_y);
}

//Detects the closest intersection of a ray from a specific location
double Environment::ray_detect(Graphics * g, QPointF start, double angle){

    //Robot Transform
    QPointF ray_start = transform(start);
    angle *= -1;

    QPointF best_intersect = QPointF();
    double shortest_length = 1000;
    QPointF * intersect_point = new QPointF();

    //Generates a ray, which is just a line segment that ends outside of the space
    double dir_x = cos(angle);
    double dir_y = sin(angle);
    QLineF ray = QLineF(ray_start,ray_start + QPointF(1000*dir_x,1000*dir_y));

    //Goes through every line in the environment model and finds the closest intersection to the ray
    unsigned num_lines = lines.size();
    for(unsigned i = 0 ; i < num_lines ; i++ ){
        unsigned on_line = lines[i].intersect(ray, intersect_point);

        //If interesection is valid and then length is the smallest this becomes the next intersect point
        if( on_line == 1){
            double curr_length = QLineF(ray_start,*intersect_point).length();
            if(curr_length < shortest_length){
                best_intersect = *intersect_point;
                shortest_length = curr_length;
            }
        }
    }

    //Defines 5% detection maximum error in either direction
    double error = (shortest_length*2*D_ERROR)*(double(qrand())/double(RAND_MAX) - 0.5);

    //Updates graphics
    best_intersect += QPointF(error*dir_x,error*dir_y);
    g->update_points(best_intersect);

    return shortest_length+error;
}
