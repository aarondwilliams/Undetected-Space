#ifndef ROBOT_H
#define ROBOT_H
//Personal Headers
#include "shapes.h"
#include "environment.h"
#include "graphics.h"

//QT headers
#include <QPointF>
#include <QVector>

class Robot{
private:
    unsigned objective;
    QPointF * objective_dest; //If applicable

    //Location
    QPointF location;
    ConvexPolygon * cvxpoly;
    ConvexEdge * cvxedge;

    //For Graph Planning
    unsigned index_counter;
    QVector<unsigned> route;
    //List of untouched edges
    QVector<ConvexEdge *> undetected;

    //Description of the Robot
    unsigned state; //0 - detecting area, 1 - moving
    double radius;  //Size of Robot
    double buffer;

public:
    Robot(Environment *);

    bool take_action(Graphics *, Environment *); //true with completed objective

    unsigned next_location(unsigned *);
    unsigned best_edge(QVector<ConvexEdge *>);
    QVector<Path> generate_path(unsigned);

    //
    //  All Detection Functions
    //

    //For space detection
    void detect_space(Graphics *, Environment *, QVector<QPointF> *, QVector<double> *);
    QVector<QPointF> detect_space(Graphics *, Environment *);
    double angle_increment(QVector<QPointF> *, QVector<double> *, double, Graphics *, Environment *);
    void help_increment(QVector<QPointF> *, QVector<double> *, Graphics *, Environment *, unsigned);

    //For convex polygon creation
    ConvexPolygon * form_polygon(Graphics *, QVector<QPointF> * , QVector<double> * );
    ConvexPolygon * make_polygon(QVector<QLineF> *, QVector<Vertex> *);
    ConvexPolygon * identify_edges(QVector<QLineF> * , QVector<Vertex> );
    QVector<ConvexEdge *> edge_checker(QVector<ConvexEdge *>, ConvexPolygon *);
    unsigned best_min(QVector<double>);

    //Point discretion
    void translate_cloud(QVector<QPointF> * , QVector<double> * , QVector<QLineF> * , QVector<Vertex> * );
    QVector<double> pick_out_line(QVector<QPointF> *, unsigned *, unsigned *, unsigned, char *);
    QVector<double> line_info(QVector<QPointF>, double, double, double, double );
    double find_var(QPointF, QLineF, double, QLineF, double );

    //
    //  Return Functions
    //

    QPointF get_location(){
        return location;
    }

    double get_radius(){
        return radius;
    }

    void add_undetected(ConvexEdge * edge){
        undetected.append(edge);
    }

    void remove_undetected(QLineF edge){
        for( unsigned i = 0 ; i < undetected.size() ; i++ ){
            if( undetected[i]->get_line() == edge ){
                undetected.remove(i);
                return;
            }
        }
    }

    bool are_undetected(){
        if( undetected.size() > 0 ){
            return true;
        }
        return false;
    }

    void update_poly(ConvexPolygon * poly){
        cvxpoly = poly;
        QVector<ConvexEdge *> curr_edges = poly->get_edges();

        //Determines if edges already exist and marks them as been to if they have
        for( unsigned j = 0 ; j < curr_edges.size() ; j++ ){
            if( !curr_edges[j]->been_to() ){
                bool match;
                for( unsigned i = 0 ; i < undetected.size() ; i++ ){
                    match = undetected[i]->edge_match(curr_edges[j]);
                    if( match ){
                        undetected.remove(i);
                        i--;
                        break;
                    }
                }
                if( !match ){
                    undetected.append(curr_edges[j]);
                }
            }
        }
        return;
    }

    unsigned get_objective(){
        return objective;
    }

    unsigned get_state(){
        return state;
    }



};

#endif // ROBOT_H
