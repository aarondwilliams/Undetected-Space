#ifndef SHAPES_H
#define SHAPES_H

#include <QPolygonF>
#include <QLineF>
#include <QString>
#include <QRectF>

#include <QColor>
#include <QDebug>
#include "environment.h"

/* This header file includes a few different classes
 *
 * Graphics - A graphics class that holds relevent graphics for later display
 */

//Declare convex edge ahead of time
class ConvexPolygon;
double e_distance(QPointF, QPointF );
double norm_angle(double, QLineF );
QPointF around_circle(QPointF , QPointF , double , QPointF );

class Path{
private:
    //Initial straightline
    QLineF straight;
    bool just_straight;

    //Curve
    QPointF arc_center;
    double radius;
    double angle1;
    double angle2;

    //Endpoint
    QPointF end_point;

public:
    Path(QLineF line): just_straight(true), straight(line), end_point(line.p2()){
    }

    Path(QLineF line, QPointF c, QPointF p2, double a1, double a2, double r): straight(line), just_straight(false),
        angle1(a1), angle2(a2), end_point(p2), radius(r){

        arc_center = QPointF(c.x() - r,c.y() + r);
    }

    Path(){}

    bool is_straight(){
        return just_straight;
    }

    QLineF get_straight(){
        return straight;
    }

    QPointF get_end(){
        return end_point;
    }

    QPointF get_center(){
        return arc_center;
    }

    double get_radius(){
        return radius;
    }

    double get_angle1(){
        return angle1;
    }

    double get_angle2(){
        return angle2;
    }



};

class Vertex{
private:
    QPointF location;
    //Maximum error for vertex
    double variance;

public:
    Vertex(QPointF l, double var) : location(l), variance(var){
    }

    Vertex(): location(QPointF(0,0)) , variance(0){
    }

    QPointF get_loc(){
        return location;
    }

    double get_var(){
        return variance;
    }

    Path path_around(QLineF curr_path){

        double angle = atan2(curr_path.p2().y() - curr_path.p1().y(),curr_path.p2().x() - curr_path.p1().x());
        QPointF dir = QPointF(variance*cos(angle + M_PI/2),variance*sin(angle + M_PI/2));
        QLineF vert_cross = QLineF(location - dir,location + dir);

        QPointF inter; unsigned did_intersect;
        did_intersect = vert_cross.intersect(curr_path,&inter);

        if( did_intersect != 1){
            //If the path does not intersect with the vertex
            return Path(curr_path);
        }

        QPointF arc_start = around_circle(curr_path.p1(), location, variance, inter);
        double angle1 = atan2(arc_start.y() - location.y(),arc_start.x() - location.x());

        QPointF arc_end = around_circle(curr_path.p2(), location, variance, inter);
        double angle2 = atan2(arc_end.y() - location.y(),arc_end.x() - location.x());

        if( fabs( angle2 - angle1 ) > M_PI ){
            //Switches angles if in the wrong order
            angle2 -= 2*M_PI;
            if( fabs( angle2 - angle1 ) > M_PI ){
                angle2 += 4*M_PI;
            }
        }

        return Path(QLineF(curr_path.p1(),arc_start),location,arc_end,angle1,angle2, variance);
    }
};


class ConvexEdge{
private:
    //Verticies for convex edge
    Vertex v1;
    Vertex v2;

    //Faces out of the polygon
    double angle;

    //Pointer to edge it connects too
    ConvexPolygon * leads_to;   //Null if undetected
    ConvexPolygon * is_at;      //Points to current polygon

public:
    ConvexEdge(Vertex v_1, Vertex v_2, double a ,  ConvexPolygon * at): v1(v_1), v2(v_2), angle(a), is_at(at), leads_to(NULL){
    }

    ConvexEdge(Vertex v_1, Vertex v_2, double a , ConvexPolygon * at, ConvexPolygon * to):
        v1(v_1), v2(v_2), angle(a), is_at(at), leads_to(to){
    }

    ConvexEdge(){
    }

    modify(Vertex v_1, Vertex v_2, double a, ConvexPolygon * to){
        v1 = v_1;
        v2 = v_2;
        angle = a;
        leads_to = to;
    }

    bool been_to(){
        if( leads_to == NULL ){
            return false;
        }
        return true;
    }

    QLineF get_line(){
        return QLineF(v1.get_loc(),v2.get_loc());
    }

    double get_angle(){
        return angle;
    }

    Vertex get_v1(){
        return v1;
    }

    Vertex get_v2(){
        return v2;
    }

    ConvexPolygon * get_poly(){
        return is_at;
    }

    ConvexPolygon * goes_to(){
        return leads_to;
    }

    double closest_dist(QPointF loc){
        //Returns the distance of the closest vertex from a point
        double dist1 = e_distance(v1.get_loc(),loc);
        double dist2 = e_distance(v2.get_loc(),loc);
        if( dist1 < dist2 ){
            return dist1;
        }
        return dist2;
    }

    Vertex get_best_vert(QPointF loc){
        //Returns the location that is most similar to an input point
        double dist1 = e_distance(v1.get_loc(),loc);
        double dist2 = e_distance(v2.get_loc(),loc);
        if( dist1 < dist2 ){
            return v1;
        }
        return v2;
    }

    QVector<Vertex> best_verts(QVector<Vertex> * verts){
        //Looks at both verticies and determines the closest two from a set of vertexs
        double best_dist1 = e_distance(v1.get_loc(),(*verts)[0].get_loc());
        double best_dist2 = e_distance(v2.get_loc(),(*verts)[0].get_loc());
        unsigned id1 = 0;
        unsigned id2 = 0;

        //Check should always garantuee two seperate vertecies
        for( unsigned i = 1 ; i < verts->size() ; i++ ){
            double dist1 = e_distance(v1.get_loc(),(*verts)[i].get_loc());
            double dist2 = e_distance(v2.get_loc(),(*verts)[i].get_loc());
            if( dist1 < dist2 && dist1 < best_dist1 ){
                best_dist1 = dist1;
                id1 = i;
            }
            else if( dist2 < best_dist2){
                best_dist2 = dist2;
                id2 = i;
            }
        }

        QVector<Vertex> hold = QVector<Vertex>({(*verts)[id1],(*verts)[id2]});
        if( id1 > id2 ){
            verts->remove(id1);
            verts->remove(id2);
        }
        else{
            verts->remove(id2);
            verts->remove(id1);
        }

        return hold;
    }

    bool edge_match(ConvexEdge * oth_edge){
        //Find closest vert to first vert
        Vertex oth_v1 = oth_edge->get_best_vert(v1.get_loc());
        double min_dist = e_distance(oth_v1.get_loc(),v1.get_loc());

        //Check if minimum distance falls within both vertex variances
        if( min_dist < v1.get_var() && min_dist < oth_v1.get_var() ){
            Vertex oth_v2 = oth_edge->get_best_vert(v2.get_loc());
            min_dist = e_distance(oth_v2.get_loc(),v2.get_loc());

            //Check if second distance falls within both vertex variances
            if( min_dist < v2.get_var() && min_dist < oth_v2.get_var() ){

                Vertex best_v1 = v1;
                if( oth_v1.get_var() < v1.get_var() ){
                    best_v1 = oth_v1;
                    v1 = oth_v1;
                }
                Vertex best_v2 = v2;
                if( oth_v2.get_var() < v2.get_var() ){
                    best_v2 = oth_v2;
                    v2 = oth_v2;
                }

                leads_to = oth_edge->get_poly();
                angle = norm_angle(angle,QLineF(v1.get_loc(),v2.get_loc()));
                oth_edge->modify(best_v1,best_v2,angle + M_PI,is_at);
                return true;
            }
        }
        return false;
    }

    bool equals(ConvexEdge * edge){
        if( edge->get_line() == QLineF(v1.get_loc(),v2.get_loc())){
            return true;
        }
        return false;
    }

    double get_cost(ConvexEdge * edge){
        //Simply returns the average distance between two edges
        double dist1 = e_distance(v1.get_loc(),edge->get_v1().get_loc());
        double dist2 = e_distance(v2.get_loc(),edge->get_v2().get_loc());
        return (dist1 + dist2)/2;
    }

};

class ConvexPolygon{
private:
    unsigned index;
    QPolygonF shape;
    QVector<ConvexEdge *> connecting;

public:
    ConvexPolygon(unsigned i) : index(i){
    }

    void complete(QVector<ConvexEdge *> edges, QPolygonF s){
        connecting = edges;
        shape = s;
    }

    QPolygonF get_shape(){
        return shape;
    }

    unsigned get_index(){
        return index;
    }

    bool are_there_edges(){
        for( unsigned i = 0 ; i < connecting.size() ; i++ ){
            if( !connecting[i]->been_to() ){
                return true;
            }
        }
        return false;
    }

    QVector<ConvexEdge *> get_edges(){
        return connecting;
    }

    void fix_point(QPointF point){

        double min_dist = e_distance(shape[0],point);
        unsigned min_ind = 0;
        for(unsigned i = 1 ; i < shape.size() ; i++ ){
            double curr_dist = e_distance(shape[i],point);
            if( curr_dist < min_dist ){
                min_dist = curr_dist;
                min_ind = i;
            }
        }
        shape[min_ind] = point;
    }

    unsigned edge_to(unsigned poly){
        for( unsigned i = 0 ; i < connecting.size() ; i++ ){
            if( poly == connecting[i]->goes_to()->get_index() ){
                return i;
            }
        }
        qDebug() << "Could not find edge named in route";
        return 0;
    }

    QVector<QPolygonF> poly_list(QVector<unsigned> * curr_list){

        QVector<QPolygonF> shapes;
        shapes.append(shape);
        curr_list->append(index);

        for( unsigned i = 0 ; i < connecting.size() ; i++ ){
            if( connecting[i]->been_to() ){
                unsigned goes_to = connecting[i]->goes_to()->get_index();
                bool used = false;
                for( unsigned j = 0 ; j < curr_list->size() ; j++ ){
                    if( goes_to == (*curr_list)[j] ){
                        used = true;
                    }
                }

                if( used == false ){
                    shapes.append(connecting[i]->goes_to()->poly_list(curr_list));
                }
            }
        }
        return shapes;
    }

    double graph_search(QVector<unsigned> path,QVector<unsigned> * branch ,double cost, ConvexEdge * current, ConvexEdge * target){

        //Checks if current poly includes the targeted edge, if so, returns cost
        for( unsigned i = 0 ; i < connecting.size() ; i++ ){
            if(connecting[i]->equals(target)){
                branch->append(index);
                return cost + current->get_cost(target);
            }
        }

        double best_cost = 0;
        QVector<unsigned> best_branch;

        //If no target, searches polygons already seen
        for( unsigned i = 0 ; i < connecting.size() ; i++ ){

            //Looks to see if a direction is already on the current path
            unsigned goes_to = connecting[i]->goes_to()->get_index();
            bool on_path = false;
            for( unsigned j = 0 ; j < path.size() ; j++ ){
                if( goes_to == path[j] ){
                    on_path = true;
                }
            }

            //Recurses the graph search if any paths are applicable
            if( !on_path ){
                QVector<unsigned> curr_branch;
                path.append(goes_to);
                double curr_cost = connecting[i]->goes_to()->graph_search(path, &curr_branch, \
                       cost + current->get_cost(connecting[i]), connecting[i],target);
                path.removeLast();
                //Is this the best
                if( curr_cost != 0 && (curr_cost < best_cost || best_cost == 0)){
                    best_cost = curr_cost;
                    best_branch = curr_branch;
                }
            }
        }

        //Updates with the best cost and branch
        if( best_cost != 0 ){
            best_branch.prepend(index);
            *branch = best_branch;
            return best_cost;
        }
        return 0;
    }

};

#endif // SHAPES_H
