#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <QPolygonF>
#include <QLineF>
#include <QString>
#include <QRectF>

#include <QColor>
#include "environment.h"
#include "shapes.h"

/*
 * Graphics - A graphics class that holds relevent graphics for later display
 */

//Updated to draw additional distinguishing features on the display
class Graphics{

//All values are made public to allow ease of acces for graphics
private:
    unsigned anim_count;
    bool pathing;

    //For displaying ray detection locations
    QVector<QPointF> detection_points;
    //For displaying line estimations
    QVector<QLineF> line_estimates;
    //For displaying the verticies
    QVector<Vertex> vertex_points;
    //For displaying convex polygons as they are created
    QVector<QPolygonF> poly_list;
    //For displaying the projected path through a convex shape
    QVector<Path> convex_path;


public:
    Graphics(): detection_points(QVector<QPointF>()),
                line_estimates(QVector<QLineF>()),
                vertex_points(QVector<Vertex>()), pathing(false),
                poly_list(QVector<QPolygonF>()), anim_count(0) {
    }

    void update_animation(unsigned count){
        anim_count = count;
    }

    void update_pathing(bool u){
        pathing = u;
    }

    //Update Functions
    void update_points(QPointF point){
        detection_points.append(point);
    }
    void update_lines(QVector<QLineF> l_list){
        line_estimates = l_list;
    }
    void update_verts(QVector<Vertex> v_points){
        vertex_points = v_points;
    }
    void update_polys(QVector<QPolygonF> polys){
        poly_list = polys;
    }
    void update_path(QVector<Path> path){
        convex_path = path;
    }

    //Return Functions for mainwindow function
    QVector<QPointF> get_points(){
        QVector<QPointF> points_hold;
        int num_points = detection_points.size();
        if( anim_count < 5 ){
            num_points *=  ((double)anim_count/5);
        }

        points_hold.resize(num_points);
        std::copy(detection_points.begin(),detection_points.begin() + num_points,points_hold.begin());
        return points_hold;
    }
    QVector<QLineF> get_lines(){
        return line_estimates;
    }
    QVector<Vertex> get_verts(){
        return vertex_points;
    }
    QVector<QPolygonF> get_polys(){
        return poly_list;
    }
    QVector<Path> get_path(){
        return convex_path;
    }
    bool get_pathing(){
        return pathing;
    }

    //Query Function
    bool are_there_points(){
        if(anim_count > 0 && anim_count < 8){
            return !detection_points.empty();
        }
        return false;
    }
    bool are_there_lines(){
        if( (anim_count > 5 && anim_count < 8) || anim_count > 8){
            return !line_estimates.empty();
        }
        return false;
    }
    bool are_there_verts(){
        if(anim_count > 6 && anim_count < 8){
            return !vertex_points.empty();
        }
        return false;
    }
    bool are_there_polys(){
        if(anim_count > 7){
            return !poly_list.empty();
        }
        return false;
    }
    bool is_there_path(){
        if(anim_count == 10){
            return !convex_path.empty();
        }
        return false;
    }

    //Clear functions
    void clear_points(){
        detection_points.clear();
    }
    void clear_lines(){
        line_estimates.clear();
    }
    void clear_verts(){
        vertex_points.clear();
    }
    void clear_polys(){
        poly_list.clear();
    }
    void clear_path(){
        convex_path.clear();
    }

    ~Graphics();
};

#endif // GRAPHICS_H
