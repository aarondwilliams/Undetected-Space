//Including own headers
#include "environment.h"
#include "robot.h"

#include <math.h>

//Constructor
Robot::Robot(Environment * env) : objective(env->get_objective()), location(QPointF(0,0)),
    cvxedge(NULL), cvxpoly(NULL), index_counter(0), state(0){

    if(objective == 1){
        QPointF * going_to = new QPointF(env->get_destination());
        objective_dest = going_to;
    }
    else{
        objective_dest = NULL;
    }

    //Current radius for robot which is a circle
    radius = 20;
    buffer = 2;

    return;
}

bool Robot::take_action(Graphics * g, Environment * env){

    if(state == 0){

        //Detects the space around the robot
        QVector<QPointF> point_cloud;
        QVector<double> angles;
        g->clear_points();
        detect_space(g, env, &point_cloud, &angles);

        //Convert point cloud into localized convex polygon
        ConvexPolygon * new_polygon = form_polygon(g, &point_cloud, &angles);
        if( new_polygon == NULL ){
            return EXIT_FAILURE;
        }
        //Add polygon to robot's graph
        update_poly(new_polygon);

        state = 1;
        g->update_pathing(false);

    }
    else{
        unsigned best_edge;
        QVector<Path> current_movement;

        if( state == 1 ){

            //Find next location to move to
            state = next_location(&best_edge);

            if( state == 3){
                //Exits upon seeing that all space has been detected
                return true;
            }
            else if( state == 2){
                //Perform graph planning to identify vector of instructions
                QVector<unsigned> path; path.append(cvxpoly->get_index());
                QVector<unsigned> branch;
                cvxpoly->graph_search(path,&branch,0,cvxedge,undetected[best_edge]);
                branch.removeFirst();
                route = branch;
            }
            else if( state == 4){
                //Location is discovered and moved to
                g->update_pathing(true);
                g->clear_lines();
                g->clear_path();
                current_movement.append(Path(QLineF(location,*objective_dest)));
                g->update_path(current_movement);
                location = current_movement.last().get_end();
                return true;
            }
        }
        if( state == 2 ){
            //Use graph plan to determine next edge
            best_edge = cvxpoly->edge_to(route.first());
            route.removeFirst();

            if( route.size() == 0 ){
                state = 1;
            }
        }
        //Identify best edge for graphics
        g->update_pathing(true);
        g->clear_lines();
        g->update_lines(QVector<QLineF>({cvxpoly->get_edges()[best_edge]->get_line()}));

        //Motion Planner through convex poly
        current_movement = generate_path(best_edge);
        g->clear_path();
        g->update_path(current_movement);
        location = current_movement.last().get_end();
        cvxedge = cvxpoly->get_edges()[best_edge];

        if( state != 0 ){
            cvxpoly = cvxedge->goes_to();
        }
    }

    return false;
}

unsigned Robot::next_location(unsigned * b_edge){

    if(objective == 1){
        //Is the final destination within the shape?
        bool found_goal = cvxpoly->get_shape().containsPoint(*objective_dest,Qt::OddEvenFill);
        if( found_goal ){
            return 4;
        }
    }
    if(cvxpoly->are_there_edges()){
        //If there are applicable edges in the current polygon, uses those
        *b_edge = best_edge(cvxpoly->get_edges());
        return 0;
    }
    else if(are_undetected()){
        //Looks to see if there are undetected edges in other polygons
        *b_edge = best_edge(undetected);
        return 2;
    }

    //All space detected condition
    return 3;
}


unsigned Robot::best_edge(QVector<ConvexEdge *> edges){

    unsigned best_edge = 0;
    while( edges[best_edge]->been_to() ){
        best_edge++;
    }
    //Finds the best edge given some objective
    //Starts with the assumption that the best edge is the first available

    if( objective_dest == NULL){
        //Checks through each edge and finds the minimum distance from the robot to the closest vertex
        double best_dist = edges[best_edge]->closest_dist(location);
        if( best_edge + 1 < edges.size()){
            for( unsigned i = best_edge + 1 ; i < edges.size() ; i++ ){
                if( !edges[best_edge]->been_to() ){
                    double curr_dist = edges[i]->closest_dist(location);
                    if( curr_dist < best_dist ){
                        best_dist = curr_dist;
                        best_edge = i;
                    }
                }
            }
        }
    }
    else{
        //If the objective location is known, searches for that point
        double best_dist = edges[best_edge]->closest_dist(*objective_dest);
        if( best_edge + 1 < edges.size()){
            for( unsigned i = best_edge + 1 ; i < edges.size() ; i++ ){
                if( !edges[best_edge]->been_to() ){
                    double curr_dist = edges[i]->closest_dist(*objective_dest);
                    if( curr_dist < best_dist ){
                        best_dist = curr_dist;
                        best_edge = i;
                    }
                }
            }
        }

    }

    return best_edge;
}

QVector<Path> Robot::generate_path(unsigned edge){

    //Get the closest vertex on opposite line
    ConvexEdge * target_edge = cvxpoly->get_edges()[edge];
    Vertex closest_vert = target_edge->get_best_vert(location);

    //Determine if perpendicular direction produces an adequate line
    QLineF dest = target_edge->get_line();
    double angle = target_edge->get_angle();
    QPointF dir = 1000*QPointF(cos(angle),sin(angle));
    QLineF poss_path = QLineF(location,location + dir);
    QPointF inter; unsigned did_intersect;
    did_intersect = poss_path.intersect(dest,&inter);

    if( cvxedge != NULL){
        //If we're already on the target edge, moves to it if it's been adjusted
        if( cvxedge->equals(target_edge) ){
            QVector<Path> path;
            path.append(Path(QLineF(location,inter)));
            return path;
        }
    }

    if( did_intersect == 1 ){
        //Create initial line
        poss_path = QLineF(location,inter);
    }
    else{
        //Create initial line to the interior of that vertex point plus variance

        dir = closest_vert.get_var()*QPointF(cos(angle+M_PI/2),sin(angle+M_PI/2));
        double dist1 = e_distance(inter,closest_vert.get_loc() + dir);
        double dist2 = e_distance(inter,closest_vert.get_loc() - dir);
        if( dist1 > dist2 ){
            poss_path = QLineF(location,closest_vert.get_loc() + dir);
        }
        else{
            poss_path = QLineF(location,closest_vert.get_loc() - dir);
        }
    }

    //Starts off the path
    QVector<Path> convex_path;
    //Checks if robot intersects either vertex, and fixes if necessary

    if( cvxedge != NULL ){
        //Determine if there is any interceptions straight line movement
        Path new_path = cvxedge->get_v1().path_around(poss_path);
        if( new_path.is_straight() ){
            new_path = cvxedge->get_v2().path_around(poss_path);
        }
        if( !new_path.is_straight()){
            //If the path returned by the vertexes includes an arc
            convex_path.append(new_path);
            poss_path = QLineF(new_path.get_end(),poss_path.p2());
        }
    }

    //Check if robot intersects far path vertexes
    //Adjust path if necessary
    convex_path.append(closest_vert.path_around(poss_path));

    //Make movements?
    return convex_path;
}
