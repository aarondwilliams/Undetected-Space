//Including own headers
#include "environment.h"
#include "robot.h"

#include <math.h>
#include <QDebug>

double point_to_line(QPointF, QLineF);
double norm_angle(double , QLineF );
void replace_point(QPointF, QPointF, QVector<QLineF> *);
bool half_pi(double , double );

void Robot::detect_space(Graphics * g, Environment * env, QVector<QPointF> * point_list, QVector<double> * angles){

    //Initialize variables
    double curr_angle = 0;
    double finish_angle = 0;

    //Determine the angles the robot should go between
    if(cvxedge == NULL){
        finish_angle = 2*M_PI;
    }
    else{
        //Angles follow the direction of the edge with an additional 3*pi/8 on each side to catch verticies
        curr_angle = cvxedge->get_angle() - M_PI*11/16;
        finish_angle = cvxedge->get_angle() + M_PI*11/16;
    }

    //Takes a measurement at each angle
    double curr_distance = 0;
    while( curr_angle < finish_angle ){
        curr_distance = env->ray_detect(g, location, curr_angle);
        point_list->append(QPointF(location.x() + curr_distance*cos(curr_angle), location.y() + curr_distance*sin(curr_angle)));
        angles->append(curr_angle);
        //Increment angle based on the distance reading
        curr_angle += angle_increment(point_list, angles, curr_distance ,g ,env);
    }
    //Include finish angle in with the measurements
    curr_distance = env->ray_detect(g, location, finish_angle);
    point_list->append(QPointF(location.x() + curr_distance*cos(finish_angle), location.y() + curr_distance*sin(finish_angle)));
    angles->append(finish_angle);
    angle_increment(point_list, angles, curr_distance ,g ,env);
}

//Determines the appropriate angle increment for the space detection algorithm
//Uses a help function bisect problem areas and vertices that stick out
double Robot::angle_increment(QVector<QPointF> * point_list, QVector<double> * angles, double depth, Graphics * g, Environment * env){

    unsigned last_index = point_list->size() - 1;

    if( last_index >= 1){
        //If the seperation space is less than 2*radius, it's unlikely a fitable space would have resulted.
        QPointF last_diff = point_list->last() - (*point_list)[last_index-1];
        double last_change = sqrt(pow(last_diff.x(),2) + pow(last_diff.y(),2));

        if( last_change > 2*radius ){
            //Helper function to bisect angles appropriately
            help_increment(point_list,angles,g,env,0);
            last_diff = point_list->last() - (*point_list)[last_index-1];
        }

        //An angle is selected by looking at a 3 sided triangle of the depth, the radius, and the predicted depth.
        double diff_angle = atan(last_diff.y()/last_diff.x());
        double angle = angles->last();
        double pred_depth = sqrt(pow(depth*cos(angle) + radius*cos(diff_angle),2)+pow(depth*sin(angle) + radius*sin(diff_angle),2));
        double angle_change = acos((pow(depth,2) + pow(pred_depth,2) - pow(radius,2))/(2*pred_depth*depth));
        if( angle_change > M_PI/16 ){
            //Minimum angle change
            angle_change = M_PI/16;
        }

        return angle_change;

    }
    else{
        //Returns the angle of a normal surface, but calculates it to half the radius
        return 2*asin(radius/(4*depth));
    }

}

void Robot::help_increment(QVector<QPointF> * point_list, QVector<double> * angles, Graphics * g, Environment * e, unsigned recurse){

    if(recurse == 6){
        //Maximum depth for function.  This value would be based off of angle error if it was currently implemented
        return;
    }

    //Pops last point
    QPointF last_point = point_list->takeLast();
    double last_angle = angles->takeLast();
    QPointF first_point = point_list->last();
    double first_angle = angles->last();

    //Gets bisection angle and distance
    double bisect_angle = (last_angle + first_angle) / 2;
    double curr_dist = e->ray_detect(g, location, bisect_angle);

    //Adds in middle point
    QPointF middle_point = QPointF(location.x() + curr_dist*cos(bisect_angle), location.y() + curr_dist*sin(bisect_angle));
    point_list->append(middle_point);
    angles->append(bisect_angle);

    //Recurses the function if the the first seperation is still less than 2*radius or err
    double dist = e_distance(first_point,middle_point);
    double err_dist = (e_distance(first_point,location) + e_distance(middle_point,location))/2;
    if( dist > 2*radius && dist > 0.1*err_dist ){
        help_increment(point_list,angles,g,e,recurse+1);
    }

    //Adds in last point
    point_list->append(last_point);
    angles->append(last_angle);

    //Recurses the function if the the second seperation is still less than 2*radius or err
    dist = e_distance(last_point,middle_point);
    err_dist = (e_distance(last_point,location) + e_distance(middle_point,location))/2;
    if( dist > 2*radius && dist > 0.1*err_dist ){
        help_increment(point_list,angles,g,e,recurse+1);
    }

    return;
}

ConvexPolygon * Robot::form_polygon(Graphics * g, QVector<QPointF> * point_cloud, QVector<double> * angles){

    QVector<QLineF> solid_lines;
    QVector<Vertex> vertices;
    translate_cloud(point_cloud, angles, &solid_lines, &vertices);

    g->clear_lines();
    g->update_lines(solid_lines);
    g->clear_verts();
    g->update_verts(vertices);

    ConvexPolygon * poly_ptr =  make_polygon( &solid_lines, &vertices);

    QVector<unsigned> been_to;
    g->update_polys(poly_ptr->poly_list(&been_to));

    return poly_ptr;
}

//Gets the verticies of the for the polygon
ConvexPolygon * Robot::make_polygon(QVector<QLineF> * s_lines, QVector<Vertex> * verts){

    QVector<double> vert_dists;
    QVector<QLineF> vert_rays;
    QVector<Vertex> poly_verts;
    unsigned length = verts->size();
    unsigned addin;

    if( cvxedge == NULL ){
        //Sets values for initial case
        addin = 3;
    }
    else{
        //Determine the two vertices on the current polygon border and add in their rays
        //Initialize other variables of importance
        poly_verts = cvxedge->best_verts(verts);
        //Append rays
        QPointF dir1 = 100*(poly_verts[0].get_loc() - location);
        QPointF dir2 = 100*(poly_verts[1].get_loc() - location);
        vert_rays.append(QLineF(poly_verts[0].get_loc(),poly_verts[0].get_loc() + dir1));
        vert_rays.append(QLineF(poly_verts[1].get_loc(),poly_verts[1].get_loc() + dir2));
        length = length - 2;
        addin = 1;
    }

    //Gets distances for detected vertices
    for( unsigned i = 0 ; i < verts->size() ; i++ ){
        QPointF loc = (*verts)[i].get_loc();
        //Removes vertices that are in the incorrect direction
        if( cvxedge != NULL ){
            double vert_angle = atan2((*verts)[i].get_loc().y() - location.y(),(*verts)[i].get_loc().x() - location.x());
            if( !half_pi(cvxedge->get_angle(),vert_angle) ){
                verts->remove(i);
                i--;
            }
            else{
                  vert_dists.append(e_distance((*verts)[i].get_loc(),location));
            }
        }
        else{
            vert_dists.append(e_distance((*verts)[i].get_loc(),location));
        }
    }

    //Takes a certain number of the closest vertices
    if( verts->size() >= addin ){
        for( unsigned i = 0 ; i < addin ; i++ ){
            unsigned min_index = best_min(vert_dists);
            poly_verts.append((*verts)[min_index]);
            //Adds ray for convex determination
            QPointF dir = 100*((*verts)[min_index].get_loc() - location);
            vert_rays.append(QLineF((*verts)[min_index].get_loc(),(*verts)[min_index].get_loc() + dir));
            vert_dists.remove(min_index);
            verts->remove(min_index);
        }
    }
    else{
        return NULL;
    }

    //Checks if each vert is capable of forming a convex polygon and adds it in if so
    unsigned vert_len = verts->size();
    for( unsigned i = 0 ; i < vert_len ; i++ ){
        //Checks the next closest index for the next vertex check
        unsigned min_index = best_min(vert_dists);
        bool valid_vert = true;
        for( unsigned j = 0 ; j < poly_verts.size() ; j++ ){
            //Forms a connecting line to each verticy already in the polygon
            QLineF conn_line = QLineF((*verts)[min_index].get_loc(),poly_verts[j].get_loc());

            //Verifies that the line is not already contained as a solid line
            if( !s_lines->contains(conn_line) ){
                //Checks each outward vertex for intersection
                for( unsigned k = 0 ; k < vert_rays.size() ; k++ ){
                    if( k != j ){
                        QPointF * inter_point = NULL;
                        if(conn_line.intersect(vert_rays[k],inter_point) == 1){
                            valid_vert = false;
                            break;
                        }
                    }
                }
            }
            if( valid_vert == false){
                //breaks the second for loop
                verts->remove(min_index);
                vert_dists.remove(min_index);
                break;
            }
        }
        if(valid_vert == true){
            //Adds in new vertex
            poly_verts.append((*verts)[min_index]);
            QPointF dir = 100*(2*(*verts)[min_index].get_loc() - location);
            vert_rays.append(QLineF((*verts)[min_index].get_loc(),dir));
            vert_dists.remove(min_index);
            verts->remove(min_index);
        }
    }
    return identify_edges(s_lines, poly_verts);
}

//Identifies the convex edges for a new convex polygon
ConvexPolygon * Robot::identify_edges(QVector<QLineF> * s_lines, QVector<Vertex> p_verts){

    //Get all connecting lines that are not solid lines
    QPolygonF shape;
    //Create convex polygon to fill later with checked edges
    ConvexPolygon * convex_poly = new ConvexPolygon(index_counter);
    index_counter++;
    //Form edges
    QVector<ConvexEdge *> edges;

    /*Edge verts from previous shape are first two vertecies in p_verts
      Make sure that lower variance vertex is selected for both edges*/
    if( cvxedge != NULL ){
        Vertex v1 = p_verts[0];
        Vertex v2 = p_verts[1];
        if( cvxedge->get_v1().get_var() < v1.get_var() ){
            //Replaces point in appropriate solid line categories
            replace_point(v1.get_loc(),cvxedge->get_v1().get_loc(),s_lines);
            v1 = cvxedge->get_v1();
            p_verts[0] = v1;
        }
        if( cvxedge->get_v2().get_var() < v2.get_var() ){
            //Replaces point in appropriate solid line categories
            replace_point(v2.get_loc(),cvxedge->get_v2().get_loc(),s_lines);
            v2 = cvxedge->get_v2();
            p_verts[1] = v2;
        }
        double norm = norm_angle(cvxedge->get_angle(),QLineF(v1.get_loc(),v2.get_loc()));
        //Modifies current edge and creates a new one to be added to the list
        cvxedge->modify(v1,v2,norm,convex_poly);
        cvxpoly->fix_point(v1.get_loc()); cvxpoly->fix_point(v2.get_loc());
        ConvexEdge * curr_edge = new ConvexEdge(v1,v2,norm+M_PI,convex_poly,cvxpoly);
        edges.append(curr_edge);
        //Removes the vertex from
        s_lines->append(curr_edge->get_line());
        remove_undetected(curr_edge->get_line());
    }

    //Sort vertices by minimum angle
    QVector<double> angles;
    QVector<Vertex> o_verts;
    for( unsigned i = 0 ; i < p_verts.size() ; i++ ){
        QPointF p_angle = p_verts[i].get_loc()-location;
        double curr_angle = atan2(p_angle.y(),p_angle.x());
        //Iterate up until the next angle is larger
        unsigned j = 0;
        for( ; j < angles.size() ; j++ ){
            if( curr_angle < angles[j]){
                break;
            }
        }
        angles.insert(j,curr_angle);
        o_verts.insert(j,p_verts[i]);
    }

    //Form shapes and edges
    for( unsigned i = 0 ; i < o_verts.size() ; i++ ){
        Vertex v1 = o_verts[i];
        Vertex v2;
        if(i == 0){
            v2 = o_verts.last();
        }
        else{
            v2 = o_verts[i-1];
        }

        QLineF curr_line1 = QLineF(v1.get_loc(),v2.get_loc());
        QLineF curr_line2 = QLineF(v2.get_loc(),v1.get_loc());
        shape << o_verts[i].get_loc();

        bool is_solid = s_lines->contains(curr_line1) || s_lines->contains(curr_line2);
        bool is_long = curr_line1.length() >= (v1.get_var() + v2.get_var());
        if( !is_solid && is_long ){
            //Gets a normal angle that is within 'pi' of the incidental angle robot angle
            double norm = norm_angle(angles[i],curr_line1);

            ConvexEdge * curr_edge = new ConvexEdge(v1,v2,norm,convex_poly);
            edges.append(curr_edge);
        }
    }

    convex_poly->complete(edges,shape);

    return convex_poly;
}

//Returns the best minimum index for verts
unsigned Robot::best_min(QVector<double> v_dists){
    unsigned length = v_dists.size();

    double min_val = v_dists[0];
    unsigned min_idx = 0;
    for( unsigned i = 1 ; i < length ; i++ ){
        if( min_val > v_dists[i]){
            min_val = v_dists[i];
            min_idx = i;
        }
    }
    return min_idx;
}

//Turns an ordered point cloud into a set of lines and verticies
void Robot::translate_cloud(QVector<QPointF> * p_cloud, QVector<double> * angles, QVector<QLineF> * s_lines, QVector<Vertex> * verts){

    //Initialize iterators
    unsigned cloud_size = p_cloud->size();
    unsigned start_line = 0;
    unsigned end_line = 0;

    //Line information
    QVector<double> curr_info;
    QVector<double> prev_info;
    QPointF first_point;
    char cond = 0;

    do{
        char prev_cond = cond;
        curr_info = pick_out_line(p_cloud, &start_line, &end_line, cloud_size, &cond );

        //Determines a connecting vertex for the previous line if applicable
        if( prev_cond == 1 ){
            double prev_x = (prev_info[1] - curr_info[1])/(curr_info[0]-prev_info[0]);
            double prev_y = prev_info[0]*prev_x + prev_info[1];
            QPointF intersect_point = QPointF(prev_x,prev_y);

            //If the last point was a vert, adds a solid line
            if( verts->size() > 0 ){
                if(first_point == verts->last().get_loc()){
                    s_lines->append(QLineF(first_point,intersect_point));
                }
            }
            //Uses the first point determined from the last iteration which has not yet been modified.
            double var = find_var(intersect_point,QLineF(first_point,intersect_point),radius + prev_info[2], \
                    QLineF(intersect_point,intersect_point+QPointF(1,curr_info[0])), radius + curr_info[2]);
            verts->append(Vertex(intersect_point, var + buffer));

            first_point = intersect_point;
        }
        else if( prev_cond == 3 ){
            //Condition for forward point versus previous line
            //Gets angles and estimates a middle angle from the previous function
            double ray_angle = (*angles)[start_line];
            double last_ray_angle = (*angles)[start_line-1];
            double middle_angle = (ray_angle + last_ray_angle) / 2;
            //Gets intersect for the ray and the middle ray on the predicted line, determining the difference between them for error
            QLineF ray = QLineF((*p_cloud)[start_line],location);
            QLineF middle_ray = QLineF(location.x(),location.y(),location.x()+cos(middle_angle),location.y()+sin(middle_angle));
            QLineF line = QLineF(0,curr_info[1],1,curr_info[1]+curr_info[0]);
            QPointF int1;
            QPointF int2;
            line.intersect(ray,&int1);
            line.intersect(middle_ray,&int2);
            //Bound error
            double angle_error = sqrt(pow(int1.x() - int2.x(),2) + pow(int1.y() - int2.y(),2));
            double var = find_var(int2,line,curr_info[2]+angle_error+radius,middle_ray,radius);
            verts->append(Vertex(int2, buffer + var));
            first_point = int2;
        }
        else if( prev_cond != 0 ){
            //Gets a filler point (no vertex) for the solid line
            QLineF ray = QLineF((*p_cloud)[start_line],location);
            QLineF line = QLineF(0,curr_info[1],1,curr_info[1]+curr_info[0]);
            QPointF int1;
            line.intersect(ray,&int1);
            first_point = int1;
        }

        if( cond == 1 ){
            prev_info = curr_info;
            if( prev_cond == 0){
                QLineF ray = QLineF((*p_cloud)[start_line],location);
                QLineF line = QLineF(0,curr_info[1],1,curr_info[1]+curr_info[0]);
                QPointF int1;
                line.intersect(ray,&int1);
                first_point = int1;
            }
        }
        else if( cond == 2 ){
            //Condition for forward point versus previous line

            //Gets angles and estimates a middle angle from the previous function
            double ray_angle = (*angles)[end_line];
            double next_ray_angle = (*angles)[end_line+1];
            double middle_angle = (ray_angle + next_ray_angle) / 2;

            if( curr_info[3] > 3 ){
                //Gets intersect for the ray and the middle ray on the predicted line
                QLineF ray = QLineF((*p_cloud)[end_line],location);
                QLineF middle_ray = QLineF(location.x(),location.y(),location.x()+cos(middle_angle),location.y()+sin(middle_angle));
                QLineF line = QLineF(0,curr_info[1],1,curr_info[1]+curr_info[0]);
                QPointF int1;
                QPointF int2;
                line.intersect(ray,&int1);
                line.intersect(middle_ray,&int2);

                //Determines error based on the difference in angle intersections
                double angle_error = sqrt(pow(int1.x() - int2.x(),2) + pow(int1.y() - int2.y(),2));

                if(verts->size() != 0){
                    if(first_point == verts->last().get_loc()){
                        s_lines->append(QLineF(first_point,int2));
                    }
                }
                //Finds possible error in vertex and appends new vertex
                double var = find_var(int2,line,curr_info[2]+angle_error+radius,middle_ray,radius);
                verts->append(Vertex(int2,var+buffer));
            }
            else{
                double angle_error = next_ray_angle - middle_angle;
                if(verts->size() != 0){
                    if(first_point == verts->last().get_loc()){
                        s_lines->append(QLineF(first_point,(*p_cloud)[end_line]));
                    }
                }
                //Error in this case is distance error, angle error, radius, and buffer so all terms are bounded
                verts->append(Vertex((*p_cloud)[end_line], radius + buffer + angle_error + D_ERROR*e_distance(location,(*p_cloud)[end_line])));
            }
        }

        //Updates start and end of line positions
        end_line++;
        start_line = end_line;

    }while( cond != 0 );

    if( (angles->last())-(angles->first()) > 1.8*M_PI){
        //Connects the first and last verticy if initial scan
        double dist = point_to_line(verts->first().get_loc(),QLineF(0,curr_info[1],1,curr_info[1]+curr_info[0]));
        if( dist < radius ){
            s_lines->append(QLineF(verts->last().get_loc(),verts->first().get_loc()));
        }
    }
}

QVector<double> Robot::pick_out_line(QVector<QPointF> * p_cloud, unsigned * start, unsigned * end, unsigned max, char * cond ){

    unsigned length = 1;
    QPointF mean = (*p_cloud)[*start];
    if( *cond == 1 ){
        (*start)--;
        length++;
        mean = mean/2 + (*p_cloud)[*end]/2;
    }
    *cond = 1;
    QVector<double> curr_info = QVector<double>({0,0,0,1});
    QVector<double> prev_info = QVector<double>({0,0,0,1});
    QVector<QPointF> section;
    double avg_err = 0;
    double err_diff = 0;
    double tot_err = 0;

    //Iterates through until the point selection no longer forms a viable line
    do{
        //Determines if there are any jumps in the data, and if so, ends the current line there
        double dist = e_distance((*p_cloud)[*end],(*p_cloud)[*end+1]);
        double at_end = e_distance(location,(*p_cloud)[*end]);
        double after_end = e_distance(location,(*p_cloud)[*end+1]);

        if( dist > 2*radius && dist > D_ERROR*at_end*2 && dist > D_ERROR*after_end*2){
            if( after_end > at_end ){
                *cond = 2;
            }
            else{
                *cond = 3;
            }
            return curr_info;
        }

        //Determines the current length of the set
        (*end)++;
        length++;
        //Updates the middle point and error direction
        mean = mean*(length-1)/length + (*p_cloud)[*end]/length;
        double angle_for_error = atan2(mean.y() - location.y(),mean.x() - location.x());
        double dir_err = pow(fabs(sin(angle_for_error)/cos(angle_for_error)),2);
        //Finds the regression characteristics for the set
        section.resize(length);
        std::copy(p_cloud->begin() + *start,p_cloud->begin() + *end + 1,section.begin());
        prev_info = curr_info;
        curr_info = line_info(section,length,mean.x(),mean.y(),dir_err);

        //If iterates past the end of the array, gets the current line regression info and breaks it off
        if( *end + 1 == max ){
            break;
        }

        //Determines amount of distance error that is applicable based on perpendicularity
        tot_err = D_ERROR*2*e_distance(mean,location)*sqrt(fabs(sin(angle_for_error - atan2(curr_info[0],1))));

        //Use integrated terms to bound error
        if( length > 3){
            avg_err = avg_err/2 + prev_info[2]/2;
            if( fabs(prev_info[2] - avg_err) > err_diff ){
                err_diff = fabs(prev_info[2] - avg_err);
            }
            //Originally C = 2
            tot_err = (avg_err+err_diff+2)*(1-sqrt(1/((double)length-3))) + tot_err*sqrt(1/((double)length-3));
        }
    }while( curr_info[2] < tot_err );

    if( *end + 1 == max ){
        *cond = 0;
    }
    else{
        (*end)--;
    }

    return prev_info;
}

//Performs orthogonal regression for line estimation
QVector<double> Robot::line_info(QVector<QPointF> points, double length, double mean_of_x, double mean_of_y, double err){

    if( length < 2 ){
        return QVector<double>({0,0,0,1});
    }
    //Calculate varx-vary and cov for slope and intercept determinations
    double w = 0;
    double r = 0;
    for( QVector<QPointF>::iterator iter = points.begin() ; iter != points.end() ; iter++){
        w += pow((iter->y() - mean_of_y),2)-err*pow((iter->x()-mean_of_x),2);
        r += 2*(iter->y() - mean_of_y)*(iter->x()-mean_of_x);
    }
    //We use one part regression, one front to back point to determine the slope
    //We add the normal of the
    double slope = (w + sqrt(pow(w,2)+err*pow(r,2)))/(r);
    double intercept = mean_of_y - slope*mean_of_x;
    //Determine maximum error
    double max_error = 0;
    for( QVector<QPointF>::iterator iter = points.begin() ; iter != points.end() ; iter++){
        double part = fabs((-slope)*iter->x() + iter->y() - intercept);
        double curr_error = part/sqrt(pow(slope,2)+1);
        if( curr_error > max_error){
            max_error = curr_error;
        }
    }

    return QVector<double>({slope,intercept,max_error,length});
}

//Finds the variance for
double Robot::find_var(QPointF vert, QLineF line1, double var1, QLineF line2, double var2){

    //Finds a normal error buffer for an identified solid line
    double towards_rob = atan2(location.y() - vert.y(),location.x() - vert.x());
    double norm1 = norm_angle(towards_rob,line1); double norm2;
    QLineF trans_line1 = line1.translated(var1*cos(norm1),var1*sin(norm1));

    if( var2 == radius ){
        //Find point on line1 that isn't vert
        double towards_vert = atan2(vert.y() - location.y(),vert.x() - location.x());
        double help_angle = norm_angle(towards_vert,QLineF(0,0,cos(norm1),sin(norm1)));
        norm2 = norm_angle(help_angle,line2);
    }
    else{
        norm2 = norm_angle(towards_rob,line2);
    }

    QLineF trans_line2 =line2.translated(var2*cos(norm2),var2*sin(norm2));

    //identifies the intersection between these buffers and returns this as error
    QPointF intersect;
    trans_line1.intersect(trans_line2,&intersect);
    double error = e_distance(vert,intersect);

    /*
    if( var2 == radius ){
        //Identifies the possible collision point against undetected object and adds variance to get around it
        line1.intersect(trans_line2,&intersect);
        double error2 = e_distance(vert,intersect);
        if( error2 > error){
            return error2;
        }
        else{
            return error;
        }
    }*/
    return error;

}
