/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower Pathfinding
 *         Matthew Dughi, Tiffany Huang, Gregory Izatt
 *
 *  Revision History:
 *   Gregory Izatt  20130509  Init revision, bringing code over from lab1.cc
 *   Tiffany Huang  20130509  Added go_to_point function
 *   Team           20130509-15 Continual revision; too many to count...
 * 
 * Description:
 *    Given an up-to-date laser scan and an occupancy grid, figures out what 
 *  movement to make, in terms of speed forward, and rotation, to execute.
 *  There are presently a couple of different models for doing this here. 
 *  Common among them are:
 *
 *   MOVEMENT PRIMITIVES: turn() and go_to_point() can be called at
 *   any time to set speed and turnrate appropriately to continue
 *   executing a given turn or straight-to-point movement.
 *   
 *   FIGURE_OUT_MOVEMENT: main function that calls all others. It is
 *   passed some state info, the occ map, and output variables for
 *   speed and turnrate, and manages a state machine describing what the
 *   robot is trying to do at the moment. The robot starts in an
 *   initialization state, in which is retrieves whatever point is
 *   directly in front of it as a "pivot" that is remembered forever;
 *   it is assumed the robot is started up with this being a point on
 *   our obstacle to go around. If using the vector field approach, it then
 *   just calls our vector field calculation functions to manage the
 *   movement. If not, then it repeatedly (over many calls to this
 *   function) turns 360* to build up the occ map, calls
 *   route_given_occupancy_main() to figure out the next point to path to
 *   given the occupancy map info, and then goes there with go_to_point,
 *   before repeating. This (non-vector field) method is the one we used
 *   in our demo on 20130514.
 *
 * Known issues / TODOs:
 *   - Debug printfs are everywhere; add VERBOSE option?
 *   - Direction-choosing logic in route_given_occupancy_main is still
 *     pretty shaky, and makes poor choices pretty often. Need better 
 *     prioritizing of paths of reasonable (but not max) length close
 *     to around the pivot than paths that are longer but in the wrong
 *     direction. The start of this is there -- just need setting up and
 *     a lot of tuning in real-world experiments or better simulations.
 *
 ****************************************************************************/
 
#include <stdio.h>
#include <math.h>
#include <vector>

#include "lab1.h"
#include "common_functions.h"
#include "occupancy_grid.h"
#include "pathfind.h"

// declare state globals, for navigation
int state = 0;
bool starting_scan = false;
double * goals = new double[3];

// declare state constants
enum robot_states {
 start,
 scanning,
 move,
 get_point,
 using_vector_field
};
 
/* Pivot around which we always try to round in ccw direction */
double pivot[2];

/* Prototypes */
bool apply_vector_field_old(vector<double> range_data,
   vector<double> bearing_data, unsigned int n, vector<double>& dir,
   double * pivot, double * curr_pose);

int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot);
                
int turn(double goal_theta, int turn_dir, double robot_theta,
                double* r_dot, double* theta_dot);
void findPoint(double * goals);
bool route_given_occupancy_main(double curr_pose[2], 
    double return_vert[2], double pivot[2], 
    SimpleOccupancyGrid& oc);
bool route_given_occupancy_vect(double curr_pose[2], 
    double return_vert[2], double pivot[2], 
    SimpleOccupancyGrid& oc);

    
/* Given range data / bearing data (of length n), figures out what
    movements to perform, and stores them in speed / turnrate. Returns
    nonzero values in speed and turnrate (always makes robot do
    something) and true if everything is logically OK, or sets speed and
    turnrate to zero and returns false if we somehow get dead-ended
    or reach some logically invalid state.
    (presently has debug outputs printing what state is being moved
    to when a state transition is set up) */
bool figure_out_movement(double * speed, double * turnrate,
    vector<double> range_data, vector<double> bearing_data, unsigned int n,
    double * pose, SimpleOccupancyGrid& oc, bool use_vector_field) {
    Point goal;
    double dr;
    int i; double theta;
    vector <double> dir(2);
    double dir_pass[2];
    
    // Act based on state
    switch(state) {
        // scan everything and grab out pivot
        case start :
             printf("START STATE\n");
             if (use_vector_field) {
                 printf("VECTOR FIELD STATE\n");
                 /* go start using vector field; do nothing till then */
                 state = using_vector_field;
                 *turnrate = 0;
                 *speed = 0;
             } else {
                 printf("SCAN STATE\n");
                 /* start spinning; goal is present orientation (so turn
                    360*) */
                 goals[2] = pose[2];
                 *turnrate = MAX_TURN_RATE;
                 *speed = 0;
                 state = scanning;
                 starting_scan = true;
             }
             /* and set pivot to whatever's right in front of us */
             i = ceil(n/2);
             theta = pose[2] + bearing_data[i];
		     pivot[0] = pose[0] + cosf(theta) * range_data[i];
		     pivot[1] = pose[1] + sinf(theta) * range_data[i];
             break;
        case using_vector_field:
            /* Non-occupancy-grid-based approach: */
            //apply_vector_field_old(range_data, bearing_data, n, dir, pivot, pose);
            //*speed = dir[0]*TARG_SPEED;
            //*turnrate = dir[1];
            /* Occupancy grid based approach: */
            route_given_occupancy_vect(pose, dir_pass, pivot, oc);
            *speed = dir_pass[0]*TARG_SPEED;
            *turnrate = dir_pass[1];
            
            /* do some modifications to normalize it for physical robot and
               make sure what it's doing isn't unreasonable */
            if (*speed <= 0.05) {
                *speed = 0;
                *turnrate = MAX_TURN_RATE;
            }
            if (*speed > MAX_SPEED)
                *speed = MAX_SPEED;
            if (abs(*turnrate) > MAX_TURN_RATE)
                *turnrate *= MAX_TURN_RATE / *turnrate;

            printf("Vect field: moving [%f, %f]\n", *speed, *turnrate);
            break;
            
        case scanning :
            // If we've reached the angle we want, get the
            // point to go to
            if (!starting_scan && 
                (fabs(pose[2] - goals[2] + 2*PI) <= THETA_ERROR || 
                 fabs(pose[2] - goals[2] - 2*PI) <= THETA_ERROR ||
                 fabs(pose[2] - goals[2]) <= THETA_ERROR )) { 
                state = get_point;
                *speed = 0;
                *turnrate = 0;
            }
            else if(fabs(pose[2] - goals[2]) > THETA_ERROR) { 
                starting_scan = false;
            }
            break;
        // move to a point
        case move : 
            go_to_point(goals[0], goals[1], pose[0], pose[1], pose[2],
                        speed, turnrate);
            goal = Point(goals[0], goals[1]); // Turn into point for convenience
            // Get distance and direction to goal
            dr = goal.distance_to(pose[0], pose[1]);
            if (dr <= DIST_ERROR) {
                goals[2] = pose[2];
                *turnrate = MAX_TURN_RATE;
                *speed = 0;
                starting_scan = true;
                state = scanning;
                printf("SCAN STATE\n");
            }
            break;
        // get a new point and change state to be moving
        case get_point : 
            printf("CALCULATING...\n");
            route_given_occupancy_main(pose, goals, pivot, oc); // get the next point
            state = move;  
            printf("MOVE STATE\n");       
            break;
        // something broke
        default : return false;
            break;    
    }
    return true;
    
}
void findPoint(double * goals) {
    goals[0] = int(goals[0] + 1) % 2;
    goals[1] = 0;
}

/* This function implements a go to function based on inputs of where the robot
   destination is and the current pose. It returns the direction and speed to go
   by reference. */
int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot)
{
    Point goal = Point(goal_x, goal_y); // Turn into point for convenience
    // Get distance and direction to goal
    double dr = goal.distance_to(robot_x, robot_y);
    double dtheta = goal.angle_to(robot_x, robot_y, robot_theta);
    // Don't move if more than 22.5 degrees off, otherwise scale speed with log
    // of angle but cap at dr to prevent overshoot
    *r_dot = (fabs(dtheta) < PI/8) * dr * -log(fabs(dtheta)) / 10.0;
    *r_dot = (dr < *r_dot) ? dr : *r_dot;
    *r_dot = (MAX_SPEED < *r_dot) ? MAX_SPEED : *r_dot;
    // Turn angle is simply direction offset
    *theta_dot = dtheta / 2;    // Slow rotation to prevent overshoot
    
    // Check that the turn rate is greater than the minimum if turning in place
    if (fabs(*theta_dot) < MIN_TURN_RATE && fabs(*r_dot) < SPEED_EPS &&
        fabs(*theta_dot) > ANGLE_EPS)
    {
        // Set to min turn rate if below
        *theta_dot = MIN_TURN_RATE * ((*theta_dot > 0.0) * 2 - 1);
    }
    return 0;   // Dummy return value
}

/* This function sets a turn rate to make the robot turn to a desired angle. */
int turn(double goal_theta, int turn_dir, double robot_theta,
                double* r_dot, double* theta_dot)
{
    double d_theta, d_theta_2;
    d_theta = goal_theta;
    // Don't move

    *r_dot = 0;
    
    // Get change in theta
    // First, make robot_theta positive to make math easier
    if (robot_theta < 0)
        robot_theta = (2 * PI + robot_theta);
        
    // Make the goal angle positive to make math easier
    if (goal_theta < 0)
        goal_theta = 2 * PI + goal_theta;
    
    d_theta = fabs(goal_theta - robot_theta);
    // Deal with 0/2pi boundary 
	if (goal_theta > robot_theta) d_theta_2 = fabs(goal_theta - robot_theta - 2*PI);
	else d_theta_2 = fabs(goal_theta - robot_theta + 2*PI);
	if (d_theta > d_theta_2) d_theta = d_theta_2;

    *theta_dot = turn_dir * d_theta / 2;    // Slow rotation to prevent overshoot
    
    // Check that the turn rate is greater than the minimum if turning in place
    if (fabs(*theta_dot) < MIN_TURN_RATE && fabs(*r_dot) < SPEED_EPS &&
        fabs(*theta_dot) > ANGLE_EPS)
    {
        // Set to min turn rate if below
        *theta_dot = MIN_TURN_RATE * ((*theta_dot > 0.0) * 2 - 1);
    }
    return 0;   // Dummy return value
}


/* Given a current position, a pivot, and an up-to-date occupancy grid,
   figure next vertex to visit. Renders c-occupancy grid (which contains
   state about how what locations we want to be in), and then, if we're
   not presently in a traversable location, routes to closest traversable
   position; otherwise, figures out direction we'd like to go (by using
   helper method with occupancy grid -- see occupancy.cc) and then
   picking a point as far out in that general direction as we can. (We
   scan a bunch of potential things in that general direction +/- 
   scanning theta bounds set in lab1.h, and pick the one closest to
   the direction we want to go that's as big as our max desired
   distance to traverse. This demands more tuning in the future...) */
bool route_given_occupancy_main(double curr_pose[2], 
    double return_vert[2], double pivot[2], 
    SimpleOccupancyGrid& oc){
    /* Update c-occupancy grid */
    oc.updateCGrid(DANGER_MAX_THRESH, TRAVERSE_MAX_THRESH);
    /* If our present position isn't traversable... */
    if (oc.cgrid_state(curr_pose) <= 0){
        /* Find closest traversable point to current pose and go there */
        if (!oc.get_closest_traversable(curr_pose, return_vert))
            return false;
        printf("Retreating to %f, %f\n", return_vert[0], return_vert[1]);
        if (PRINT_GLOBAL_MAP)
            oc.printPPM(78, 30, curr_pose, true);
        if (PRINT_LOCAL_MAP)
            oc.printPPM_local(78, 30, return_vert, curr_pose, true);
        return true;
    }
    /* We're in reasonable territory, so figure out our direction
       we kind of want to head */
    double dir_to_go[2];
    if (!oc.get_next_dir(curr_pose, pivot, dir_to_go)) return false;
    double orig_theta = atan2(dir_to_go[1], dir_to_go[0]);
    /* Now that we have the direction, step over theta range we care
       for offsetting from that dir */
    double theta;
    double best_dist_so_far = -1;
    double final_pos[2];
    for (theta=THETA_SEARCH_BEGIN; theta < THETA_SEARCH_END;
         theta+=THETA_SEARCH_DTHETA){
        /* In this direction offset from  the suggested next direction,
           scan along until we hit a not-traversable block; if dist between
           final position and current position is bigger than we've found
           so far, this is our new best candidate for point to move to. */
        dir_to_go[0] = cosf(orig_theta + theta);
        dir_to_go[1] = sinf(orig_theta + theta);
        double i = 0;
        double curr_pos[2];
        do {
            curr_pos[0] = curr_pose[0] + dir_to_go[0]*i;
            curr_pos[1] = curr_pose[1] + dir_to_go[1]*i;
            i += THETA_PATHTRACE_STEP;
        } while (oc.cgrid_state(curr_pos) > 0.0 && i < THETA_PATHTRACE_MAXDIST);
        i -= THETA_PATHTRACE_STEP;
        if (i > best_dist_so_far){
            best_dist_so_far = i;
            i = i > THETA_PATHTRACE_MAXDIST ? THETA_PATHTRACE_MAXDIST : i;
            final_pos[0] = curr_pose[0] 
                         + dir_to_go[0]*i*THETA_PATHTRACE_FINALDISTMOD;
            final_pos[1] = curr_pose[1] 
                         + dir_to_go[1]*i*THETA_PATHTRACE_FINALDISTMOD;
        }
    }
    /* Given final position, step outward from the parent point as much as we
       can while preserving who the parent point is */
    oc.increase_dist_to_parent(final_pos);
    return_vert[0] = final_pos[0];
    return_vert[1] = final_pos[1];
    printf("Pathing to %f, %f\n", return_vert[0], return_vert[1]);
    if (PRINT_GLOBAL_MAP)
        oc.printPPM(78, 30, curr_pose, true);
    if (PRINT_LOCAL_MAP)
        oc.printPPM_local(78, 30, return_vert, curr_pose, true);
    return true;
}


/* Given a current position, a pivot, and an up-to-date occupancy grid,
   figure next vertex to visit. Renders c-occupancy grid (which contains
   state about how what locations we want to be in), and then, if we're
   not presently in a traversable location, routes to closest traversable
   position; otherwise, figures out direction we'd like to go (by using
   occupancy grid helper for vector field method -- see occupancy.cc */
bool route_given_occupancy_vect(double curr_pose[3], double return_dir[2],
    double pivot[2], SimpleOccupancyGrid& oc){
    /* Update c-occupancy grid */
    double tmp[3] = {-1000, -1000, -1000};
    oc.updateCGrid(DANGER_MAX_THRESH, TRAVERSE_MAX_THRESH);
    if (PRINT_GLOBAL_MAP)
        oc.printPPM(78, 30, curr_pose, true);
    if (PRINT_LOCAL_MAP)
        oc.printPPM_local(78, 30, tmp, curr_pose, true);
    /* If our present position isn't traversable... */
    double dir[2];
    if (oc.cgrid_state(curr_pose) <= 0){
        /* Find closest traversable point to current pose and go there */
        double tmp_vert[2];
        if (!oc.get_closest_traversable(curr_pose, dir))
            return false;
        /* that's a vertex; need to return direction to it in local frame
           of robot. */
        dir[0] -= curr_pose[0];
        dir[1] -= curr_pose[1];
        double tmp = sqrtf(pow(dir[0], 2) + pow(dir[1], 2));
        dir[0] /= tmp;
        dir[1] /= tmp;
    } else {
        /* We're in reasonable territory, so figure out our direction
           we kind of want to head, using vector field method on cgrid */
        if (!oc.get_next_dir_vect(curr_pose, pivot, dir)) return false;
        /* now return_dir is a normalized global direction. Put it in local
           coods and we're good */
    }
    return_dir[0] = dir[0]*cosf(-curr_pose[2]) + dir[1]*sinf(curr_pose[2]);
    return_dir[1] = dir[0]*sinf(-curr_pose[2]) + dir[1]*cosf(-curr_pose[2]);
    printf("Final grad: %f, %f\n", return_dir[0], return_dir[1]);
    return true;
}   

/* For all points in visible range, consider a 1/r^2 contribution
   along the direction of that point -> next point if next point is
   also valid, with attraction if farther than a desired dist
   or repulsion if closer. Does not use occupancy grid at all --
   just current laser scan. */
bool apply_vector_field_old(vector<double> range_data,
   vector<double> bearing_data, unsigned int n, vector<double>& dir,
   double * pivot, double * curr_pose){
    int i;
    dir[0] = 0; dir[1] = 0;
    double total_contribs = 0;
    /* in order of decreasing bearing... */
    for (i=n-2; i>0; i--){
        /* If this point & next point are valid dist */
        if (range_data[i] < MAX_RANGE_DIST && range_data[i] > 0 
            && range_data[i+1] < MAX_RANGE_DIST && range_data[i+1] > 0) {
            double weight = 1; /*pow(bearing_data[i] + 2, 2); */
            /* This is a new contribution! Calculate along-edge and
               out-of-edge contributions, scale down by dist^2 */
            total_contribs += 1.5;
            
            
            double x1 = range_data[i]*cosf(bearing_data[i]);
            double y1 = range_data[i]*sinf(bearing_data[i]);
            double x2 = range_data[i+1]*cosf(bearing_data[i+1]);
            double y2 = range_data[i+1]*sinf(bearing_data[i+1]);
            double norm = sqrtf(pow(x1-x2, 2) + pow(y1-y2, 2));
            
            dir[0] += (x1 - x2)/norm/pow(range_data[i], 4) * weight / 2;
            dir[1] += (y1 - y2)/norm/pow(range_data[i], 4) * weight / 2;
            
            
            /* As well as contribution along normal toward robot */
            double n1 = (y1 - y2)/norm;
            double n2 = -(x1 - x2)/norm;
            dir[0] += 
               n1 * (1.0 - range_data[i]) / pow(range_data[i], 4) * weight;
            dir[1] += 
               n2 * (1.0 - range_data[i]) / pow(range_data[i], 4) * weight;
        }
    }
    /* normalize over the n contributions */
    if (total_contribs > 0) {
        
        dir[0] /= total_contribs;
        dir[1] /= total_contribs;
    }
        
    /* Add in direction around pivot */
    /* Invert if it's not going around pivot */
    double diff_pivot[2]; diff_pivot[0] = pivot[0] - curr_pose[0];
                          diff_pivot[1] = pivot[1] - curr_pose[1];
    /* Add component toward pivot */
    double theta = atan2(diff_pivot[1], diff_pivot[0]);
    theta = theta - curr_pose[2];
    dir[0] += TOWARD_PIVOT_WEIGHT*cosf(theta);
    dir[1] += TOWARD_PIVOT_WEIGHT*sinf(theta);
    
    double tmp = diff_pivot[0];
    diff_pivot[0] = diff_pivot[1];
    diff_pivot[1] = -tmp;
    /* now diff_pivot is normal to robot-pivot vector */
    theta = atan2(diff_pivot[1], diff_pivot[0]);
    theta = theta - curr_pose[2];
    
    /* Add on to curr pose and then get a normalized vector in
       this direction */
    dir[0] += NORMAL_PIVOT_WEIGHT*cosf(theta);
    dir[1] += NORMAL_PIVOT_WEIGHT*sinf(theta);
    
    /* scale what we care about */
    dir[1] *= 2;
    
    double norm = sqrt(pow(dir[0], 2)+pow(dir[1], 2));
    if (norm)
        dir[0] /= norm; dir[1] /= norm;
    dir[0] *= TARG_SPEED; dir[1] *= TARG_SPEED;
    
    return true;
}