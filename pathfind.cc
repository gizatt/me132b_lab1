/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower Pathfinding
 *         Matthew Dughi, Tiffany Huang, Gregory Izatt
 *
 *  Revision History:
 *   Gregory Izatt  20130509  Init revision, bringing code over from lab1.cc
 *   Tiffany Huang  20130509  Added go_to_point function
 *
 * Description:
 *    Given a laser scan and an occupancy grid, figures out what movement
 *  to make, in terms of speed forward, and rotation, to execute.   
 *
 *
 ****************************************************************************/
 
#include <stdio.h>
#include <math.h>
#include <vector>

#include "lab1.h"
#include "common_functions.h"
#include "occupancy_grid.h"
#include "pathfind.h"

// declare state constants
const int start = 0;
const int scanning = 1;
const int move = 2;
const int get_point = 3;
 
// declare constants used for robot odometry
const double dist_error = .02;
const double theta_error = .05;
 
/* Prototypes */
bool apply_vector_field_old(vector<double> range_data,
   vector<double> bearing_data, unsigned int n, vector<double>& dir);

int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot);
                
int turn(double goal_theta, int turn_dir, double robot_theta,
                double* r_dot, double* theta_dot);
                
void findPoint(double * goals);
 
/* Given range data / bearing data (of length n), figures out what
    movements to perform, and stores them in speed / turnrate. Returns
    nonzero values in speed and turnrate (always makes robot do
    something) and true if everything is logically OK, or sets speed and
    turnrate to zero and returns false if we somehow get dead-ended
    or reach some logically invalid state. */
bool figure_out_movement(double * speed, double * turnrate, int * state,
    vector<double> range_data, vector<double> bearing_data, unsigned int n,
    double * pose, double * goals) {
    Point goal;
    double dr;
    
    // Act based on state
    switch(*state) {
        // scan everything
        case start :
             goals[2] = pose[2];
             *turnrate = MAX_TURN_RATE;
             *speed = 0;
             *state = scanning;
             break;
        case scanning : 
            // If we've reached the angle we want, get the
            // point to go to
            if (fabs(pose[2] - goals[2]) <= theta_error) { 
                *state = get_point;
            }            
            break;
        // move to a point
        case move : 
            go_to_point(goals[0], goals[1], pose[0], pose[1], pose[2],
                        speed, turnrate);
            goal = Point(goals[0], goals[1]); // Turn into point for convenience
            // Get distance and direction to goal
            dr = goal.distance_to(pose[0], pose[1]);
            if (dr <= dist_error) {
                goals[2] = pose[2];
                *turnrate = MAX_TURN_RATE;
                *speed = 0;
                *state = scanning;
            }
            break;
        // get a new point and change state to be moving
        case get_point : findPoint(goals); // get the next point
            *state = move;         
            break;
        // something broke
        default : return false;
            break;    
    }
         
    return true;
    
}
void findPoint(double * goals) {
    goals[0] = int(goals[0] + 1) % 6;
    goals[1] = int(goals[1] + 1) % 6;
}
// This function implements a go to function based on inputs of where the robot
// destination is and the current pose. It returns the direction and speed to go
// by reference.
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
/* For all points in visible range, consider a 1/r^2 contribution
   along the direction of that point -> next point if next point is
   also valid, with attraction if farther than a desired dist
   or repulsion if closer */
bool apply_vector_field_old(vector<double> range_data,
   vector<double> bearing_data, unsigned int n, vector<double>& dir){
    int i;
    dir[0] = 0; dir[1] = 0;
    double total_contribs = 0;
    /* in order of decreasing bearing... */
    for (i=n-2; i>0; i--){
        /* If this point & next point are valid dist */
        if (range_data[i] < MAX_RANGE_DIST && range_data[i] > 0 
            && range_data[i+1] < MAX_RANGE_DIST && range_data[i+1] > 0) {
            double weight = pow(bearing_data[i] + 2, 2);
            /* This is a new contribution! Calculate along-edge and
               out-of-edge contributions, scale down by dist^2 */
            total_contribs += weight;
            double x1 = range_data[i]*cosf(bearing_data[i]);
            double y1 = range_data[i]*sinf(bearing_data[i]);
            double x2 = range_data[i+1]*cosf(bearing_data[i+1]);
            double y2 = range_data[i+1]*sinf(bearing_data[i+1]);
            double norm = sqrtf(pow(x1-x2, 2) + pow(y1-y2, 2));
            dir[0] += (x1 - x2)/norm/pow(range_data[i], 4) * weight;
            dir[1] += (y1 - y2)/norm/pow(range_data[i], 4) * weight;
            
            /* As well as contribution along normal toward robot */
            double n1 = (y1 - y2)/norm;
            double n2 = -(x1 - x2)/norm;
            dir[0] += 
               n1 * (.5 - range_data[i]) / pow(range_data[i], 4) * weight;
            dir[1] += 
               n2 * (.5 - range_data[i]) / pow(range_data[i], 4) * weight;
        }
    }
    /* normalize over the n contributions */
    if (total_contribs > 0) {
        /*
        dir[0] /= total_contribs;
        dir[1] /= total_contribs;
        */
        
        /* scale what we care about */
        dir[1] *= 2;
        
        double norm = sqrt(pow(dir[0], 2)+pow(dir[1], 2));
        dir[0] /= norm; dir[1] /= norm;
        dir[0] *= TARG_SPEED; dir[1] *= TARG_SPEED;
    }
    return true;
}
