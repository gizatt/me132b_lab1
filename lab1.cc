/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower
 *    Matthew Dughi, Tiffany Huang, Gregory Izatt
 *
 *  Revision History:
 *   Gregory Izatt  20130507  Init revision, based on Lab2 code from ME132a
 *
 *
 *
 * Description:
 *    Given a Pioneer 2 robot system, equiped with two primary wheels,
 * a forward-facing stereo camera, and horizontal laser scanner, in a 
 * world containing a polygonal obstacle of reasonable size that is
 * circumnavigable, and given that the Pioneer robot starts with the
 * obstacle in view, the robot will attempt to navigate around the
 * obstacle by holding it on its left side at a reasonable distance.     
 *
 *
 ****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <vector>

#include "cmdline_parsing.h"
#include "common_functions.h"

#define MAX_RANGE_DIST (2.0)
#define TARG_SPEED (0.6)

using namespace PlayerCc;
using namespace std;

/* Given range data / bearing data (of length n), figures out what
    movements to perform, and stores them in speed / turnrate. Returns
    nonzero values in speed and turnrate (always makes robot do
    something) and true if everything is logically OK, or sets speed and
    turnrate to zero and returns false if we somehow get dead-ended
    or reach some logically invalid state. */
bool figure_out_movement(double * speed, double * turnrate,
    vector<double> range_data, vector<double> bearing_data, unsigned int n) {

    /* for all points in visible range, consider a 1/r^2 contribution
       along the direction of that point -> next point if next point is
       also valid, with attraction if farther than av desired dist
       or repulsion if closer */
    int i;
    vector<double> dir(2); dir[0] = 0; dir[1] = 0;
    double total_contribs = 0;
    for (i=0; i<n-1; i++){
        /* If this point & next point are valid dist */
        if (range_data[i] < MAX_RANGE_DIST && range_data[i] > 0 
            && range_data[i+1] < MAX_RANGE_DIST && range_data[i+1] > 0) {
            /* Weight things that are on the right very low unless
               they're really close & relevant */
            double weight = 1.0;
            if (bearing_data[i] < 0.0 && range_data[i] > 1.0){
                weight = 0.01;
            }
            
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
               n1 * (1.0 - range_data[i]) / pow(range_data[i], 4) * weight;
            dir[1] += 
               n2 * (1.0 - range_data[i]) / pow(range_data[i], 4) * weight;
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
    
    /* That's the direction we want to go. Speed is total forward magnitude,
       turnrate is the angle it deviates. */

    /* placeholder; wall follow logic here */
    *speed = dir[0];
    *turnrate = dir[1] + 0.2;
    if (*speed == 0) {
        *speed = 0.3;
        *turnrate = 0.2;
    }
    printf("Speed: %f, turn: %f\n", *speed, *turnrate);
    return true;
    
}

/* Main loop. Parses arguments, connects to player using port from arguments,
    and repeatedly acquires state, calls function to figure out movement,
    and applies that movement, writing info about all that out to logs
    every iter. */
int main(int argc, char **argv)
{
    /* Calls the command line parser */
    parse_args(argc, argv);
    
    try {
        /* Initialize connection to player */
        PlayerClient robot(gHostname, gPort);
        Position2dProxy pp(&robot, gIndex);
        LaserProxy lp(&robot, gIndex); 

        int num_attempts = 20;
        if(!check_robot_connection(robot, pp, 20)) 
            exit(-2);
  
        /* Open a log file with name based on current time */
        char filename[150];
        time_t now = time(NULL);
        strftime(filename, 150, "%Y%m%d.%H.%M_log.csv", gmtime(&now));
        FILE * log_file = fopen ( filename, "w" );        

        /* Initialize speed and turnrate */
        double speed = 0.0;
        double turnrate = 0.0;
        /* And a time counter */
        int time_step = 0;
        /* And loop val */
        bool not_done = true;
        /* Start main processing loop: */
        while(not_done) {
            // read from the proxies
            robot.Read();
        
            /* Read in laser scan range & bearing data */     
            unsigned int n = lp.GetCount();
            vector<double> range_data(n);
            vector<double> bearing_data(n);
            for(uint i=0; i<n; i++) {
                range_data[i] = lp.GetRange(i);
                bearing_data[i] = lp.GetBearing(i);
            }
    
            /* Get our supposed pose information */
            double xpos = pp.GetXPos();
            double ypos = pp.GetYPos();
            double thetapos = pp.GetYaw();      
            
            /* If this is a multiple-of-five timestep, log out pose in csv:
               [ poseX, poseY, poseTheta ] */
            if (time_step%5==0){
                //Write out to our log file
                fprintf(log_file, "%f, %f, %f", xpos, ypos, thetapos);
                fprintf(log_file, "\n");
                fflush(log_file);
            }
            time_step++;
            
            /* Double check if there are any small laser scan values indicating 
                that we're impacting or about to impact an object. Abort if 
                that is true. */
            for (int i=0; i<n; i++){
                if (range_data[i] < 0.1){
                    printf("Impact detected at bearing %f! Abort!\n", 
                           bearing_data[i]);
                    not_done = false;
                }
            }

            /* Update movement */
            if (not_done){
                not_done = figure_out_movement(&speed, &turnrate, range_data, 
                                               bearing_data, n);
            } else {
                speed = 0.0;
                turnrate = 0.0;
            }
            pp.SetSpeed(speed, turnrate);
        }
        
        /* Close up log file */
        fclose(log_file);
    
    } catch(PlayerError e) {
        write_error_details_and_exit(argv[0], e);
    }    
}
