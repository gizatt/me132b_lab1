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

    /* placeholder; wall follow logic here */
    *speed = 0.5;
    *turnrate = 1.0;
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
