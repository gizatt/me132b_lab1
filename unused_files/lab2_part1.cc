// General include files
#include <vector>
// Player related files
#include <libplayerc++/playerc++.h>
#include "cmdline_parsing.h"
#include "common_functions.h"
// Local library files
#include "lab2_shared.h"

using namespace PlayerCc;
using namespace std;

// This defines when we are "close enough" to the goal point
#define DIST_EPS    0.05

int main(int argc, char **argv)
{
    // Calls the command line parser
    parse_args(argc, argv);

    // Get the list of points to go to
    vector<Point> p_list = read_points();
    int num_points = p_list.size();
    int idx = 0;
    // Get first point to go to, or exit if none
    if (num_points == 0)  {return 0;}   // Exit when done
    Point curr_goal = p_list[0];

    try
    {
        // Initialize connection to player
        PlayerClient robot(gHostname, gPort);
        Position2dProxy pp(&robot, gIndex);
        LaserProxy lp(&robot, gIndex);
        int num_attempts = 20;
        if(!check_robot_connection(robot, pp, 20))
        {
            exit(-2);
        }
        // Start main processing loop
        while(true)
        {
            // Read from the proxies
            robot.Read();
            Pose robot_pose = Pose(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());
            // Check if close enough to destination and move to next point if yes
            if (curr_goal.distance_to(robot_pose) < DIST_EPS)
            {
                idx++;
                if (idx == num_points)  {break;} // Exit when done
                curr_goal = p_list[idx];
            }
            
            // Query the laserproxy to gather the laser scanner data
            unsigned int n = lp.GetCount();
            vector<LaserData> data(n);
            for(unsigned int i=0; i<n; i++)
            {
                data[i] = LaserData(lp.GetRange(i), lp.GetBearing(i));
            }
            // Check if close enough to destination and move to next point if yes
            if (curr_goal.distance_to(robot_pose) < DIST_EPS)
            {
                idx++;
                if (idx == num_points)  {break;} // Exit when done
                curr_goal = p_list[idx];
            }
            
            // Check if next to a wall, if not, proceed
            if (!next_to_wall(data, MIN_WALL_DIST))
            {
                // Move towards current point
                double r_dot, theta_dot;
                go_to_point(curr_goal.x, curr_goal.y, robot_pose.x, robot_pose.y, 
                            robot_pose.theta, &r_dot, &theta_dot);
                pp.SetSpeed(r_dot, theta_dot);
            }
            // Otherwise, stop so we don't hit anything
            else
            {
                pp.SetSpeed(0.0, 0.0);
            }

        }
    }
    catch(PlayerError e)
    {
        write_error_details_and_exit(argv[0], e);
    }

    pp.SetSpeed(0.0, 0.0);
    return 0;
}
