/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower Constants, etc
 *           Matthew Dughi, Tiffany Huang, Gregory Izatt
 *****************************************************************************/

/* Maintain the occupancy grid? */
#define USE_OCCUPANCY_GRID 1
/* Max range we "see" when updating occ grid */
#define OCC_MAX_RANGE (6.0)
/* Min range from obstacle in meters*/
#define DANGER_MAX_THRESH (0.3)
/* Max range from obstacle we want to be, in meters */
#define TRAVERSE_MAX_THRESH (0.5)
/* Theta search range */
#define THETA_SEARCH_BEGIN (-0.3)
#define THETA_SEARCH_END (0.3)
#define THETA_SEARCH_DTHETA (0.01)
/* Step distance in meters along path trace; must be < cell size */
#define THETA_PATHTRACE_STEP (0.01)
/* Percent of final long-path to take before stopping */
#define THETA_PATHTRACE_FINALDISTMOD (0.9)  
/* And max distance we move at once */
#define THETA_PATHTRACE_MAXDIST (0.3)
 
/* Inds into pose array for xpos, ypos, yaw */
#define POSE_X 0
#define POSE_Y 1
#define POSE_YAW 2

/* Max range we "see" when doing vector field */
#define MAX_RANGE_DIST (1.0)
/* Target movement speed when we're done figuring out movement */
#define TARG_SPEED (0.3)

#define PI (3.141592653)
