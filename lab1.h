/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower Constants, etc
 *           Matthew Dughi, Tiffany Huang, Gregory Izatt
 *****************************************************************************/

/* Maintain the occupancy grid? */
#define USE_OCCUPANCY_GRID 1
/* Max range we "see" when updating occ grid */
#define OCC_MAX_RANGE (6.0)

/* Inds into pose array for xpos, ypos, yaw */
#define POSE_X 0
#define POSE_Y 1
#define POSE_YAW 2

/* Max range we "see" when doing vector field */
#define MAX_RANGE_DIST (1.0)
/* Target movement speed when we're done figuring out movement */
#define TARG_SPEED (0.3)

#define PI (3.141592653)
