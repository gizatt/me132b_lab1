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
#define TRAVERSE_MAX_THRESH (0.6)
/* Theta search range */
#define THETA_SEARCH_BEGIN (-1.2)
#define THETA_SEARCH_END (1.2)
#define THETA_SEARCH_DTHETA (0.05)
/* Step distance in meters along path trace; must be < cell size */
#define THETA_PATHTRACE_STEP (0.01)
/* Percent of final long-path to take before stopping */
#define THETA_PATHTRACE_FINALDISTMOD (0.7)  
/* And max distance we move at once */
#define THETA_PATHTRACE_MAXDIST (1.5)

/* Inds into pose array for xpos, ypos, yaw */
#define POSE_X 0
#define POSE_Y 1
#define POSE_YAW 2

/* Max range we "see" when doing vector field */
#define MAX_RANGE_DIST (2.0)
/* Target movement speed when we're done figuring out movement */
#define TARG_SPEED (0.3)
/* Weight around pivot */
#define NORMAL_PIVOT_WEIGHT (5.0)
/* Weight toward pivot */
#define TOWARD_PIVOT_WEIGHT (3.0)


/* Search space in grid cells for grid-based vector calc */
#define SEARCH_N (20)
/* Weight given to non-traversable cells for avoidance */
#define ILLEGAL_COMP (8.0)
/* weight given to traversable ones, relative to illegal_comp */
#define TRAV_COMP (0.0)
/* weight given to direction around pivot, relative to 1 (as components
   from all neihboring cells are normalized first */
#define AROUND_COMP (0.5)

#define PI (3.141592653)
