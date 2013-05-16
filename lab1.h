/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower Constants, etc
 *           Matthew Dughi, Tiffany Huang, Gregory Izatt
 *****************************************************************************/

/**** ALGORITHM CHOICE ****/
/* Use vector field approach, or use occupancy grid-based explicit planning
   approach? The former is a prototype and is still undergoing tuning; the
   former was demo'd to function (using constants described below) on
   20130514. */
#define USE_VECTOR_FIELD (false)
/* Print global map when maps are being printed? */
#define PRINT_GLOBAL_MAP (true)
/* Print local map when maps are being printed? */
#define PRINT_LOCAL_MAP (true)

/* MISC GENERAL CONSTANTS */
/* Inds into pose array for xpos, ypos, yaw, used in lab.c */
#define POSE_X 0
#define POSE_Y 1
#define POSE_YAW 2
#define PI (3.141592653)

/**** MOVEMENT PRIMITIVES CONSTANTS ****/
/* range of acceptable turn rates for the robot */
#define MIN_TURN_RATE 0.3
#define MAX_TURN_RATE 0.5
#define SPEED_EPS 0.01
#define ANGLE_EPS 0.01
/* Being any closer to a wall than this freaks out robot */
#define MIN_WALL_DIST 0.1
/* Max speed allowed */
#define MAX_SPEED 0.5

/**** MOVEMENT STATE MACHINE CONSTANTS ****/
/* max errors we'll allow in dist and theta (for moving to
   a point and turning 360*) before changing state */
#define DIST_ERROR (.08)
#define THETA_ERROR (.05)

/**** OCCUPANCY GRID MAINTENANCE CONSTANTS ****/
/* Maintain the occupancy grid? */
#define USE_OCCUPANCY_GRID 1
/* Max range we "see" when updating occ grid */
#define OCC_MAX_RANGE (6.0)

/**** OCCUPANCY GRID NAVIGATION CONSTANTS *****/

/* If we're out of traversable space, and we want to navigate back, this
   is the CLOSEST we'll want our target point to be. Given a perfect
   robot, 0 is the obvious choice here, but our move_to_point doesn't handle
   very small movements well, so making this big ensures we don't get caught
   in a cycle of trying to move to a really close point, failing, trying
   again, etc... */
#define GET_CLOSEST_TRAVERSE_TOO_CLOSE_THRESH (0.2)

/* Min range we will traverse around obstacles, in meters. Robot will never
   navgiate inside of this distance to an obstacle; if it finds itself too
   close, it navigate outward first to be safe. */
	/* fits throw roughly 3 foot gap in the L, but cuts corners fairly tightly */
	#define DANGER_MAX_THRESH (0.25) 
	/* does not pass the 3 foot gap, instead navigate around it. Has a tendency to
	   sometimes decide to follow other closeby obstacles, depending on choice of
	   pivot. */
	//#define DANGER_MAX_THRESH (0.4)

/* Max range from obstacle we want to be, in meters: always 0.25 higher
   than danger thresh to give a nice but reasonably small window of
   preferred distance. Robot will not navigate outside of this distance
   from an obstacle; if it is outside of this distance, it'll path back
   inside a safe distance before doing anything else. */
	/* good for navigating through 3-foot gap */
	#define TRAVERSE_MAX_THRESH (0.50)
	/* Good for passing the gap instead of taking it */
	//#define TRAVERSE_MAX_THRESH (0.65)

/* Search range defining out broad the headings from "ideal" (i.e. around
   pivot) the robot looks for a path. Leaving these fairly broad is OK,
   because the robot does not rank superlong paths over relatively
   long paths closer to the correct heading. However, that logic still
   needs tuning to make this robot work better than it presently does! */
#define THETA_SEARCH_BEGIN (-2.0)
#define THETA_SEARCH_END (2.0)
#define THETA_SEARCH_DTHETA (0.05)
/* Step distance in meters along path trace; must be < cell size */
#define THETA_PATHTRACE_STEP (0.01)
/* Percent of final long-path to take before stopping; once final
   destination is chosen we go this % of the way to that destinatino.
   This helps us avoid pathing right into a tight corner or noise in
   the boundaries of our legal path. */
#define THETA_PATHTRACE_FINALDISTMOD (0.8)
/* And max distance we move at once, before distmod is applied */
#define THETA_PATHTRACE_MAXDIST (0.8)

/* When we've chosen our final point, we step it out from the closest
   obstacle at max this many times, outward across grid cells. Our
   algorithm really loves to stop precisely on or near walls; this
   combats this. */
#define MAX_STEP_ITER 10


/**** OUT-OF-DATE or UNUSED CONSTANTS ****/

/*** VECTOR FIELD CONSTANTS ****/
/* Max range we "see" when doing vector field */
#define MAX_RANGE_DIST (3.0)
/* Target movement speed when we're done figuring out movement */
#define TARG_SPEED (0.1)
/* Weight around pivot */
#define NORMAL_PIVOT_WEIGHT (5.0)
/* Weight toward pivot */
#define TOWARD_PIVOT_WEIGHT (3.0)

/*** OCCUPANCY-GRID-BASED VECTOR FIELD CONSTANTS ****/
/* Note: these still need massive tuning & fixing! */
/* Search space in grid cells for grid-based vector calc */
#define SEARCH_N (20)
/* Weight given to non-traversable cells for avoidance */
#define ILLEGAL_COMP (8.0)
/* weight given to traversable ones, relative to illegal_comp */
#define TRAV_COMP (0.0)
/* weight given to direction around pivot, relative to 1 (as components
   from all neihboring cells are normalized first */
#define AROUND_COMP (0.5)
