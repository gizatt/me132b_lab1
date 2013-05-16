/*
 * Occupancy Grid
 *
 * Supplied by ME132b TA's
 *
 * Rev history:
 *  Gregory Izatt    20130509    Import from ME132b site; adding a "print
 *                                occ grid to console" function on a whim
 *  Gregory Izatt    20130509    Adding c-occ grid, which is extended outward
 *                                from regular grid.
 *  Gregory Izatt    20130513    Much addition of helper functions for various
 *                                pathfinding attempts
 *  Gregory Izatt    20130514    ctd
 *
 * Description: Maintains an occupancy grid of specified size, in global
 *  coordinates. Provides accessors and mutators for updating the grid,
 *  retrieving information at specific points, updating a c-space approximation
 *  ("c-grid") based on the occupancy grid, visualizing the c-grid, and
 *  various helper functions for performing pathfinding using the occ grid
 *  and c-grid, which are put in here because they require lots of access
 *  to raw c-grid data.
 * Occupancy grid is roughly probabilistic; it decays over time. Each entry
 *  is a number (initialized to -5.0); subsequent observations of an obstacle
 *  increase this (such that an obstacle is "detected" if the number is >0), but
 *  the grid decays whenever a new sweep is added as well, so obstacles not
 *  repeatedly observed are slowly forgotten (which helps prevent against drift
 *  and a changing environment.)
 * CGrid can have one of three things in each location: CGRID_DANGER, which is
 *  "too close to obstacle"; CGRID_INIT, which is "not close enough to obstacle
 *  to be traversable", or a positive number, which indicates the cell is indeed
 *  traversable and reflects distance to closest cell. For traversable cells,
 *  cgrid_owner[i][2*j] and cgrid_owner[i][2*j+1] contain the i and j coords
 *  of the closest obstacle point, in case you want to use that for something.
 * Known issues / TODO's: 
 *   - Grid initial condition and decay rate currently defined by magic #'s in
 *     code. This should be fixed eventually, it's messy~
 *   - This is generally disorganized and merits some reorganization. Make 
 *     cgrid its own thing, maybe? Better organize divide between occ_grid and
 *     pathfinding proper, at the very least!
 *   - Real rendering (not ASCII rendering) desirable for higher resolution
 *     and no distortion. Even though it's hilarious.
 *   - Debug printfs and things are still around, though they're not too
 *     bad. Eventually add a VERBOSE option or something.
 */

#include <assert.h>
#include <math.h>
#include "occupancy_grid.h"
#include <stdio.h>
#include <fstream>
#include "lab1.h"

/* Constructor; sets sizes and such, and zeros out grid (and cgrid,
   though that isn't strictly necessary) */
SimpleOccupancyGrid::SimpleOccupancyGrid(
	const double lower_left[2],
	const double upper_right[2], 
	double cell_size) 
{
	
	double size[2] = {
		upper_right[0] - lower_left[0],
	 	upper_right[1] - lower_left[1]
	};
	
	assert(size[0] > 0);
	assert(size[1] > 0); 

	this->ncells[0] = (int) ceil(size[0]/cell_size);
	this->ncells[1] = (int) ceil(size[1]/cell_size);
	this->origin[0] = lower_left[0];
	this->origin[1] = lower_left[1];
	this->size[0] = this->ncells[0] * cell_size;
	this->size[1] = this->ncells[1] * cell_size;
	
	for(int i=0;i<this->ncells[0];i++) {
		this->grid.push_back(vector<double>(this->ncells[1]));
		this->cgrid.push_back(vector<double>(this->ncells[1]));
		this->cgrid_owner.push_back(vector<double>(2*this->ncells[1]));
    }
    
	for(int i=0;i<this->ncells[0];i++)
	for(int j=0;j<this->ncells[1];j++){
		this->grid[i][j] = -5.0;
		this->cgrid[i][j] = CGRID_INIT;
    }
	
}

/* Converts world coords to grid coords in grid[] */
bool SimpleOccupancyGrid::world2grid(const double world[2], int grid[2]) const {
	for(int i=0;i<2;i++) {
		double u = (world[i] - this->origin[i] ) / this->size[i]; // \in [0,1]
		grid[i]  = (int) floor(u * this->ncells[i]);
		
		if(grid[i] < 0 || grid[i] >= this->ncells[i])
			return false;
	}
	return true;
}

/* Returns whether specified space is occupied */
bool SimpleOccupancyGrid::occupied(const double world[2]) const {
	
	int c[2];
	if(this->world2grid(world, c)) {
		double value = this->grid[c[0]][c[1]];
		return (value>0);
	} else {
		// outside is occupied
		return true;
	}
	
}

/* Given grid coordinates, marks an obstacle in that space. */
void SimpleOccupancyGrid::markObstacle(const double p[2]) {
	int c[2];
	if(this->world2grid(p, c))
		this->grid[c[0]][c[1]] += 1.0;
}
	
/* Adds a passed scan to the grid proper. */
void SimpleOccupancyGrid::addScan(
	const double pose[3], 
	int n,  
	const double *bearing,
	const double *reading,
	double max_range) 
{
    /* Decay everything first */
	for(int i=0;i<this->ncells[0];i++)
	for(int j=0;j<this->ncells[1];j++) {
		this->grid[i][j] = (this->grid[i][j]*0.99 - 0.03);
    }
    
	for(int i=0;i<n;i++) {
		if( !(reading[i] > 0)  || (reading[i] >= max_range))
			continue;
		
		double theta = pose[2] + bearing[i];
		
		double p[2] = {
			pose[0] + cos(theta) * reading[i],
			pose[1] + sin(theta) * reading[i]
		};
		
		this->markObstacle(p);
	}
		
};

/* Save things out~ */
void SimpleOccupancyGrid::savePPM(const char*filename) const {
	ofstream ofs(filename);
	ofs << "P2" << endl;
	ofs << this->ncells[0] << ' ' << this->ncells[1] << endl;
	ofs << "255" << endl;
	
	for(int i=0;i<this->ncells[0];i++) {
		
		for(int j=0;j<this->ncells[1];j++) {
			int col = this->grid[i][j] ? 0 : 255;
		
			ofs << col << ' ';
		}
	
		ofs << endl;
	}
}

/* Prints out occupancy grid to console: downsamples so it fits in x chars
   wide by y tall. Prints in ASCII, with X representing the player
   location (in pose),  8 representing obstacles we're certain about,
   + representing ones we're pretty sure about, and - being ones
   that are almost decayed. Also prints cgrid overlayed, if print_cgrid
   is passed in as true: # is traversable space, and ! is space that
   is too close to an obstacle to be traversable. Empty space is
   too far from an obstacle (and also not traversable). */
void SimpleOccupancyGrid::printPPM(int x, int y, const double pose[3],
                                   bool print_cgrid) const {
    int x_step = this->ncells[0] / x; x_step = x_step == 0 ? 1 : x_step;
    int y_step = this->ncells[1] / y; y_step = y_step == 0 ? 1 : y_step;
    /* Figure out where pose turns out to be on the grid */
    int grid[2];
    bool draw_player = this->world2grid(pose, grid);
    int i_x, i_y, i_tx, i_ty;
    for (i_y = this->ncells[1]-1; i_y >= 0; i_y -= y_step){
        for (i_x = 0; i_x < this->ncells[0]; i_x += x_step){
            double best_yet = -100;
            double best_yet_cgrid = -100;
            bool this_is_player = false;
            for (i_ty = i_y; i_ty < i_y+y_step && i_ty < this->ncells[1]; 
                 i_ty++){
                for (i_tx = i_x; i_tx < i_x+x_step && i_tx < this->ncells[0]; 
                    i_tx++){
                    if (draw_player && i_ty == grid[1] && i_tx == grid[0]){
                        this_is_player = true;
                        break;
                    }
                    if (this->grid[i_tx][i_ty] > best_yet)
                        best_yet = this->grid[i_tx][i_ty];
                    if (this->cgrid[i_tx][i_ty] == CGRID_DANGER)
                        best_yet_cgrid = CGRID_DANGER;
                    else if (best_yet_cgrid != CGRID_DANGER && 
                             this->cgrid[i_tx][i_ty] > 0)
                        best_yet_cgrid = 1.0;
                }
                if (this_is_player) break;
            }
            if (this_is_player)
                printf("X");
            else if (best_yet > 4)
                printf("8");
            else if (best_yet > 1)
                printf("+");
            else if (best_yet > 0)
                printf("-");
            else if (print_cgrid && best_yet_cgrid > 0)
                printf("#");
            else if (print_cgrid && best_yet_cgrid == CGRID_DANGER)
                printf("!");
            else
                printf(" ");
        }
        printf("\n");
    }
}

/* Prints out local occupancy grid to console, look x/2 cells l and r, and
   y/2 u and d. Prints using same character set as above, with one addition:
   if pose_new is within the world, it'll be drawn in as "O"; this is generally
   used to render a motion plan showing current location as "X", local world
   as described above, and goal as "O". */
void SimpleOccupancyGrid::printPPM_local(int x, int y, const double pose_new[2],
           const double pose[3], bool print_cgrid) const {
    /* Figure out where pose turns out to be on the grid */
    int grid[2];
    bool draw_player = this->world2grid(pose, grid);
    int grid_new[2];
    bool draw_new = this->world2grid(pose_new, grid_new);
    int i_x, i_y;
    if (draw_player){
        for (i_y = y/2; i_y >= -y/2; i_y -= 1){
            for (i_x = -x/2; i_x < x/2; i_x += 1){
                int m = grid[0]+i_x; int n = grid[1]+i_y;
                if (m >= this->ncells[0] || m <= 0 || n >= this->ncells[1] || n <= 0)
                    continue;
                double best_yet = -100;
                double best_yet_cgrid = -100;
                bool this_is_player = false;
                bool this_is_new = false;
                if (i_y == 0 && i_x == 0){
                    this_is_player = true;
                } else if (n == grid_new[1] && m == grid_new[0]){
                    this_is_new = true;
                } else {
                    best_yet = this->grid[m][n];
                    if (this->cgrid[m][n] == CGRID_DANGER)
                        best_yet_cgrid = CGRID_DANGER;
                    else if (best_yet_cgrid != CGRID_DANGER && 
                         this->cgrid[m][n] > 0)
                        best_yet_cgrid = 1.0;
                }
                if (this_is_player)
                    printf("X");
                else if (this_is_new)
                    printf("O");
                else if (best_yet > 4)
                    printf("8");
                else if (best_yet > 1)
                    printf("+");
                else if (best_yet > 0)
                    printf("-");
                else if (print_cgrid && best_yet_cgrid > 0)
                    printf("#");
                else if (print_cgrid && best_yet_cgrid == CGRID_DANGER)
                    printf("!");
                else
                    printf(" ");
            }
            printf("\n");
        }
    }
}


/* Given current state of occupancy grid, recomputes cgrid and cgrid
   neighbors. WARNING: THIS MIGHT TAKE A WHILE. */
void SimpleOccupancyGrid::updateCGrid(double danger_thresh, 
                                      double traverse_max) {
    /* Clear out cgrid */
    for(int i=0;i<this->ncells[0];i++)
	for(int j=0;j<this->ncells[1];j++) {
		this->cgrid[i][j] = CGRID_INIT;
    }
    
    /* For each point in the occupancy grid that is currently set... */
    for(int i=0;i<this->ncells[0];i++) {
	for(int j=0;j<this->ncells[1];j++) {
		if (this->grid[i][j] > 0.0){
		    /* It's set, so try to set all relevant points in cgrid.
		       So, for every point in legitimate range... */
		    for(int m=0;m<this->ncells[0];m++) {
	        for(int n=0;n<this->ncells[1];n++) {
	            /* If it isn't already "danger" */
	            if (this->cgrid[m][n] != CGRID_DANGER) {
	                double dist = sqrtf( 
	                   pow(((double)(m-i))*this->size[0]/this->ncells[0], 2.0f) + 
	                   pow(((double)(n-j))*this->size[1]/this->ncells[1], 2.0f) );
	                if (dist < danger_thresh){
	                    /* not traversable, too close */
	                    this->cgrid[m][n] = CGRID_DANGER;
	                } else if (dist < traverse_max) {
	                    /* traverseable! Set / maintain this state, and if
	                       we're the closest person to have done this,
	                       update us as the owner of this point! */
	                    if (this->cgrid[m][n] == CGRID_INIT || this->cgrid[m][n] > dist){
	                        this->cgrid[m][n] = dist;
	                        this->cgrid_owner[m][2*n] = (double)i;
	                        this->cgrid_owner[m][2*n+1] = (double)j;
	                    }
	                }
	            }
	        }}
		}
    }}
}

/* Return state of given c_grid spot; DANGER if out of bounds. Defined
   in include as either CGRID_INIT, CGRID_DANGER, or positive (being
   a traversable space == distance to closest obstacle.) */
double SimpleOccupancyGrid::cgrid_state(const double world[2]) const {
	int c[2];
	if(this->world2grid(world, c)) {
		return this->cgrid[c[0]][c[1]];
	} else {
		// outside is danger!
		return CGRID_DANGER;
	}
}

/* Get closest traversable grid cell (converted into world space) into
   temp, if any; filters out candidates that are TOO close, given
   constant defined in lab1.h. */
bool SimpleOccupancyGrid::get_closest_traversable(double curr_pose[2], 
       double temp[2]){
    int c[2];
    bool found_one = false;
	if(this->world2grid(curr_pose, c)) {
        /* we're in bounds, so execute least efficient search evar */
        double best_dist = 100000000;
        for(int i=0;i<this->ncells[0];i++) {
	    for(int j=0;j<this->ncells[1];j++) {  
	        if (cgrid[i][j] > 0){
                double dist = sqrtf( 
                   pow(((double)(c[0]-i))*this->size[0]/this->ncells[0], 2.0f) + 
                   pow(((double)(c[1]-j))*this->size[1]/this->ncells[1], 2.0f) );
                /* If this is a better candidate BUT isn't TOO close, go for it. */
                if (dist < best_dist && dist > GET_CLOSEST_TRAVERSE_TOO_CLOSE_THRESH){
	                found_one = true;
                    best_dist = dist;
                    temp[0] = (double)i;
                    temp[1] = (double)j;
                }
            }
	    }}
	    /* If we found one, convert temp to world coords */
	    if (found_one){
	        temp[0] = temp[0]*this->size[0]/this->ncells[0] 
	                  + this->origin[0];
	        temp[1] = temp[1]*this->size[1]/this->ncells[1] 
	                  + this->origin[1];
	    } else {
	        printf("Couldn't find traversable cell.\n");
	        temp[0] = curr_pose[0];
	        temp[1] = curr_pose[1];
	    }
	} else {
		// outside? no closest point.
	    printf("Closest_traversable passed noninternior point.\n");
	    temp[0] = curr_pose[0];
	    temp[1] = curr_pose[1];
	}
    return found_one;
}

/* Given a position that is guaranteed to be in a traversable cell,
   return the direction we'd want to go, which is presently just
   going ccw around the given pivot. Returns true if successful,
   false if not, and updates return_dir with the dir we choose. */
bool SimpleOccupancyGrid::get_next_dir(double * curr_pose, double * pivot,
                                           double * return_dir){
    double diff[2];

    // OLD: added influence from closest obstacle point.
        /* Figure out the idfference between pose and parent point */
        // int c[2];
        // printf("Start: %f, %f -> %f, %f ::: ", diff[0], diff[1], this->cgrid_owner[c[0]][2*c[1]], this->cgrid_owner[c[0]][2*c[1]+1]);
        // if (!this->world2grid(curr_pose, c)) return false;
        // diff[0] = curr_pose[0] - (this->cgrid_owner[c[0]][2*c[1]]
        //                          *this->size[0]/this->ncells[0] + this->origin[0]);
        // diff[1] = curr_pose[1] - (this->cgrid_owner[c[0]][2*c[1]+1]
        //                          *this->size[1]/this->ncells[1] + this->origin[1]);  
        // printf("To closest pt: %f, %f\n", diff[0], diff[1]);  
        // /* Dir in axis we car about is perp to that */
        // double tmp = diff[0];
        // diff[0] = diff[1];
        // diff[1] = -tmp;
        // printf("Norm: %f, %f\n", diff[0], diff[1]);
        /* Invert if it's not going around pivot */

    /* Get vector to pivot: */
    double diff_pivot[2]; diff_pivot[0] = pivot[0] - curr_pose[0];
                          diff_pivot[1] = pivot[1] - curr_pose[1];
    double tmp = diff_pivot[0];
    diff_pivot[0] = diff_pivot[1];
    diff_pivot[1] = -tmp;
    /* now diff_pivot is normal to robot-pivot vector, going ccw around pivot */

    // Again, old
       // if (diff[0]*diff_pivot[0] + diff[1]*diff_pivot[1] < 0){
       //      invert if our direction isn't going that way 
       //     diff[0] *= -1.0;
       //     diff[1] *= -1.0;
       // }

    /* That's simply the direction we want to go */
    diff[0] = diff_pivot[0];
    diff[1] = diff_pivot[1];
    
    printf("Direction to go: %f, %f\n", diff[0], diff[1]);
    /* And normalize */
    double tot = sqrtf( pow(diff[0], 2) + pow(diff[1], 2) );
    diff[0] /= tot;
    diff[1] /= tot;
    return_dir[0] = diff[0];
    return_dir[1] = diff[1];
    return true;
}

/* Given a position that is guaranteed to be in a traversable cell,
   return the direction we'd want to go, going ccw around pivot, by
   weighting contributions from all local cells. Ret true if successful,
   false if not. "Direction we want to go" is as close as we can
   get to "too close" to an obstacle (by following gradient of
   positive c_grid entries, and adding strong contributions away
   from non-traversable points. */
bool SimpleOccupancyGrid::get_next_dir_vect(double * curr_pose, double * pivot,
                                           double * return_dir){
    /* Figure out curr point, and move it inside if it's on a border
       (this was necessary for old method; maybe not anymore?) */
    int c[2];
    if (!this->world2grid(curr_pose, c)) return false;
    if (c[0] == 0) c[0] += 1;
    if (c[0] == this->ncells[0]-1) c[0] -= 1;
    if (c[1] == 0) c[1] += 1;
    if (c[1] == this->ncells[1]-1) c[1] -= 1;
    
    /* For all points... OLD APPROACH, NOW IRRELEVANT
    for(int i=0;i<this->ncells[0];i++) {
    for(int j=0;j<this->ncells[1];j++) {  
        if (cgrid[i][j] > 0){
            found_one = true;
            double dist = sqrtf( 
               pow(((double)(c[0]-i))*this->size[0]/this->ncells[0], 2.0f) + 
               pow(((double)(c[1]-j))*this->size[1]/this->ncells[1], 2.0f) );
            if (dist < best_dist){
                best_dist = dist;
                temp[0] = (double)i;
                temp[1] = (double)j;
            }
        }
    }} */

    /* Figure out "gradient" at current point, looking at surrounding N
       grid cells */
    double grad[2] = {0.0, 0.0};
    double our_dist = this->cgrid[c[0]][c[1]];
    printf("our dist at spot: %f at [%d, %d]\n", our_dist, c[0], c[1]);
    for (int i=-SEARCH_N; i<=SEARCH_N; i++){
        for (int j=-SEARCH_N; j<=SEARCH_N; j++){
            int m = c[0]+i; int n = c[1]+j;
            if (m >= this->ncells[0] || m <= 0 || n >= this->ncells[1] || n <= 0
                || i == 0 || j == 0)
                continue;
            /* Add contribution from this part. If it's not valid, go away */
            if (this->cgrid[m][n] == CGRID_DANGER){
                grad[0] += ILLEGAL_COMP / pow((double)(c[0]-m), 3); grad[1] += ILLEGAL_COMP / pow((double)(c[1]-n), 3);
            }
            /* Otherwise, go torward if it's a lower dist or away if not */ 
            else if (this->cgrid[m][n] != CGRID_INIT) {
                grad[0] += TRAV_COMP * (this->cgrid[m][n] - our_dist)*((double)(c[0]-m))/fabs(((double)c[0]-m));
                grad[1] += TRAV_COMP * (this->cgrid[m][n] - our_dist)*((double)(c[1]-n))/fabs(((double)c[1]-n));
            }
        }
    }
    /* Normalize */
    double tot = sqrtf( pow(grad[0], 2) + pow(grad[1], 2) );
    grad[0] /= tot;
    grad[1] /= tot;
    
    printf("Before pivot norm: %f, %f\n", grad[0], grad[1]);
    
    /* And add component in direction around pivot */
    double diff_pivot[2]; diff_pivot[0] = pivot[0] - curr_pose[0];
                          diff_pivot[1] = pivot[1] - curr_pose[1];
    double tmp = diff_pivot[0];
    diff_pivot[0] = diff_pivot[1];
    diff_pivot[1] = -tmp;
    /* now diff_pivot is normal to robot-pivot vector */
    grad[0] += AROUND_COMP * diff_pivot[0];
    grad[1] += AROUND_COMP * diff_pivot[1];
    
    printf("Final grad: %f, %f ->", grad[0], grad[1]);
    /* And normalize */
    tot = sqrtf( pow(grad[0], 2) + pow(grad[1], 2) );
    grad[0] /= tot;
    grad[1] /= tot;
    printf(" %f, %f \n", grad[0], grad[1]);
    return_dir[0] = grad[0];
    return_dir[1] = grad[1];
    return true;
}

/* Given a traversable cell, steps outward from the closest obstacle
   a hair until we're either "far enough" away (see const in lab1.h),
   or the closest obstacle changes. */
bool SimpleOccupancyGrid::increase_dist_to_parent(double final_pos[2]){
    /* Figure out curr point */
    int c[2];
    if (!this->world2grid(final_pos, c)) return false;
    double parent[2];
    /* set up a parent, a vector away from it, and loop vars */
    parent[0] = this->cgrid_owner[c[0]][2*c[1]];
    parent[1] = this->cgrid_owner[c[0]][2*c[1]+1];
    double i=(double)c[0]; double j=(double)c[1];
    double norm[2];
    norm[0] = c[0] - parent[0]; norm[1] = c[1] - parent[1];
    double tot = sqrtf( pow(norm[0], 2) + pow(norm[1], 2) );
    norm[0] /= tot; norm[1] /= tot;
    int iter = 0;
    while (true){
        /* if we've gone far enough or changed parents, we're done */
        if (iter > MAX_STEP_ITER || parent[0] != this->cgrid_owner[(int)i][(int)j]){
            break;
        }
        /* otherwise step further */
        iter++;
        i += norm[0]; j += norm[1];
    }
    /* set final pos to wherever we wound up */
    final_pos[0] = (i+0.5)*this->size[0]/this->ncells[0]  + this->origin[0];
    final_pos[1] = (j+0.5)*this->size[1]/this->ncells[1] + this->origin[1]; 
}
