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
 */

#include <assert.h>
#include <math.h>
#include "occupancy_grid.h"
#include <stdio.h>
#include <fstream>

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
		this->cgrid[i][j] = -1.0;
    }
	
}

bool SimpleOccupancyGrid::world2grid(const double world[2], int grid[2]) const {
	for(int i=0;i<2;i++) {
		double u = (world[i] - this->origin[i] ) / this->size[i]; // \in [0,1]
		grid[i]  = (int) floor(u * this->ncells[i]);
		
		if(grid[i] < 0 || grid[i] >= this->ncells[i])
			return false;
	}
	return true;
}

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


void SimpleOccupancyGrid::markObstacle(const double p[2]) {
	int c[2];
	if(this->world2grid(p, c))
		this->grid[c[0]][c[1]] += 1.0;
}
	
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
 * wide by y tall */
void SimpleOccupancyGrid::printPPM(int x, int y, const double pose[3],
                                   bool print_cgrid) const {
    int x_step = this->ncells[0] / x; x_step = x_step == 0 ? 1 : x_step;
    int y_step = this->ncells[1] / y; y_step = y_step == 0 ? 1 : y_step;
    /* Figure out where pose turns out to be on the grid */
    int grid[2];
    bool draw_player = this->world2grid(pose, grid);
    int i_x, i_y, i_tx, i_ty;
    for (i_y = 0; i_y < this->ncells[1]; i_y += y_step){
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

/* Return state of given c_grid spot; DANGER if out of bounds */
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
   temp, if any */
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
	    }}
	    /* If we found one, convert temp to world coords */
	    if (found_one){
	        temp[0] = temp[0]*this->size[0]/this->ncells[0] 
	                  + this->origin[0];
	        temp[1] = temp[1]*this->size[1]/this->ncells[1] 
	                  + this->origin[1];
	        printf("Reportin at %f, %f\n", temp[0], temp[1]);
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
   return the direction we'd want to go, going around pivot.*/
bool SimpleOccupancyGrid::get_next_dir(double * curr_pose, double * pivot,
                                           double * return_dir){
    /* Figure out the idfference between pose and parent point */
    double diff[2];
    int c[2];
    printf("Start: %f, %f -> %f, %f ::: ", diff[0], diff[1], this->cgrid_owner[c[0]][2*c[1]], this->cgrid_owner[c[0]][2*c[1]+1]);
    if (!this->world2grid(curr_pose, c)) return false;
    diff[0] = curr_pose[0] - (this->cgrid_owner[c[0]][2*c[1]]
                             *this->size[0]/this->ncells[0] + this->origin[0]);
    diff[1] = curr_pose[1] - (this->cgrid_owner[c[0]][2*c[1]+1]
                             *this->size[1]/this->ncells[1] + this->origin[1]);  
    printf("To closest pt: %f, %f\n", diff[0], diff[1]);  
    /* Dir in axis we car about is perp to that */
    double tmp = diff[0];
    diff[0] = diff[1];
    diff[1] = -tmp;
    printf("Norm: %f, %f\n", diff[0], diff[1]);
    /* Invert if it's not going around pivot */
    double diff_pivot[2]; diff_pivot[0] = pivot[0] - curr_pose[0];
                          diff_pivot[1] = pivot[1] - curr_pose[1];
    tmp = diff_pivot[0];
    diff_pivot[0] = diff_pivot[1];
    diff_pivot[1] = -tmp;
    /* now diff_pivot is normal to robot-pivot vector */
    if (diff[0]*diff_pivot[0] + diff[1]*diff_pivot[1] < 0){
        /* invert if our direction isn't going that way */
        diff[0] *= -1.0;
        diff[1] *= -1.0;
    }
    printf("Inverted Norm: %f, %f\n", diff[0], diff[1]);
    /* And normalize */
    double tot = sqrtf( pow(diff[0], 2) + pow(diff[1], 2) );
    diff[0] /= tot;
    diff[1] /= tot;
    return_dir[0] = diff[0];
    return_dir[1] = diff[1];
    return true;
}
