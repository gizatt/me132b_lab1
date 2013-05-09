/*
 * Occupancy Grid
 *
 * Supplied by ME132b TA's
 *
 * Rev history:
 *  Gregory Izatt    20130509    Import from ME132b site; adding a "print
 *                                occ grid to console" function on a whim
 *
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
	
	for(int i=0;i<this->ncells[0];i++)
		this->grid.push_back(vector<double>(this->ncells[1]));

	for(int i=0;i<this->ncells[0];i++)
	for(int j=0;j<this->ncells[1];j++)
		this->grid[i][j] = 0;
	
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
		this->grid[c[0]][c[1]] = 1;
}
	
void SimpleOccupancyGrid::addScan(
	const double pose[3], 
	int n,  
	const double *bearing,
	const double *reading,
	double max_range) 
{
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
void SimpleOccupancyGrid::printPPM(int x, int y) const {
    int x_step = this->ncells[0] / x;
    int y_step = this->ncells[1] / y;
    int i_x, i_y, i_tx, i_ty;
    for (i_y = 0; i_y < this->ncells[1]; i_y += y_step){
        for (i_x = 0; i_x < this->ncells[0]; i_x += x_step){
            bool occ = false;
            for (i_ty = i_y; i_ty < i_y+y_step && i_ty < this->ncells[1]; 
                 i_ty++){
                for (i_tx = i_x; i_tx < i_x+x_step && i_tx < this->ncells[0]; 
                    i_tx++){
                    if (this->grid[i_tx][i_ty] > 0.0) {
                        occ = true;
                        break;  
                    }
                }
                if (occ) break;
            }
            if (occ)
                printf("O");
            else
                printf(" ");
        }
        printf("\n");
    }
}

