#ifndef H_OCCUPANCY_GRID
#define H_OCCUPANCY_GRID

#include <vector>

using namespace std;

class SimpleOccupancyGrid {
	
	public:
		
		/** Builds an occupancy grid given lower left and 
			upper right coordinates, plus the cell size. */
		SimpleOccupancyGrid(const double lower_left[2], 
							const double upper_right[2], double cell_size);

		/** Returns true if the specified point is occupied. 
			If a point is outside the grid, it is considered occupied. */
		bool occupied(const double world[2]) const;
	
		void markObstacle(const double p[2]);

		/** Ignores readings set to 0, nan, or larger than max_range */
		void addScan(const double pose[3], int n, 
			const double *bearing,
			const double *reading,
			double max_range
		);
		
		void savePPM(const char*filename) const;
		
		/* Prints out to console */
		void printPPM(int x, int y, const double pose[3]) const;
		
	private:

		/** Converts world coordinates to grid coordinates.
			Returns false if the point is outside the grid.    */
		bool world2grid(const double world[2], int grid[2]) const;
	
		int ncells[2];
		int origin[2];
		int size[2];
	
		vector< vector<double> > grid;
};

#endif
