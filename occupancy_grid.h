#ifndef H_OCCUPANCY_GRID
#define H_OCCUPANCY_GRID

#include <vector>

using namespace std;

#define CGRID_INIT (-1.0)
#define CGRID_DANGER (-2.0)


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
		void printPPM(int x, int y, const double pose[3], 
		              bool print_cgrid) const;
		
		void updateCGrid(double danger_thresh, double traverse_max);
        double cgrid_state(const double world[2]) const;        
        bool get_closest_traversable(double curr_pose[2], double temp[2]);
        bool get_next_dir(double curr_pose[2], double pivot[2],
                                           double return_dir[2]);                  
	private:

		/** Converts world coordinates to grid coordinates.
			Returns false if the point is outside the grid.    */
		bool world2grid(const double world[2], int grid[2]) const;
	
		int ncells[2];
		int origin[2];
		int size[2];
	
		vector< vector<double> > grid;
		
		/* And a container for c_occ map */
		vector< vector<double> > cgrid;
		/* And the owner of each point */
		vector< vector<double> > cgrid_owner;
};

#endif
