#ifndef PATH_PLANNING_JUMPPOINTSEARCH_H
#define PATH_PLANNING_JUMPPOINTSEARCH_H

#include <vector>

namespace path_planning {

class JumpPointSearch {
public:
int*** ogm;
int ogm_size;
int max_iterations;
struct Node
{
	int current_node_x;
	int current_node_y;
	int current_node_z;
	int parent_node_x;
	int parent_node_y;
	int parent_node_z;
	double dist_start;
	double dist_dest;
	double dist_heur;
	bool is_expanded = false;
};
std::vector <Node> node_array;
std::vector <Node> rpath;
JumpPointSearch();
~JumpPointSearch();
int check_search(int x, int y, int z);
int minimum();
void padding(int pad_size,int x, int y, int z);
void clear_ogm();
void set_mapsize(int map_size);
void set_max_iter(int max_iter);
int search_x(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, double distance, int node_array_index);
int search_y(int x, int y, int z, int xdest, int ydest, int zdest, int y_dist, double distance, int node_array_index);
int search_z(int x, int y, int z, int xdest, int ydest, int zdest, int z_dist, double distance, int node_array_index);
int search_diag_two_dim(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, int y_dist, int z_dist, double distance, int node_array_index);
int search_diag_three_dim(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, int y_dist, int z_dist, double distance, int node_array_index);
void Jump_Point_Search(int start[], int target[]);
};

}
#endif
