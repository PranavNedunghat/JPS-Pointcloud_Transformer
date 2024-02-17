#include<iostream>
#include<stdio.h>
#include<cstdlib>
#include<cmath>
#include<vector>
#include "float.h"
#include "JPS/path_planning/JumpPointSearch.h"

using namespace std;

namespace path_planning {

JumpPointSearch::JumpPointSearch()
{
 ogm = nullptr;
}

JumpPointSearch::~JumpPointSearch()
{
if(ogm != nullptr)
{
for (int i = 0; i< ogm_size; i++)
 {
  for (int j = 0; j< ogm_size; j++)
  {
   delete[] ogm[i][j];
  }
  delete[] ogm[i];
 } 
 delete[] ogm;
}
}

void JumpPointSearch::clear_ogm()
{
	for(int i = 0; i < ogm_size; i++)
	for(int j = 0; j < ogm_size; j++)
	for(int k = 0; k < ogm_size; k++)
	ogm[i][j][k] = 1;
}

void JumpPointSearch::set_mapsize(int map_size)
{
 ogm_size = map_size;
 ogm = new int**[ogm_size];
 for (int i = 0; i < ogm_size; i++)
 {ogm[i] = new int*[ogm_size];
  for (int j = 0; j < ogm_size; j++)
  {ogm[i][j] = new int[ogm_size];
   for (int k = 0; k < ogm_size; k++)
   {
    ogm[i][j][k] = 1;
   }
  }
 }
}

void JumpPointSearch::set_max_iter(int max_iter)
{
	max_iterations = max_iter;
}

void JumpPointSearch::padding(int pad_size,int x, int y, int z)
{
	ogm[x][y][z] = 0;
	if (pad_size <= 0)
	{
		return;
	}
	for (int i = -pad_size; i<=pad_size; i++)
	for (int j = -pad_size; j<=pad_size; j++)
	for (int k = -pad_size; k<=pad_size; k++)
	{
		if (((x+i >=0 ) && (x+i < ogm_size)) && ((y+j >=0 ) && (y+j < ogm_size)) && ((z+k >=0 ) && (z+k < ogm_size)))
		{
			ogm[x+i][y+j][z+k] = 0;
		}
	}
	
	
return;
}

int JumpPointSearch::check_search(int x, int y, int z)
	{	
	
		int index = -1;
		for (int i = 0; i < node_array.size(); i++)
		{
			if ((node_array[i].current_node_x == x) && (node_array[i].current_node_y == y) && (node_array[i].current_node_z == z))
			{
				index = i;
			}
		}
		return index;
	}

	int JumpPointSearch::minimum()
	{
		double min = DBL_MAX;
		int index = -1;
		for (int i = 0; i < node_array.size(); i++)
		{
			if ((node_array[i].dist_heur < min) && (node_array[i].is_expanded == false))
			{
				index = i;
				min = node_array[i].dist_heur;
			}
		}
		return index;
	}

	int JumpPointSearch::search_x(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, double distance, int node_array_index)
	{
		int x_node = x;
		int y_node = y;
		int z_node = z;
		int x_dest = xdest;
		int y_dest = ydest;
		int z_dest = zdest;
		double dist = distance;
		int map_x = ogm_size;
		int map_y = ogm_size;
		int map_z = ogm_size;
		int X_dist = 0;
		int x1 = x_node;
		int i = node_array_index;
		int flag = 0;
		int x2;
		int f;
		Node temp_node{};
		if ((y_node < 0 || y_node > map_y) ||(z_node < 0 || z_node > map_z))
		{
		return i;
		}
		while (true)
		{
			x1 += x_dist;
			X_dist += abs(x_dist);
			if (x1 >= map_x || x1 < 0)
			{
				break;
			}
			if (ogm[x1][y_node][z_node] == 0)
				break;
			if (x1 == x_dest && y_node == y_dest && z_node == z_dest)
			{
				temp_node.current_node_x = x1;
				temp_node.current_node_y = y_node;
				temp_node.current_node_z = z_node;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = X_dist + dist;
				temp_node.dist_dest = 0;
				temp_node.dist_heur = X_dist + dist;
				node_array.push_back(temp_node);
				i += 1;
				break;
			}
			x2 = x1 + x_dist;
			if ((y_node - 1 >= 0) && (x2 >= 0) && (x2 < map_x))
			{
				if (ogm[x1][y_node - 1][z_node] == 0 && ogm[x2][y_node - 1][z_node] != 0)
				{
					f = check_search(x2, y_node - 1, z_node);
					if (f == -1)
					{
						temp_node.current_node_x = x2;
						temp_node.current_node_y = y_node - 1;
						temp_node.current_node_z = z_node;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + X_dist + 1.414;
						temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						temp_node.dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x1;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + X_dist + 1.414;
							node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							node_array[f].dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((y_node + 1 < map_y) && (x2 >= 0) && (x2 < map_x))
			{
				if (ogm[x1][y_node + 1][z_node] == 0 && ogm[x2][y_node + 1][z_node] != 0)
				{
					f = check_search(x2, y_node + 1, z_node);
					if (f == -1)
					{
						temp_node.current_node_x = x2;
						temp_node.current_node_y = y_node + 1;
						temp_node.current_node_z = z_node;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + X_dist + 1.414;
						temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						temp_node.dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x1;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + X_dist + 1.414;
							node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							node_array[f].dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((z_node - 1 >= 0) && (x2 >= 0) && (x2 < map_x))
			{
				if (ogm[x1][y_node][z_node - 1] == 0 && ogm[x2][y_node][z_node - 1] != 0)
				{
					f = check_search(x2, y_node, z_node - 1);
					if (f == -1)
					{
						temp_node.current_node_x = x2;
						temp_node.current_node_y = y_node;
						temp_node.current_node_z = z_node - 1;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + X_dist + 1.414;
						temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						temp_node.dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x1;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + X_dist + 1.414;
							node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							node_array[f].dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((z_node + 1 < map_z) && (x2 >= 0) && (x2 < map_x))
			{
				if (ogm[x1][y_node][z_node + 1] == 0 && ogm[x2][y_node][z_node + 1] != 0)
				{
					f = check_search(x2, y_node, z_node + 1);
					if (f == -1)
					{
						temp_node.current_node_x = x2;
						temp_node.current_node_y = y_node;
						temp_node.current_node_z = z_node + 1;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + X_dist + 1.414;
						temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						temp_node.dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if( X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x1;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + X_dist + 1.414;
							node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							node_array[f].dist_heur = X_dist + dist + 1.414 + double(sqrt((x2 - x_dest) * (x2 - x_dest) + ((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							flag = 1;
						}
					}
				}
			}
			if (flag == 1)
			{
				f = check_search(x1, y_node, z_node);
				if (f == -1)
				{
					temp_node.current_node_x = x1;
					temp_node.current_node_y = y_node;
					temp_node.current_node_z = z_node;
					temp_node.parent_node_x = x_node;
					temp_node.parent_node_y = y_node;
					temp_node.parent_node_z = z_node;
					temp_node.dist_start = dist + X_dist;
					temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + ((y_node)-y_dest) * ((y_node)-y_dest) + (z_node - z_dest) * (z_node - z_dest)));
					temp_node.dist_heur = X_dist + dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + ((y_node)-y_dest) * ((y_node)-y_dest) + (z_node - z_dest) * (z_node - z_dest)));
					node_array.push_back(temp_node);
					i += 1;
					flag = 0;
				}
				else if(f != -1)
					{
						if( X_dist + dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + ((y_node)-y_dest) * ((y_node)-y_dest) + (z_node - z_dest) * (z_node - z_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + X_dist;
							node_array[f].dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + ((y_node)-y_dest) * ((y_node)-y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							node_array[f].dist_heur = X_dist + dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + ((y_node)-y_dest) * ((y_node)-y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							flag = 0;
						}
					}
			}

		}
		return i;
	}

	int JumpPointSearch::search_y(int x, int y, int z, int xdest, int ydest, int zdest, int y_dist, double distance, int node_array_index)
	{
		int x_node = x;
		int y_node = y;
		int z_node = z;
		int x_dest = xdest;
		int y_dest = ydest;
		int z_dest = zdest;
		double dist = distance;
		int map_x = ogm_size;
		int map_y = ogm_size;
		int map_z = ogm_size;
		int Y_dist = 0;
		int y1 = y_node;
		int i = node_array_index;
		int flag = 0;
		int g;
		int y2;
		int f;
		Node temp_node{};
		if ((x_node < 0 || x_node > map_x) ||(z_node < 0 || z_node > map_z))
		{
		return i;
		}
		while (true)
		{
			y1 += y_dist;
			Y_dist += abs(y_dist);
			if (y1 >= map_y || y1 < 0)
			{
				break;
			}
			g = ogm[x_node][y1][z_node];
			if (g == 0)
				break;
			if (x_node == x_dest && y1 == y_dest && z_node == z_dest)
			{
				temp_node.current_node_x = x_node;
				temp_node.current_node_y = y1;
				temp_node.current_node_z = z_node;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = Y_dist + dist;
				temp_node.dist_dest = 0;
				temp_node.dist_heur = Y_dist + dist;
				node_array.push_back(temp_node);
				i += 1;
				break;
			}
			y2 = y1 + y_dist;
			if ((x_node - 1 >= 0) && (y2 >= 0) && (y2 < map_y))
			{
				if (ogm[x_node - 1][y1][z_node] == 0 && ogm[x_node - 1][y2][z_node] != 0)
				{
					f = check_search(x_node - 1, y2, z_node);
					if (f == -1)
					{
						temp_node.current_node_x = x_node - 1;
						temp_node.current_node_y = y2;
						temp_node.current_node_z = z_node;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + Y_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						temp_node.dist_heur = Y_dist + dist + 1.414 + double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if( Y_dist + dist + 1.414 + double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y1;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + Y_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							node_array[f].dist_heur = Y_dist + dist + 1.414 + double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((x_node + 1 < map_x) && (y2 >= 0) && (y2 < map_y))
			{
				if (ogm[x_node + 1][y1][z_node] == 0 && ogm[x_node + 1][y2][z_node] != 0)
				{
					f = check_search(x_node + 1, y2, z_node);
					if (f == -1)
					{
						temp_node.current_node_x = x_node + 1;
						temp_node.current_node_y = y2;
						temp_node.current_node_z = z_node;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + Y_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						temp_node.dist_heur = Y_dist + dist + 1.414 + double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Y_dist + dist + 1.414 + double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y1;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + Y_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							node_array[f].dist_heur = Y_dist + dist + 1.414 + double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((z_node - 1 >= 0) && (y2 >= 0) && (y2 < map_y))
			{
				if (ogm[x_node][y1][z_node - 1] == 0 && ogm[x_node][y2][z_node - 1] != 0)
				{
					f = check_search(x_node, y2, z_node - 1);
					if (f == -1)
					{
						temp_node.current_node_x = x_node;
						temp_node.current_node_y = y2;
						temp_node.current_node_z = z_node - 1;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + Y_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
						temp_node.dist_heur = Y_dist + dist + 1.414 + double(sqrt(((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Y_dist + dist + 1.414 + double(sqrt(((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y1;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + Y_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
							node_array[f].dist_heur = Y_dist + dist + 1.414 + double(sqrt(((z_node - 1) - z_dest) * ((z_node - 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((z_node + 1 < map_z) && (y2 >= 0) && (y2 < map_y))
			{
				if (ogm[x_node][y1][z_node + 1] == 0 && ogm[x_node][y2][z_node + 1] != 0)
				{
					f = check_search(x_node, y2, z_node + 1);
					if (f == -1)
					{
						temp_node.current_node_x = x_node;
						temp_node.current_node_y = y2;
						temp_node.current_node_z = z_node + 1;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z_node;
						temp_node.dist_start = dist + Y_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
						temp_node.dist_heur = Y_dist + dist + 1.414 + double(sqrt(((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Y_dist + dist + 1.414 + double(sqrt(((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y1;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + Y_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
							node_array[f].dist_heur = Y_dist + dist + 1.414 + double(sqrt(((z_node + 1) - z_dest) * ((z_node + 1) - z_dest) + (y2 - y_dest) * (y2 - y_dest) + (x_node - x_dest) * (x_node - x_dest)));
							flag = 1;
						}
					}
				}
			}
			if (flag == 1)
			{
				f = check_search(x_node, y1, z_node);
				if (f == -1)
				{
					temp_node.current_node_x = x_node;
					temp_node.current_node_y = y1;
					temp_node.current_node_z = z_node;
					temp_node.parent_node_x = x_node;
					temp_node.parent_node_y = y_node;
					temp_node.parent_node_z = z_node;
					temp_node.dist_start = dist + Y_dist;
					temp_node.dist_dest = double(sqrt((x_node - x_dest) * (x_node - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
					temp_node.dist_heur = Y_dist + dist + double(sqrt((x_node - x_dest) * (x_node - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
					node_array.push_back(temp_node);
					i += 1;
					flag = 0;
				}
				else if(f != -1)
					{
						if(Y_dist + dist + double(sqrt((x_node - x_dest) * (x_node - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z_node - z_dest) * (z_node - z_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + Y_dist;
							node_array[f].dist_dest = double(sqrt((x_node - x_dest) * (x_node - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							node_array[f].dist_heur = Y_dist + dist + double(sqrt((x_node - x_dest) * (x_node - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z_node - z_dest) * (z_node - z_dest)));
							flag = 0;
						}
					}
			}

		}
		return i;
	}

	int JumpPointSearch::search_z(int x, int y, int z, int xdest, int ydest, int zdest, int z_dist, double distance, int node_array_index)
	{	
		int x_node = x;
		int y_node = y;
		int z_node = z;
		int x_dest = xdest;
		int y_dest = ydest;
		int z_dest = zdest;
		double dist = distance;
		int map_x = ogm_size;
		int map_y = ogm_size;
		int map_z = ogm_size;
		int Z_dist = 0;
		int z1 = z_node;
		int i = node_array_index;
		int flag = 0;
		int g;
		int z2;
		int f;
		Node temp_node{};
		if ((y_node < 0 || y_node > map_y) ||(x_node < 0 || x_node > map_x))
		{
		return i;
		}
		while (true)
		{
			z1 += z_dist;
			Z_dist += abs(z_dist);
			if (z1 >= map_z || z1 < 0)
			{
				break;
			}
			g = ogm[x_node][y_node][z1];
			if (g == 0)
				{break;}
			if (x_node == x_dest && y_node == y_dest && z1 == z_dest)
			{
				temp_node.current_node_x = x_node;
				temp_node.current_node_y = y_node;
				temp_node.current_node_z = z1;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = Z_dist + dist;
				temp_node.dist_dest = 0;
				temp_node.dist_heur = Z_dist + dist;
				node_array.push_back(temp_node);
				i += 1;
				break;
			}
			z2 = z1 + z_dist;
			if ((x_node - 1 >= 0) && (z2 >= 0) && (z2 < map_z))
			{
				if (ogm[x_node - 1][y_node][z1] == 0 && ogm[x_node - 1][y_node][z2] != 0)
				{	
					f = check_search(x_node - 1, y_node, z2);
					if (f == -1)
					{
						temp_node.current_node_x = x_node - 1;
						temp_node.current_node_y = y_node;
						temp_node.current_node_z = z2;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist + Z_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						temp_node.dist_heur = Z_dist + dist + 1.414 + double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Z_dist + dist + 1.414 + double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z1;
							node_array[f].dist_start = dist + Z_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							node_array[f].dist_heur = Z_dist + dist + 1.414 + double(sqrt(((x_node - 1) - x_dest) * ((x_node - 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((x_node + 1 < map_x) && (z2 >= 0) && (z2 < map_z))
			{
				if (ogm[x_node + 1][y_node][z1] == 0 && ogm[x_node + 1][y_node][z2] != 0)
				{
					f = check_search(x_node + 1, y_node, z2);
					if (f == -1)
					{
						temp_node.current_node_x = x_node + 1;
						temp_node.current_node_y = y_node;
						temp_node.current_node_z = z2;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist + Z_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						temp_node.dist_heur = Z_dist + dist + 1.414 + double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Z_dist + dist + 1.414 + double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z1;
							node_array[f].dist_start = dist + Z_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							node_array[f].dist_heur = Z_dist + dist + 1.414 + double(sqrt(((x_node + 1) - x_dest) * ((x_node + 1) - x_dest) + (z2 - z_dest) * (z2 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((y_node - 1 >= 0) && (z2 >= 0) && (z2 < map_z))
			{
				if (ogm[x_node][y_node - 1][z1] == 0 && ogm[x_node][y_node - 1][z2] != 0)
				{
					f = check_search(x_node, y_node - 1, z2);
					if (f == -1)
					{
						temp_node.current_node_x = x_node;
						temp_node.current_node_y = y_node - 1;
						temp_node.current_node_z = z2;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist + Z_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
						temp_node.dist_heur = Z_dist + dist + 1.414 + double(sqrt(((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Z_dist + dist + 1.414 + double(sqrt(((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z1;
							node_array[f].dist_start = dist + Z_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
							node_array[f].dist_heur = Z_dist + dist + 1.414 + double(sqrt(((y_node - 1) - y_dest) * ((y_node - 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
							flag = 1;
						}
					}
				}
			}
			if ((y_node + 1 < map_y) && (z2 >= 0) && (z2 < map_z))
			{
				if (ogm[x_node][y_node + 1][z1] == 0 && ogm[x_node][y_node + 1][z2] != 0)
				{
					f = check_search(x_node, y_node + 1, z2);
					if (f == -1)
					{
						temp_node.current_node_x = x_node;
						temp_node.current_node_y = y_node + 1;
						temp_node.current_node_z = z2;
						temp_node.parent_node_x = x_node;
						temp_node.parent_node_y = y_node;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist + Z_dist + 1.414;
						temp_node.dist_dest = double(sqrt(((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
						temp_node.dist_heur = Z_dist + dist + 1.414 + double(sqrt(((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
					else if(f != -1)
					{
						if(Z_dist + dist + 1.414 + double(sqrt(((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z1;
							node_array[f].dist_start = dist + Z_dist + 1.414;
							node_array[f].dist_dest = double(sqrt(((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
							node_array[f].dist_heur = Z_dist + dist + 1.414 + double(sqrt(((y_node + 1) - y_dest) * ((y_node + 1) - y_dest) + (z2 - z_dest) * (z2 - z_dest) + (x_node - x_dest) * (x_node - x_dest)));
							flag = 1;
						}
					}
				}
			}
			if (flag == 1)
			{
				f = check_search(x_node, y_node, z1);
				if (f == -1)
				{
					temp_node.current_node_x = x_node;
					temp_node.current_node_y = y_node;
					temp_node.current_node_z = z1;
					temp_node.parent_node_x = x_node;
					temp_node.parent_node_y = y_node;
					temp_node.parent_node_z = z_node;
					temp_node.dist_start = dist + Z_dist;
					temp_node.dist_dest = double(sqrt((x_node - x_dest) * (x_node - x_dest) + (z1 - z_dest) * (z1 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
					temp_node.dist_heur = Z_dist + dist + double(sqrt((x_node - x_dest) * (x_node - x_dest) + (z1 - z_dest) * (z1 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
					node_array.push_back(temp_node);
					i += 1;
					flag = 0;
				}
				else if(f != -1)
					{
						if(Z_dist + dist + double(sqrt((x_node - x_dest) * (x_node - x_dest) + (z1 - z_dest) * (z1 - z_dest) + (y_node - y_dest) * (y_node - y_dest))) < node_array[f].dist_heur)
						{
							node_array[f].parent_node_x = x_node;
							node_array[f].parent_node_y = y_node;
							node_array[f].parent_node_z = z_node;
							node_array[f].dist_start = dist + Z_dist;
							node_array[f].dist_dest = double(sqrt((x_node - x_dest) * (x_node - x_dest) + (z1 - z_dest) * (z1 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							node_array[f].dist_heur = Z_dist + dist + double(sqrt((x_node - x_dest) * (x_node - x_dest) + (z1 - z_dest) * (z1 - z_dest) + (y_node - y_dest) * (y_node - y_dest)));
							flag = 0;
						}
					}
			}

		}
		return i;
	}

	int JumpPointSearch::search_diag_two_dim(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, int y_dist, int z_dist, double distance, int node_array_index)
	{
		int x_node = x;
		int y_node = y;
		int z_node = z;
		int x_dest = xdest;
		int y_dest = ydest;
		int z_dest = zdest;
		int x1 = x_node;
		int y1 = y_node;
		int z1 = z_node;
		const double dist = distance;
		int map_x = ogm_size;
		int map_y = ogm_size;
		int map_z = ogm_size;
		int i, in;
		i = in = node_array.size();
		double diag_dist = 0;
		double diag_dist_one_step;
		int f, x2, y2, z2, flag, flag1;
		flag = flag1 = 0;
		Node temp_node{};

		while (true)
		{
			x1 += x_dist;
			y1 += y_dist;
			z1 += z_dist;
			diag_dist = diag_dist + 1.414;
			if ((x1<0 || x1>=map_x) || (y1<0 || y1>=map_y) || (z1<0 || z1>=map_z))
				break;
			if (ogm[x1][y1][z1] == 0)
				break;
			if ((x1 == x_dest) && (y1 == y_dest) && (z1 == z_dest))
			{
				temp_node.current_node_x = x1;
				temp_node.current_node_y = y1;
				temp_node.current_node_z = z1;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = distance + diag_dist;
				temp_node.dist_dest = 0;
				temp_node.dist_heur = diag_dist + distance;
				node_array.push_back(temp_node);
				i += 1;
				break;
			}
			diag_dist_one_step = diag_dist + 1.414;
			x2 = x1 + x_dist;
			y2 = y1 + y_dist;
			z2 = z1 + z_dist;
			flag = 0;
			flag1 = 0;
			if (z_dist == 0)
			{
				if ((y2 >= 0 && y2 < map_y) && ((x1 - x_dist >=0) && (x1 - x_dist < map_x)))
				{
					if ((ogm[x1 - x_dist][y1][z1] == 0) && (ogm[x1 - x_dist][y2][z1] != 0))
					{
						f = check_search(x1 - x_dist, y2, z1);
						if (f == -1)
						{
							temp_node.current_node_x = x1 - x_dist;
							temp_node.current_node_y = y2;
							temp_node.current_node_z = z1;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step;
							temp_node.dist_dest = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
							temp_node.dist_heur = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist + diag_dist_one_step;
							i += 1;
							node_array.push_back(temp_node);
							flag = 1;
						}
						else if(f != -1)
						{
						if(double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist + diag_dist_one_step < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step;
								node_array[f].dist_dest = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
								node_array[f].dist_heur = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist + diag_dist_one_step;
								flag = 1;
							}
						}
					}
				}
				if ((x2 >= 0 && x2 < map_x) && ((y1 - y_dist >=0) && (y1 - y_dist < map_y)))
				{
					if ((ogm[x1][y1 - y_dist][z1] == 0) && ogm[x2][y1 - y_dist][z1] != 0)
					{
						f = check_search(x2, y1 - y_dist, z1);
						if (f == -1)
						{
							temp_node.current_node_x = x2;
							temp_node.current_node_y = y1 - y_dist;
							temp_node.current_node_z = z1;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step;
							temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
							temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist + diag_dist_one_step;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
						}
						else if(f != -1)
						{
						if(double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist + diag_dist_one_step < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step;
								node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist + diag_dist_one_step;
								flag = 1;
							}
						}
					}
				}
				if ((x2>=0 && x2 < map_x) && (y2>=0 && y2<map_y))
				{
					if (z1 + 1 < map_z)
					{
					 if (ogm[x1][y1][z1+1] == 0 && ogm[x2][y2][z1+1] != 0)
					 {
					  f = check_search(x2,y2,z1+1);
					  if (f == -1)
					  {
					   		temp_node.current_node_x = x2;
							temp_node.current_node_y = y2;
							temp_node.current_node_z = z1 + 1;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step + 0.318;
							temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 + 1 - z_dest) * (z1 + 1 - z_dest)));
							temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 + 1 - z_dest) * (z1 + 1 - z_dest))) + dist + diag_dist_one_step + 0.318;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
					  }
					  else if(f != -1)
						{
						if(double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 + 1 - z_dest) * (z1 + 1 - z_dest))) + dist + diag_dist_one_step + 0.318 < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step + 0.318;
								node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 + 1 - z_dest) * (z1 + 1 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 + 1 - z_dest) * (z1 + 1 - z_dest))) + dist + diag_dist_one_step + 0.318;
								flag = 1;
							}
						}
					 }
					}
					if (z1 - 1 >= 0)
					{
					 if (ogm[x1][y1][z1-1] == 0 && ogm[x2][y2][z1-1] != 0)
					 {
					   f = check_search(x2,y2,z1-1);
					  if (f == -1)
					  {
					   		temp_node.current_node_x = x2;
							temp_node.current_node_y = y2;
							temp_node.current_node_z = z1 - 1;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step + 0.318;
							temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - 1 - z_dest) * (z1 - 1 - z_dest)));
							temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - 1 - z_dest) * (z1 - 1 - z_dest))) + dist + diag_dist_one_step + 0.318;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
					  }
					  else if(f != -1)
						{
						if(double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - 1 - z_dest) * (z1 - 1 - z_dest))) + dist + diag_dist_one_step + 0.318 < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step + 0.318;
								node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - 1 - z_dest) * (z1 - 1 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - 1 - z_dest) * (z1 - 1 - z_dest))) + dist + diag_dist_one_step + 0.318;
								flag = 1;
							}
						}
					 }
					}
				}
			}
			else if (y_dist == 0)
			{
				if ((z2 >= 0 && z2 < map_z) && ((x1 - x_dist >=0) && (x1 - x_dist < map_x)))
				{
					if ((ogm[x1 - x_dist][y1][z1] == 0) && (ogm[x1 - x_dist][y1][z2] != 0))
					{
						f = check_search(x1 - x_dist, y1, z2);
						if (f == -1)
						{
							temp_node.current_node_x = x1 - x_dist;
							temp_node.current_node_y = y1;
							temp_node.current_node_z = z2;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step;
							temp_node.dist_dest = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
							temp_node.dist_heur = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step;
							i += 1;
							node_array.push_back(temp_node);
							flag = 1;
						}
						else if(f != -1)
						{
						if( double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step;
								node_array[f].dist_dest = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
								node_array[f].dist_heur = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step;
								flag = 1;
							}
						}
					}
				}
				if ((x2 >= 0 && x2 < map_x) && ((z1 - z_dist >=0) && (z1 - z_dist < map_z)))
				{
					if ((ogm[x1][y1][z1 - z_dist] == 0) && ogm[x2][y1][z1 - z_dist] != 0)
					{
						f = check_search(x2, y1, z1 - z_dist);
						if (f == -1)
						{
							temp_node.current_node_x = x2;
							temp_node.current_node_y = y1;
							temp_node.current_node_z = z1 - z_dist;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step;
							temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest)));
							temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist + diag_dist_one_step;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
						}
						else if(f != -1)
						{
						if(double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist + diag_dist_one_step < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step;
								node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest)));
								node_array[f].dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist + diag_dist_one_step;
								flag = 1;
							}
						}
					}
				}
				if ((x2>=0 && x2 < map_x) && (z2>=0 && z2<map_z))
				{
					if (y1 + 1 < map_y)
					{
					 if (ogm[x1][y1+1][z1] == 0 && ogm[x2][y1+1][z2] != 0)
					 {
					  f = check_search(x2,y1+1,z2);
					  if (f == -1)
					  {
					   		temp_node.current_node_x = x2;
							temp_node.current_node_y = y1 + 1;
							temp_node.current_node_z = z2;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step + 0.318;
							temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 + 1 - y_dest) * (y1 + 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
							temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 + 1 - y_dest) * (y1 + 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
					  }
					  else if(f != -1)
						{
						if(double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 + 1 - y_dest) * (y1 + 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318 < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step + 0.318;
								node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 + 1 - y_dest) * (y1 + 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 + 1 - y_dest) * (y1 + 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
								flag = 1;
							}
						}
					 }
					}
					if (y1 - 1 >= 0)
					{
					 if (ogm[x1][y1-1][z1] == 0 && ogm[x2][y1-1][z2] == 1)
					 {
					   f = check_search(x2,y1-1,z2);
					  if (f == -1)
					  {
					   		temp_node.current_node_x = x2;
							temp_node.current_node_y = y1 - 1;
							temp_node.current_node_z = z2;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step + 0.318;
							temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - 1 - y_dest) * (y1 - 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
							temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - 1 - y_dest) * (y1 - 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
					  }
					  else if(f != -1)
						{
						if( double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - 1 - y_dest) * (y1 - 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318 < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step + 0.318;
								node_array[f].dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - 1 - y_dest) * (y1 - 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - 1 - y_dest) * (y1 - 1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
								flag = 1;
							}
						}
					 }
					}
				}
				
			}
			else if (x_dist == 0)
			{
				if ((z2 >= 0 && z2 < map_z) && ((y1 - y_dist >=0) && (y1 - y_dist < map_y)))
				{
					if ((ogm[x1][y1 - y_dist][z1] == 0) && (ogm[x1][y1 - y_dist][z2] != 0))
					{
						f = check_search(x1, y1 - y_dist, z2);
						if (f == -1)
						{
							temp_node.current_node_x = x1;
							temp_node.current_node_y = y1 - y_dist;
							temp_node.current_node_z = z2;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step;
							temp_node.dist_dest = double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest)));
							temp_node.dist_heur = double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step;
							i += 1;
							node_array.push_back(temp_node);
							flag = 1;
						}
						else if(f != -1)
						{
						if(double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step;
								node_array[f].dist_dest = double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest)));
								node_array[f].dist_heur = double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step;
								flag = 1;
							}
						}
					}
				}
				if ((y2 >= 0 && y2 < map_y) && ((z1 - z_dist >=0) && (z1 - z_dist < map_z)))
				{
					if ((ogm[x1][y1][z1 - z_dist] == 0) && ogm[x1][y2][z1 - z_dist] != 0)
					{
						f = check_search(x1, y2, z1 - z_dist);
						if (f == -1)
						{
							temp_node.current_node_x = x1;
							temp_node.current_node_y = y2;
							temp_node.current_node_z = z1 - z_dist;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step;
							temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest)));
							temp_node.dist_heur = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist + diag_dist_one_step;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
						}
						else if(f != -1)
						{
						if( double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist + diag_dist_one_step < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step;
								node_array[f].dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest)));
								node_array[f].dist_heur = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist + diag_dist_one_step;
								flag = 1;
							}
						}
					}
				}
				if ((y2>=0 && y2<map_y)&&(z2>=0 && z2<map_z))
				{
				 
					if (x1 + 1 < map_x)
					{
					 if (ogm[x1+1][y1][z1] == 0 && ogm[x1+1][y2][z2] != 0)
					 {
					  f = check_search(x1+1,y2,z2);
					  if (f == -1)
					  {
					   		temp_node.current_node_x = x1 + 1;
							temp_node.current_node_y = y2;
							temp_node.current_node_z = z2;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step + 0.318;
							temp_node.dist_dest = double(sqrt((x1 + 1 - x_dest) * (x1 + 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
							temp_node.dist_heur = double(sqrt((x1 + 1 - x_dest) * (x1 + 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
					  }
					  else if(f != -1)
						{
						if(double(sqrt((x1 + 1 - x_dest) * (x1 + 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318 < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step + 0.318;
								node_array[f].dist_dest = double(sqrt((x1 + 1 - x_dest) * (x1 + 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x1 + 1 - x_dest) * (x1 + 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
								flag = 1;
							}
						}
					 }
					}
					if (x1 - 1 >= 0)
					{
					 if (ogm[x1-1][y1][z1] == 0 && ogm[x1-1][y2][z2] != 0)
					 {
					   f = check_search(x1-1,y2,z2);
					  if (f == -1)
					  {
					   		temp_node.current_node_x = x1 - 1;
							temp_node.current_node_y = y2;
							temp_node.current_node_z = z2;
							temp_node.parent_node_x = x1;
							temp_node.parent_node_y = y1;
							temp_node.parent_node_z = z1;
							temp_node.dist_start = dist + diag_dist_one_step + 0.318;
							temp_node.dist_dest = double(sqrt((x1 - 1 - x_dest) * (x1 - 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
							temp_node.dist_heur = double(sqrt((x1 - 1 - x_dest) * (x1 - 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
							node_array.push_back(temp_node);
							i += 1;
							flag = 1;
					  }
					  else if(f != -1)
						{
						if(double(sqrt((x1 - 1 - x_dest) * (x1 - 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318 < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x1;
								node_array[f].parent_node_y = y1;
								node_array[f].parent_node_z = z1;
								node_array[f].dist_start = dist + diag_dist_one_step + 0.318;
								node_array[f].dist_dest = double(sqrt((x1 - 1 - x_dest) * (x1 - 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
								node_array[f].dist_heur = double(sqrt((x1 - 1 - x_dest) * (x1 - 1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist + diag_dist_one_step + 0.318;
								flag = 1;
							}
						}
					 }
					}
				
				} 
			}
			if (flag == 1)
			{
				f = check_search(x1, y1, z1);
				if (f == -1)
				{
					temp_node.current_node_x = x1;
					temp_node.current_node_y = y1;
					temp_node.current_node_z = z1;
					temp_node.parent_node_x = x_node;
					temp_node.parent_node_y = y_node;
					temp_node.parent_node_z = z_node;
					temp_node.dist_start = dist + diag_dist;
					temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
					temp_node.dist_heur = dist + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
					node_array.push_back(temp_node);
					i += 1;
					flag = 0;
					flag1 = 1;
				}
				else if(f != -1)
						{
						if(dist + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x_node;
								node_array[f].parent_node_y = y_node;
								node_array[f].parent_node_z = z_node;
								node_array[f].dist_start = dist + diag_dist;
								node_array[f].dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
								node_array[f].dist_heur = dist + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
								flag = 0;
								flag1 = 1;
							}
						}
			}
			in = i;
			if (z_dist == 0)
			{
				i = search_x(x1, y1, z1, xdest, ydest, zdest, x_dist, dist + diag_dist, i);
				i = search_y(x1, y1, z1, xdest, ydest, zdest, y_dist, dist + diag_dist, i);
			}
			else if (y_dist == 0)
			{
				i = search_x(x1, y1, z1, xdest, ydest, zdest, x_dist, dist + diag_dist, i);
				i = search_z(x1, y1, z1, xdest, ydest, zdest, z_dist, dist + diag_dist, i);
			}
			else if (x_dist == 0)
			{
				i = search_y(x1, y1, z1, xdest, ydest, zdest, y_dist, dist + diag_dist, i);
				i = search_z(x1, y1, z1, xdest, ydest, zdest, z_dist, dist + diag_dist, i);
			}
			f = check_search(x1, y1, z1);
			if (flag1 == 0 && in != i && f == -1)
			{
				temp_node.current_node_x = x1;
				temp_node.current_node_y = y1;
				temp_node.current_node_z = z1;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = dist + diag_dist;
				temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
				temp_node.dist_heur = dist + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
				node_array.push_back(temp_node);
				i += 1;
			}
			else if(flag1 == 0 && in != i && f != -1)
						{
						if(dist + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x_node;
								node_array[f].parent_node_y = y_node;
								node_array[f].parent_node_z = z_node;
								node_array[f].dist_start = dist + diag_dist;
								node_array[f].dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
								node_array[f].dist_heur = dist + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
							}
						}
		}
		return i;
	}

	int JumpPointSearch::search_diag_three_dim(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, int y_dist, int z_dist, double distance, int node_array_index)
	{
		int x_node = x;
		int y_node = y;
		int z_node = z;
		int x_dest = xdest;
		int y_dest = ydest;
		int z_dest = zdest;
		int x1 = x_node;
		int y1 = y_node;
		int z1 = z_node;
		double dist = distance;
		int map_x = ogm_size;
		int map_y = ogm_size;
		int map_z = ogm_size;
		int i, in;
		i = in = node_array.size();
		double diag_dist = 0;
		int g, f, flag, flag1;
		flag = flag1 = 0;
		Node temp_node{};

		while (true)
		{	
			x1 += x_dist;
			y1 += y_dist;
			z1 += z_dist;
			diag_dist = diag_dist + 1.732;
			if ((x1<0 || x1>=map_x) || (y1<0 || y1>=map_y) || (z1<0 || z1>=map_z))
				break;
			g = ogm[x1][y1][z1];
			if (g == 0)
				break;
			if ((x1 == x_dest) && (y1 == y_dest) && (z1 == z_dest))
			{
				temp_node.current_node_x = x1;
				temp_node.current_node_y = y1;
				temp_node.current_node_z = z1;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = distance + diag_dist;
				temp_node.dist_dest = 0;
				temp_node.dist_heur = diag_dist + distance;
				node_array.push_back(temp_node);
				i += 1;
				break;
			}
			i = in = node_array.size();
      		i = search_x(x1,y1,z1,x_dest,y_dest,z_dest,x_dist,dist + diag_dist,i);
      		i = search_y(x1,y1,z1,x_dest,y_dest,z_dest,y_dist,dist + diag_dist,i);
      		i = search_z(x1,y1,z1,x_dest,y_dest,z_dest,z_dist,dist + diag_dist,i);
			i = search_diag_two_dim(x1, y1, z1, x_dest, y_dest, z_dest, x_dist, y_dist, 0, dist + diag_dist, i);
			i = search_diag_two_dim(x1, y1, z1, x_dest, y_dest, z_dest, x_dist, 0, z_dist, dist + diag_dist, i);
			i = search_diag_two_dim(x1, y1, z1, x_dest, y_dest, z_dest, 0, y_dist, z_dist, dist + diag_dist, i);
			f = check_search(x1, y1, z1);
			if (in != i && f == -1)
			{
				temp_node.current_node_x = x1;
				temp_node.current_node_y = y1;
				temp_node.current_node_z = z1;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = distance + diag_dist;
				temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
				temp_node.dist_heur = distance + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
				node_array.push_back(temp_node);
				i += 1;
			}
			else if(in != i && f != -1)
						{
						if(distance + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) < node_array[f].dist_heur)
							{
								node_array[f].parent_node_x = x_node;
								node_array[f].parent_node_y = y_node;
								node_array[f].parent_node_z = z_node;
								node_array[f].dist_start = dist + diag_dist;
								node_array[f].dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
								node_array[f].dist_heur = distance + diag_dist + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
							}
						}

		}

		return i;
	}

	void JumpPointSearch::Jump_Point_Search(int start[], int target[])
	{
		if(ogm == nullptr)
		{
			cout<<"Map size is unintialized. Please set the map size first\n";
			return;
		}
		node_array.clear();
		rpath.clear();
		int x_node, x_start, y_node, y_start, z_node, z_start;
		x_node = x_start = start[0];
		y_node = y_start = start[1];
		z_node = z_start = start[2];
		int x_dest = target[0];
		int y_dest = target[1];
		int z_dest = target[2];
		if (ogm[x_start][y_start][z_start] == 0)
		{ cout << "Start is in an occupied cell. Path not possible\n";
		  return;
		}
		if ( ogm[x_dest][y_dest][z_dest] == 0)
		{
		  cout << "Destination is in an occupied cell. Path not possible\n";
		  return;
		}
		int iterations = 0;
		Node temp_node{};
		temp_node.current_node_x = x_start;
		temp_node.current_node_y = y_start;
		temp_node.current_node_z = z_start;
		temp_node.parent_node_x = x_start;
		temp_node.parent_node_y = y_start;
		temp_node.parent_node_z = z_start;
		temp_node.dist_start = 0;
		temp_node.dist_dest = sqrt((x_start - x_dest) * (x_start - x_dest) + (y_start - y_dest) * (y_start - y_dest) + (z_start - z_dest) * (z_start - z_dest));
		temp_node.dist_heur = sqrt((x_start - x_dest) * (x_start - x_dest) + (y_start - y_dest) * (y_start - y_dest) + (z_start - z_dest) * (z_start - z_dest));
		temp_node.is_expanded = true;
		node_array.push_back(temp_node);
		int node_array_index = 1;
		double distance = 0;
		int index = -1;
		while (iterations < max_iterations && (x_node != x_dest || y_node != y_dest || z_node != z_dest))
		{ 
			for (int i = -1; i <= 1; i += 2)
			{
				for (int j = -1; j <= 1; j += 2)
				{
					for (int k = -1; k <= 1; k += 2)
					{
						node_array_index = search_diag_three_dim(x_node, y_node, z_node, x_dest, y_dest, z_dest, i, j, k, distance, node_array_index);
					}
					node_array_index = search_diag_two_dim(x_node, y_node, z_node, x_dest, y_dest, z_dest, i, j, 0, distance, node_array_index);
					node_array_index = search_diag_two_dim(x_node, y_node, z_node, x_dest, y_dest, z_dest, i, 0, j, distance, node_array_index);
					node_array_index = search_diag_two_dim(x_node, y_node, z_node, x_dest, y_dest, z_dest, 0, i, j, distance, node_array_index);

				}
				node_array_index = search_x(x_node, y_node, z_node, x_dest, y_dest, z_dest, i, distance, node_array_index);
				node_array_index = search_y(x_node, y_node, z_node, x_dest, y_dest, z_dest, i, distance, node_array_index);
				node_array_index = search_z(x_node, y_node, z_node, x_dest, y_dest, z_dest, i, distance, node_array_index);
			}
			index = minimum();
			if (index == -1)
			{
				cout << "\nPath does not exist! Potential nodes exhausted! Iterations: "<<iterations<<" Nodes searched: "<<node_array.size()<<endl;
				return;
			}
			else if (node_array[index].is_expanded == true)
			{
				cout << "\nPath does not exist! Potential nodes exhausted! Iterations: "<<iterations<<" Nodes searched: "<<node_array.size()<<endl;
				return;
			}
			node_array[index].is_expanded = true;
			distance = node_array[index].dist_start;
			x_node = node_array[index].current_node_x;
			y_node = node_array[index].current_node_y;
			z_node = node_array[index].current_node_z;
			iterations += 1;
		}
		if (iterations < max_iterations)
		{
			while (x_node != x_start || y_node != y_start || z_node != z_start)
			{ 
				index = check_search(x_node, y_node, z_node);
				x_node = node_array[index].parent_node_x;
				y_node = node_array[index].parent_node_y;
				z_node = node_array[index].parent_node_z;
				rpath.push_back(node_array[index]);
			}
			index = check_search(x_node, y_node, z_node);
			rpath.push_back(node_array[index]);
		}
		else
		{
			cout << "\nPath does not exist! Iteration limit reached! Iterations: "<<iterations<<" Nodes searched: "<<node_array.size()<<endl;
		}
	}
}

