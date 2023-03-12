#include<iostream>
#include<stdio.h>
#include<cstdlib>
#include<cmath>
#include<vector>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include "JPS/mapinfo.h"
#include "JPS/pathpoints.h"
using namespace std;
int ogm[20][20][20];
int x_beginning,y_beginning,z_beginning, x_end, y_end,z_end;
void subscriberCallback(const JPS::mapinfo& msg)
{
  x_beginning = msg.xstart;
  y_beginning = msg.ystart;
  z_beginning = msg.zstart;
  x_end = msg.xdest;
  y_end = msg.ydest;
  z_end = msg.zdest;
  for (int i = 0; i < 20; i++)
  { for (int j = 0; j < 20; j++)
   { for (int k = 0; k < 20; k++)
    {
     ogm[i][j][k] = msg.gridmap[i*400+j*20+k];
    }
   }
  }
}
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
};
vector <Node> node_array;
vector <Node> rpath;
int check_search(int x, int y, int z, int mode)
{
	int found = 0;
	int index = -1;
	for (int i = 0; i < node_array.size(); i++)
	{
		if ((node_array[i].current_node_x == x) && (node_array[i].current_node_y == y) && (node_array[i].current_node_z == z))
		{
			found = 1;
			index = i;
			break;
		}
	}
	if (mode == 0)
	{
		return found;
	}
	else if (mode == 1)
	{
		return index;
	}
	return index;
}

int minimum()
{
	double min = 9999999.9;
	int index = -1;
	for (int i = 0; i < node_array.size(); i++)
	{
		if (node_array[i].dist_heur < min)
		{
			index = i;
			min = node_array[i].dist_heur;
		}
	}
	return index;
}

int search_x(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, double distance, int node_array_index)
{
	int x_node = x;
	int y_node = y;
	int z_node = z;
	int x_dest = xdest;
	int y_dest = ydest;
	int z_dest = zdest;
	double dist = distance;
	int map_x = 20;
	int map_y = 20;
	int map_z = 20;
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
				f = check_search(x2, y_node - 1, z_node, 0);
				if (f == 0)
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
			}
		}
		if ((y_node + 1 < map_y) && (x2 >= 0) && (x2 < map_x))
		{
			if (ogm[x1][y_node + 1][z_node] == 0 && ogm[x2][y_node + 1][z_node] != 0)
			{
				f = check_search(x2, y_node + 1, z_node, 0);
				if (f == 0)
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
			}
		}
		if ((z_node - 1 >= 0) && (x2 >= 0) && (x2 < map_x))
		{
			if (ogm[x1][y_node][z_node - 1] == 0 && ogm[x2][y_node][z_node - 1] != 0)
			{
				f = check_search(x2, y_node, z_node - 1, 0);
				if (f == 0)
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
			}
		}
		if ((z_node + 1 < map_z) && (x2 >= 0) && (x2 < map_x))
		{
			if (ogm[x1][y_node][z_node + 1] == 0 && ogm[x2][y_node][z_node + 1] != 0)
			{
				f = check_search(x2, y_node, z_node + 1, 0);
				if (f == 0)
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
			}
		}
		if (flag == 1)
		{
			f = check_search(x1, y_node, z_node, 0);
			if (f == 0)
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
		}

	}
	return i;
}

int search_y(int x, int y, int z, int xdest, int ydest, int zdest, int y_dist, double distance, int node_array_index)
{
	int x_node = x;
	int y_node = y;
	int z_node = z;
	int x_dest = xdest;
	int y_dest = ydest;
	int z_dest = zdest;
	double dist = distance;
	int map_x = 20;
	int map_y = 20;
	int map_z = 20;
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
				f = check_search(x_node - 1, y2, z_node, 0);
				if (f == 0)
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
			}
		}
		if ((x_node + 1 < map_x) && (y2 >= 0) && (y2 < map_y))
		{
			if (ogm[x_node + 1][y1][z_node] == 0 && ogm[x_node + 1][y2][z_node] != 0)
			{
				f = check_search(x_node + 1, y2, z_node, 0);
				if (f == 0)
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
			}
		}
		if ((z_node - 1 >= 0) && (y2 >= 0) && (y2 < map_y))
		{
			if (ogm[x_node][y1][z_node - 1] == 0 && ogm[x_node][y2][z_node - 1] != 0)
			{
				f = check_search(x_node, y2, z_node - 1, 0);
				if (f == 0)
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
			}
		}
		if ((z_node + 1 < map_z) && (y2 >= 0) && (y2 < map_y))
		{
			if (ogm[x_node][y1][z_node + 1] == 0 && ogm[x_node][y2][z_node + 1] != 0)
			{
				f = check_search(x_node, y2, z_node + 1, 0);
				if (f == 0)
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
			}
		}
		if (flag == 1)
		{
			f = check_search(x_node, y1, z_node, 0);
			if (f == 0)
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
		}

	}
	return i;
}

int search_z(int x, int y, int z, int xdest, int ydest, int zdest, int z_dist, double distance, int node_array_index)
{	
	int x_node = x;
	int y_node = y;
	int z_node = z;
	int x_dest = xdest;
	int y_dest = ydest;
	int z_dest = zdest;
	double dist = distance;
	int map_x = 20;
	int map_y = 20;
	int map_z = 20;
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
			break;
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
				f = check_search(x_node - 1, y_node, z2, 0);
				if (f == 0)
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
			}
		}
		if ((x_node + 1 < map_x) && (z2 >= 0) && (z2 < map_z))
		{
			if (ogm[x_node + 1][y_node][z1] == 0 && ogm[x_node + 1][y_node][z2] != 0)
			{
				f = check_search(x_node + 1, y_node, z2, 0);
				if (f == 0)
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
			}
		}
		if ((y_node - 1 >= 0) && (z2 >= 0) && (z2 < map_z))
		{
			if (ogm[x_node][y_node - 1][z1] == 0 && ogm[x_node][y_node - 1][z2] != 0)
			{
				f = check_search(x_node, y_node - 1, z2, 0);
				if (f == 0)
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
			}
		}
		if ((y_node + 1 < map_y) && (z2 >= 0) && (z2 < map_z))
		{
			if (ogm[x_node][y_node + 1][z1] == 0 && ogm[x_node][y_node + 1][z2] != 0)
			{
				f = check_search(x_node, y_node + 1, z2, 0);
				if (f == 0)
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
			}
		}
		if (flag == 1)
		{
			f = check_search(x_node, y_node, z1, 0);
			if (f == 0)
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
		}

	}
	return i;
}


int search_diag_two_dim(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, int y_dist, int z_dist, double distance, int node_array_index)
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
	int map_x = 20;
	int map_y = 20;
	int map_z = 20;
	int i, in;
	i = in = node_array.size();
	double diag_dist = 0;
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
		dist = dist + 1.414;
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
					f = check_search(x1 - x_dist, y2, z1, 0);
					if (f == 0)
					{
						temp_node.current_node_x = x1 - x_dist;
						temp_node.current_node_y = y2;
						temp_node.current_node_z = z1;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist;
						temp_node.dist_dest = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
						temp_node.dist_heur = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist;
						i += 1;
						node_array.push_back(temp_node);
						flag = 1;
					}
				}
			}
			if ((x2 >= 0 && x2 < map_x) && ((y1 - y_dist >=0) && (y1 - y_dist < map_y)))
			{
				if ((ogm[x1][y1 - y_dist][z1] == 0) && ogm[x2][y1 - y_dist][z1] != 0)
				{
					f = check_search(x2, y1 - y_dist, z1, 0);
					if (f == 0)
					{
						temp_node.current_node_x = x2;
						temp_node.current_node_y = y1 - y_dist;
						temp_node.current_node_z = z1;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist;
						temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
						temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dist - y_dest) * (y1 - y_dist - y_dest) + (z1 - z_dest) * (z1 - z_dest))) + dist;
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
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
					f = check_search(x1 - x_dist, y2, z1, 0);
					if (f == 0)
					{
						temp_node.current_node_x = x1 - x_dist;
						temp_node.current_node_y = y1;
						temp_node.current_node_z = z2;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist;
						temp_node.dist_dest = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest)));
						temp_node.dist_heur = double(sqrt(((x1 - x_dist) - x_dest) * (x1 - x_dist - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist;
						i += 1;
						node_array.push_back(temp_node);
						flag = 1;
					}
				}
			}
			if ((x2 >= 0 && x2 < map_x) && ((z1 - z_dist >=0) && (z1 - z_dist < map_z)))
			{
				if ((ogm[x1][y1][z1 - z_dist] == 0) && ogm[x2][y1][z1 - z_dist] != 0)
				{
					f = check_search(x2, y1, z1 - z_dist, 0);
					if (f == 0)
					{
						temp_node.current_node_x = x2;
						temp_node.current_node_y = y1;
						temp_node.current_node_z = z1 - z_dist;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist;
						temp_node.dist_dest = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest)));
						temp_node.dist_heur = double(sqrt((x2 - x_dest) * (x2 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist;
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
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
					f = check_search(x1, y1 - y_dist, z2, 0);
					if (f == 0)
					{
						temp_node.current_node_x = x1;
						temp_node.current_node_y = y1 - y_dist;
						temp_node.current_node_z = z2;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist;
						temp_node.dist_dest = double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest)));
						temp_node.dist_heur = double(sqrt(((y1 - y_dist) - y_dest) * (y1 - y_dist - y_dest) + (x1 - x_dest) * (x1 - x_dest) + (z2 - z_dest) * (z2 - z_dest))) + dist;
						i += 1;
						node_array.push_back(temp_node);
						flag = 1;
					}
				}
			}
			if ((y2 >= 0 && y2 < map_y) && ((z1 - z_dist >=0) && (z1 - z_dist < map_z)))
			{
				if ((ogm[x1][y1][z1 - z_dist] == 0) && ogm[x1][y2][z1 - z_dist] != 0)
				{
					f = check_search(x1, y2, z1 - z_dist, 0);
					if (f == 0)
					{
						temp_node.current_node_x = x1;
						temp_node.current_node_y = y2;
						temp_node.current_node_z = z1 - z_dist;
						temp_node.parent_node_x = x1;
						temp_node.parent_node_y = y1;
						temp_node.parent_node_z = z1;
						temp_node.dist_start = dist;
						temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest)));
						temp_node.dist_heur = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y2 - y_dest) * (y2 - y_dest) + (z1 - z_dist - z_dest) * (z1 - z_dist - z_dest))) + dist;
						node_array.push_back(temp_node);
						i += 1;
						flag = 1;
					}
				}
			}
		}
		if (flag == 1)
		{
			f = check_search(x1, y1, z1, 0);
			if (f == 0)
			{
				temp_node.current_node_x = x1;
				temp_node.current_node_y = y1;
				temp_node.current_node_z = z1;
				temp_node.parent_node_x = x_node;
				temp_node.parent_node_y = y_node;
				temp_node.parent_node_z = z_node;
				temp_node.dist_start = distance + double(sqrt((x1 - x_node) * (x1 - x_node) + (y1 - y_node) * (y1 - y_node) + (z1 - z_node) * (z1 - z_node)));
				temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
				temp_node.dist_heur = distance + double(sqrt((x1 - x_node) * (x1 - x_node) + (y1 - y_node) * (y1 - y_node) + (z1 - z_node) * (z1 - z_node))) + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
				node_array.push_back(temp_node);
				i += 1;
				flag = 0;
				flag1 = 1;
			}
		}
		in = i;
		if (z_dist == 0)
		{
			i = search_x(x1, y1, z1, xdest, ydest, zdest, x_dist, dist, i);
			i = search_y(x1, y1, z1, xdest, ydest, zdest, y_dist, dist, i);
		}
		else if (y_dist == 0)
		{
			i = search_x(x1, y1, z1, xdest, ydest, zdest, x_dist, dist, i);
			i = search_z(x1, y1, z1, xdest, ydest, zdest, z_dist, dist, i);
		}
		else if (x_dist == 0)
		{
			i = search_y(x1, y1, z1, xdest, ydest, zdest, y_dist, dist, i);
			i = search_z(x1, y1, z1, xdest, ydest, zdest, z_dist, dist, i);
		}
		f = check_search(x1, y1, z1, 0);
		if (flag1 == 0 && in != i && f == 0)
		{
			temp_node.current_node_x = x1;
			temp_node.current_node_y = y1;
			temp_node.current_node_z = z1;
			temp_node.parent_node_x = x_node;
			temp_node.parent_node_y = y_node;
			temp_node.parent_node_z = z_node;
			temp_node.dist_start = distance + double(sqrt((x1 - x_node) * (x1 - x_node) + (y1 - y_node) * (y1 - y_node) + (z1 - z_node) * (z1 - z_node)));
			temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
			temp_node.dist_heur = distance + double(sqrt((x1 - x_node) * (x1 - x_node) + (y1 - y_node) * (y1 - y_node) + (z1 - z_node) * (z1 - z_node))) + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
			node_array.push_back(temp_node);
			i += 1;
		}
	}
	return i;
}

int search_diag_three_dim(int x, int y, int z, int xdest, int ydest, int zdest, int x_dist, int y_dist, int z_dist, double distance, int node_array_index)
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
	int map_x = 20;
	int map_y = 20;
	int map_z = 20;
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
		i = search_diag_two_dim(x1, y1, z1, x_dest, y_dest, z_dest, x_dist, y_dist, 0, dist + diag_dist, i);
		i = search_diag_two_dim(x1, y1, z1, x_dest, y_dest, z_dest, x_dist, 0, z_dist, dist + diag_dist, i);
		i = search_diag_two_dim(x1, y1, z1, x_dest, y_dest, z_dest, 0, y_dist, z_dist, dist + diag_dist, i);
		f = check_search(x1, y1, z1, 0);
		if (in != i && f == 0)
		{
			temp_node.current_node_x = x1;
			temp_node.current_node_y = y1;
			temp_node.current_node_z = z1;
			temp_node.parent_node_x = x_node;
			temp_node.parent_node_y = y_node;
			temp_node.parent_node_z = z_node;
			temp_node.dist_start = distance + double(sqrt((x1 - x_node) * (x1 - x_node) + (y1 - y_node) * (y1 - y_node) + (z1 - z_node) * (z1 - z_node)));
			temp_node.dist_dest = double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
			temp_node.dist_heur = distance + double(sqrt((x1 - x_node) * (x1 - x_node) + (y1 - y_node) * (y1 - y_node) + (z1 - z_node) * (z1 - z_node))) + double(sqrt((x1 - x_dest) * (x1 - x_dest) + (y1 - y_dest) * (y1 - y_dest) + (z1 - z_dest) * (z1 - z_dest)));
			node_array.push_back(temp_node);
			i += 1;
		}

	}

	return i;
}

void Jump_Point_Search(int start[], int target[])
{
	int x_node, x_start, y_node, y_start, z_node, z_start;
	x_node = x_start = start[0];
	y_node = y_start = start[1];
	z_node = z_start = start[2];
	int x_dest = target[0];
	int y_dest = target[1];
	int z_dest = target[2];
	if (ogm[x_start][y_start][z_start] == 0 || ogm[x_dest][y_dest][z_dest] == 0)
	{ cout << "Start or Destination is in an occupied cell. Path not possible";
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
	node_array.push_back(temp_node);
	int node_array_index = 1;
	double distance = 0;
	int index = -1;
	while (iterations < 1000 && (x_node != x_dest || y_node != y_dest) || z_node != z_dest)
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
			node_array_index = search_z(x_node, y_node, z_node, z_dest, y_dest, z_dest, i, distance, node_array_index);
		}

		index = minimum();
		if (index == -1)
		{
			cout << "\nPath does not exist!";
			return;
		}
		else if (node_array[index].dist_heur >= 9999999.9)
		{
			cout << "\nPath does not exist!";
			return;
		}
		node_array[index].dist_heur = 9999999.9;
		distance = node_array[index].dist_start;
		x_node = node_array[index].current_node_x;
		y_node = node_array[index].current_node_y;
		z_node = node_array[index].current_node_z;
		iterations += 1;
	}
	if (iterations < 1000)
	{
		while (x_node != x_start || y_node != y_start || z_node != z_start)
		{
			index = check_search(x_node, y_node, z_node, 1);
			x_node = node_array[index].parent_node_x;
			y_node = node_array[index].parent_node_y;
			z_node = node_array[index].parent_node_z;
			rpath.push_back(node_array[index]);
		}
		index = check_search(x_node, y_node, z_node, 1);
		rpath.push_back(node_array[index]);
	}
	else
	{
		cout << "\nPath does not exist!";
	}
}

int main( int argc, char** argv)
{
  ros::init(argc, argv, "Jump_Point_Search");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<JPS::pathpoints>("JPS_Path", 10);
  ros::Subscriber sub = n.subscribe("OccupancyGrid",1000,subscriberCallback);

  ros::Rate r(10);
  
  int flag = 0;
  while (flag == 0)
  { ros::spinOnce();
   for (int i = 0; i < 20; i++)
   	{
   	 for (int j = 0; j < 20; j++)
	   { for (int k = 0; k < 20; k++)
	   	{
		if (ogm[i][j][k] == 1)
		{
		 flag += 1;
		}
		}
	   }
        }
  }
  	
	int start[3] = { x_beginning,y_beginning,z_beginning };
	int target[3] = { x_end, y_end,z_end };
	Jump_Point_Search(start, target);
  while (ros::ok())
  {
    //visualization_msgs::Marker points, line_strip, obs_points;
    //points.header.frame_id = line_strip.header.frame_id = obs_points.header.frame_id = "my_frame";
    //points.header.stamp = line_strip.header.stamp = obs_points.header.stamp = ros::Time::now();
    //points.ns = line_strip.ns = obs_points.ns = "Jump_Point_Search";
    //points.action = line_strip.action = obs_points.action = visualization_msgs::Marker::ADD;
    //points.pose.orientation.w = line_strip.pose.orientation.w = obs_points.pose.orientation.w= 1.0;


    //points.id = 0;
    //line_strip.id = 1;
    //obs_points.id = 2;

    //points.type = visualization_msgs::Marker::POINTS;
    //line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    //obs_points.type = visualization_msgs::Marker::POINTS;

    //points.scale.x = 0.2;
    //points.scale.y = 0.2;

    //line_strip.scale.x = 0.1;
    //obs_points.scale.x = 1.0;
    //obs_points.scale.y = 1.0;

    //points.color.g = 1.0;
    //points.color.a = 1.0;

    //line_strip.color.b = 1.0;
    //line_strip.color.a = 1.0;
    
    //obs_points.color.r = 1.0;
    //obs_points.color.a = 1.0;
	
	//for (int k = rpath.size()-1; k >= 0; k--)
	//{
	//	geometry_msgs::Point p;
	//	p.x = rpath[k].current_node_x - 10;
	//	p.y = rpath[k].current_node_y - 10;
	//	p.z = rpath[k].current_node_z;
	//	points.points.push_back(p);
	//	line_strip.points.push_back(p);
	//}
	//for (int i = 0; i < 20; i++)
	//{for (int j = 0; j < 20; j++)
	 //{for (int k = 0; k < 20; k++)
	 //{if(ogm[i][j][k] == 0)
	  //{
	 //	geometry_msgs::Point obs;
	 //	obs.x = i - 10;
	 //	obs.y = j - 10;
	 //	obs.z = k;
	 //	obs_points.points.push_back(obs);
	  //}
	 //}
	 //}
	//}
	 //marker_pub.publish(points);
         //marker_pub.publish(line_strip);
         //marker_pub.publish(obs_points);
         
         JPS::pathpoints path;
         for (int k = rpath.size() - 1; k>= 0; k--)
         {
         	path.x.push_back(rpath[k].current_node_x);
         	path.y.push_back(rpath[k].current_node_y);
         	path.z.push_back(rpath[k].current_node_z);
         }
         pub.publish(path);
         ROS_INFO("Publishing..");
         r.sleep();
  }
  return 0;
}

