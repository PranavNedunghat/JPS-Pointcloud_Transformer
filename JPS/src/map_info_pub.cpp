#include "ros/ros.h"
#include "JPS/mapinfo.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "map_info_pub");
  
  ros::NodeHandle n;

  ros::Publisher info_pub = n.advertise<JPS::mapinfo>("talk",1000);
  ros::Rate loop_rate(10);
  
  int map[20][20][20];
  
  while (ros::ok())
  {
    JPS::mapinfo msg;
    msg.xstart = 0;
    msg.ystart = 1;
    msg.zstart = 1;
    msg.xdest = 18;
    msg.ydest = 19;
    msg.zdest = 19;
    for (int i = 0; i < 20; i++)
	{
		for (int j = 0; j < 20; j++)
		{
			for (int k = 0; k < 20; k++)
			{
				map[i][j][k] = 1;
			}
		}
	}
	for (int i = 5; i < 10; i++)
	{
		for (int j = 5; j < 10; j++)
		{
			for (int k = 5; k < 10; k++)
			{
				map[i][j][k] = 0;
			}
		}
	}
	for (int j = 3; j < 18; j++)
	{
		for (int k = 3; k < 18; k++)
		{
			map[j][k][17] = 0;
		}
	}
    for (int i = 0; i < 20; i++)
	{
		for (int j = 0; j < 20; j++)
		{for (int k = 0; k < 20; k++)
		{
			msg.gridmap.push_back(map[i][j][k]);
		}
		}
	}
	info_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	}
	return 0;
	}
