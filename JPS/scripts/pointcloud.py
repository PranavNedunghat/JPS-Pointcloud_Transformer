#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from JPS.msg import mapinfo, pathpoints
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import quaternion_matrix
ogm_rel = [[[1 for i in range(20)] for j in range(20)] for k in range(20)]

def find_free_node(x,y,z):
	if (ogm_rel[x][y][z] == 1):
		return (x,y,z)
	scope = 1
	while(scope < 20):
		for i in range(-scope,scope,2*scope):
			for j in range(-scope,scope,2*scope):
				for k in range(-scope,scope,2*scope):
					if (((x + scope)>=0) and ((x + scope)<20) and ((y + scope)>=0) and ((y + scope)<20) and ((z + scope)>=0) and ((z + scope)<20)):
						if (ogm_rel[x+scope][y+scope][z+scope] == 1):
							return (x+scope,y+scope,z+scope)
		scope += 1
	rospy.loginfo("Unable to find the nearest unoccupied cell!\n")
	return (-1,-1,-1)
				

class Transformer:
	
	def __init__(self):
		self.position = None
		self.orientation = None
		self.Rot_matrix = None
		self.Trans_matrix = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
		self.inv_Trans_matrix = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
		self.pubr = rospy.Publisher("Visualize",Marker,queue_size = 100)
		self.pubo = rospy.Publisher("OccupancyGrid",mapinfo,queue_size = 100)

	def point_cloud_callback(self,data):
		global Obs
		Obs = Marker(
		  header = Header(frame_id = 'world',stamp = rospy.Time.now()),
		  ns = "pointcloudTransformer",
		  action = Marker.ADD,
		  id = 1,
		  type = Marker.POINTS,
		  color = ColorRGBA(1.0,0.0,0.0,1.0)
		  )
		Obs.scale.x = 1.0
		Obs.scale.y = 1.0
		Obs.pose.orientation.w = 1.0
		for point in pc2.read_points(data,skip_nans = True,field_names = ("x", "y", "z")):
			#rospy.loginfo("%f, %f, %f\n" % (point[0],point[1],point[2]))
			point = [int(point[i])for i in range(len(point))]
			p = Point()
			p.x = point[0]
			p.y = point[1]
			p.z = point[2]
			Obs.points.append(p)
			new_point = np.array([point[0],point[1],point[2],1.0])
			#print(self.inv_Trans_matrix)
			new_point = np.matmul(self.inv_Trans_matrix,new_point)
			new_point = [(new_point[i]+10) for i in range(len(new_point)-1)]
			 
			if ((new_point[0] >= 0 and new_point[0] < 20) and (new_point[1] >= 0 and new_point[1] < 20) and (new_point[2] >= 0 and new_point[2] < 20)):
				ogm_rel[int(new_point[0])][int(new_point[1])][int(new_point[2])] = 0
		

				
		
	def pose_callback(self,Pose):
		self.position = Pose.pose.pose.position
		self.orientation = Pose.pose.pose.orientation
		self.Rot_matrix = quaternion_matrix([self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w])
		self.Trans_matrix = np.array([[self.Rot_matrix[0,0],self.Rot_matrix[0,1],self.Rot_matrix[0,2],self.position.x],[self.Rot_matrix[1,0],self.Rot_matrix[1,1],self.Rot_matrix[1,2],self.position.y],[self.Rot_matrix[2,0],self.Rot_matrix[2,1],self.Rot_matrix[2,2],self.position.z],[0.0,0.0,0.0,1.0]])
		self.inv_Trans_matrix = np.linalg.inv(self.Trans_matrix)

	def ogm_publisher(self):
		msg = mapinfo()
		pos = np.array([self.Trans_matrix[0,3],self.Trans_matrix[1,3],self.Trans_matrix[2,3],1.0])
		pos = np.matmul(self.inv_Trans_matrix,pos)
		msg.xstart = int(pos[0]+10)
		msg.ystart = int(pos[1]+10)
		msg.zstart = int(pos[2]+10)
		dest = np.array([19.0,19.0,10.0,1.0]);
		dest_rel = np.matmul(self.inv_Trans_matrix,dest)
		msg.xdest = int(dest_rel[0])
		msg.ydest = int(dest_rel[1])
		msg.zdest = int(dest_rel[2])
		#rospy.loginfo("%d, %d, %d\n" % (msg.xstart,msg.ystart,msg.zstart))
		if (msg.xdest < 0):
			msg.xdest = 0
		elif (msg.xdest > 19):
			msg.xdest = 19
		
		if (msg.ydest < 0):
			msg.ydest = 0
		elif (msg.ydest > 19):
			msg.ydest = 19
		
		if (msg.zdest < 0):
			msg.zdest = 0
		elif (msg.zdest > 19):
			msg.zdest = 19
			
		(msg.xdest,msg.ydest,msg.zdest) = find_free_node(msg.xdest,msg.ydest,msg.zdest)
			

		for i in range(20):
			for j in range(20):
				for k in range(20):
					msg.gridmap.append(ogm_rel[i][j][k])
						#rospy.loginfo("%f, %f, %f\n" % (self.p.x,self.p.y,self.p.z))

		#for i in range(20):
			#for j in range(20):
				#for k in range(20):
					#msg.gridmap.append(ogm[i][j][k])
		#for i in range(len(Points.points)):
			#rospy.loginfo("i is : %d \n %f, %f, %f\n" %(i,Points.points[i].x,Points.points[i].y,Points.points[i].z))
		if (ogm_rel[msg.xstart][msg.ystart][msg.zstart] == 0):
			rospy.loginfo("Start Node is an occupied cell!! Will not publish!\n")
		elif ((msg.xdest == -1)):
			rospy.loginfo("Could not find unoccupied Destination Node!! Will not publish!\n")
		else:
			rospy.loginfo("Destination is x: %d, y: %d, z: %d\n" %(msg.xdest,msg.ydest,msg.zdest))
			self.pubo.publish(msg)
			
			
	def jps_subscriber_and_publisher(self,point_msg):
		Lines = Marker(
		    header = Header(frame_id = 'world',stamp = rospy.Time.now()),
		    ns = "pointcloudTransformer",
		    action = Marker.ADD,
		    id = 2,
		    type = Marker.LINE_STRIP,
		    color = ColorRGBA(0.0,0.0,1.0,1.0)
		    )
		Lines.scale.x = 0.1
		Lines.pose.orientation.w = 1.0
		Points = Marker(
		    header = Header(frame_id = 'world',stamp = rospy.Time.now()),
		    ns = "pointcloudTransformer",
		    action = Marker.ADD,
		    id = 1,
		    type = Marker.POINTS,
		    color = ColorRGBA(0.0,1.0,0.0,1.0)
		    )
		Points.scale.x = 0.2
		Points.scale.y = 0.2
		Points.pose.orientation.w = 1.0
		for i in range(len(point_msg.x)):
			new_point = np.array([(point_msg.x[i]-10),(point_msg.y[i]-10),(point_msg.z[i]-10),1.0])
			new_point = np.matmul(self.Trans_matrix,new_point)
			ptr = Point()
			ptr.x = new_point[0]
			ptr.y = new_point[1]
			ptr.z = new_point[2]
			Points.points.append(ptr)
			Lines.points.append(ptr)
		self.pubr.publish(Obs)
		self.pubr.publish(Points)
		self.pubr.publish(Lines)

	
	
if __name__ == '__main__':
	rospy.init_node("pointcloudTransformer", anonymous = True)
	rate = rospy.Rate(10)
	Trans = Transformer()
	while not rospy.is_shutdown():
		rospy.Subscriber("/quadrotor/odom",Odometry,Trans.pose_callback)
		rospy.Subscriber("/voxblox_node/surface_pointcloud", PointCloud2,Trans.point_cloud_callback)
		Trans.ogm_publisher()
		rospy.Subscriber("JPS_Path", pathpoints, Trans.jps_subscriber_and_publisher)
		rate.sleep()
