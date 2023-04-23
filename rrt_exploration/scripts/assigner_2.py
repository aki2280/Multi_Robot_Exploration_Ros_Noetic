#!/usr/bin/env python3

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm
from math import sqrt

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]
vel=0.0
not_first_callback = False
last_callback_time = 0
Integral_sum_L=0
Integral_sum_R=0
Integral_sum=0


def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))
		

def mapCallBack(data):
    global mapData
    mapData=data




def velCallback(data):
	global vel, robot_namelist
	global not_first_callback, last_callback_time, Integral_sum, Integral_sum_L, Integral_sum_R
	distance_between_wheels = 0.287
	wheel_radius = 0.033

	
	topic_breakdownlist = str(data._connection_header['topic']).split('/')
	vel_linear=(sqrt((data.linear.x)**2 + (data.linear.y)**2 + (data.linear.z)**2))
	vel_angular=(sqrt((data.angular.x)**2 + (data.angular.y)**2 + (data.angular.z)**2))

	angular_L = sqrt(((vel_linear-(vel_angular*distance_between_wheels/2))/wheel_radius)**2)
	angular_R = sqrt(((vel_linear+(vel_angular*distance_between_wheels/2))/wheel_radius)**2)


	if not_first_callback:
		time_since_last_callback= rospy.get_time() - last_callback_time
		Integral_sum_L+= 10*angular_L*time_since_last_callback
		Integral_sum_R+= 10*angular_R*time_since_last_callback
		Integral_sum+= (Integral_sum_L + Integral_sum_R)
		print("Integral", Integral_sum)
	not_first_callback= True
	last_callback_time= rospy.get_time()
	
	
		
# Node----------------------------------------------

def node():
	global frontiers,mapData,globalmaps
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic								= rospy.get_param('~map_topic','/map')
	info_radius							= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier					= rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius				= rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain					= rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic					= rospy.get_param('~frontiers_topic','/filtered_points')	
	delay_after_assignement	= rospy.get_param('~delay_after_assignement',0.5)
	rateHz 									= rospy.get_param('~rate',100)
	robot_namelist          = rospy.get_param('~robot_namelist', "robot1")
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)

#---------------------------------------------------------------------------------------------------------------
	# perform name splitting for the robot
	robot_namelist = robot_namelist.split(',')
	
	rospy.Subscriber('tb3_1/cmd_vel', Twist, velCallback)

	pub = rospy.Publisher('battery_plot', Float32, queue_size=100)


# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)
	

#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[]
	for i in range(0,len(robot_namelist)):
		robots.append(robot(name=robot_namelist[i]))

	for i in range(0,len(robot_namelist)):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		
		centroids=copy(frontiers)	

		pub.publish(Integral_sum)	
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,len(robot_namelist)):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)	
		rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
		print("Assigned", robots[1].assigned_point)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]

		'''for i in range(0, len(robot_namelist)):
			rospy.Subscriber(robot_namelist[i] + '/cmd_vel', Vector3, velCallback)'''


		

		
		for ir in na:
			for ip in range(0,len(centroids)):
				cost=norm(robots[ir].assigned_point-centroids[ip])		
				threshold=1
				information_gain=infoGain[ip]
				if (norm(robots[ir].assigned_point-centroids[ip])<=hysteresis_radius):

					information_gain*=hysteresis_gain
				revenue=information_gain*info_multiplier-cost
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)
		
		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		rospy.loginfo("revenue record: "+str(revenue_record))	
		rospy.loginfo("centroid record: "+str(centroid_record))	
		# rospy.loginfo("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			rospy.loginfo(robot_namelist[id_record[winner_id]] + "  assigned to  "+str(centroid_record[winner_id]))	
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
