#! /usr/bin/env python

"""
	This node makes the A* Path Planning and publish the path in RViz using markers

"""

# chmod +x call_map_service.py
import rospy
from nav_msgs.srv import GetMap, GetMapRequest
import sys 
import numpy as np
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from geometry_msgs.msg import PoseWithCovarianceStamped

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid


class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind #  parent indice


class GlobalPlanner():
	def __init__(self):
		rospy.init_node('global_planner')
		self.rate = rospy.Rate(30)

		# SUBSCRIBERS

		# Get the cost map
		rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_cb)

		rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_pose_cb)

		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.start_pose_cb)


		# member variables
		self.MAP = None
		self.ROWS = None
		self.COLUMNS = None
		self.RESOLUTION = None

		self.goal = None
		self.start = None

		#self.grid = self.make_grid(self.MAP.data, self.ROWS, self.COLUMNS)
		

		self.loop()

		rospy.spin()


		#
	# global costmap callback
	def global_costmap_cb(self, msg):
		self.MAP = msg

		# Make the map grid only after receive the costmap
		self.grid = self.make_grid(msg.data, msg.info.height, msg.info.width)
		print msg.info.resolution
		print msg.info.height


	def start_pose_cb(self, msg):
		self.start = msg

	def goal_pose_cb(self, msg):
		self.goal = msg


	def make_grid(self, map_data, grid_height, grid_width):
		grid =  np.zeros([grid_height, grid_width])
		# Occupancy grid
		for index in range(len(map_data)):
			row = (int)(index/grid_width)
			column = index%grid_width
			grid[row][column] = map_data[index]

		print "grid shape: "
		print grid.shape
		return grid

	def calc_final_path(self, ngoal, closedset):
		# generate final course
		rx, ry = [ngoal.x ], [ngoal.y]
		pind = ngoal.pind
		while pind != -1:
			n = closedset[pind]
			rx.append(n.x)
			ry.append(n.y)
			pind = n.pind

		return rx, ry

	# A* Planner
	def a_star_planning(self, start_x, start_y, goal_x, goal_y):
	    """
	    start_x: start x position [m]
	    start_y: start y position [m]
	    goal_x: goal x position [m]
	    goal_y: goal y position [m]
	    """

	    # Initialize the start node and the goal node
	    nstart = Node(start_x, start_y, 0.0, -1)
	    ngoal = Node(goal_x, goal_y, 0.0, -1)


	    motion = self.get_motion_model()

	    openset, closedset = dict(), dict()

	    # TODO nstart
	    openset[(start_x,start_y)] = nstart


	    while 1:
	    	# it selects the path that minimizes the function f(n) = g(n) + h(n), g(n) = cost and h(n) = heuristic
			c_id = min(openset, key=lambda n: openset[n].cost + self.heuristic(ngoal, openset[n].x, openset[n].y))
			current = openset[c_id]
			#print("current", current)

	        # verify is the goal was achieved
			if current.x == ngoal.x and current.y == ngoal.y:
				print("Find goal")
				ngoal.pind = current.pind
				ngoal.cost = current.cost
				break
	    
			# Remove the item from the open set
			del openset[c_id]

			# Add it to the closed set
			closedset[c_id] = current

	        # expand search grid based on motion model
	        # create new Nodes
			for i in range(len(motion)):
				node = Node(current.x + motion[i][0], current.y + motion[i][1], current.cost + motion[i][2], c_id)
				# node id?
				n_id = self.calc_index(node)

				# verify node
				if not self.verify_node(node):
				    continue

				if n_id in closedset:
					continue

				# Otherwise if it is already in the open set
				if n_id in openset:
					if openset[n_id].cost > node.cost:
						openset[n_id].cost = node.cost
						openset[n_id].pind = c_id
				else:
					openset[n_id] = node

	    rx, ry = self.calc_final_path(ngoal, closedset)

	    return rx, ry

	# Heuristic
	def heuristic(self, ngoal, x, y):
	    w = 3.0  # weight of heuristic
	    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
	    return d


	def verify_node(self, node):

	    if node.x < 0:
	        return False
	    elif node.y < 0:
	        return False
	    elif node.x >= self.grid.shape[1]:
	        return False
	    elif node.y >= self.grid.shape[0]:
	        return False
	    elif self.grid[node.y][node.x] > 50:
	    	return False
	   
	    return True

	def calc_index(self, node):
		return (node.y, node.x)

	def get_motion_model(self):
	    # dx, dy, cost
	    motion = [[1, 0, 1],
	              [0, 1, 1],
	              [-1, 0, 1],
	              [0, -1, 1],
	              [-1, -1, math.sqrt(2)],
	              [-1, 1, math.sqrt(2)],
	              [1, -1, math.sqrt(2)],
	              [1, 1, math.sqrt(2)]]

	    return motion


	def init_markers(self):

		# Set up our waypoint markers
		marker_scale = 0.05
		marker_lifetime = 0 # 0 is forever
		marker_ns = 'waypoints'

		marker_id = 1

		marker_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}


		# Define a marker publisher.

		#self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=100)
		#self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1000)


		# Initialize the marker points list.

		self.markers = Marker()
		self.markers.ns = marker_ns
		self.markers.id = marker_id
		self.markers.type = Marker.POINTS
		self.markers.action = Marker.ADD
		self.markers.lifetime = rospy.Duration(marker_lifetime)

		self.markers.scale.x = marker_scale

		self.markers.scale.y = marker_scale

		self.markers.color.r = marker_color['r']
		self.markers.color.g = marker_color['g']
		self.markers.color.b = marker_color['b']
		self.markers.color.a = marker_color['a']
		self.markers.header.frame_id = '/map'
		self.markers.header.stamp = rospy.Time.now()
		self.markers.points = list()


	def loop(self):
		
		while not rospy.is_shutdown():
			rospy.wait_for_message('move_base_simple/goal', PoseStamped)

			## TODO: delete previous result path if it exists
			#deleteMarker(marker_id)

			# Print " We have a new goal!"



			# Initialize the visualization markers for RViz
			#self.init_markers()

			start_x = self.start.pose.pose.position.x
			start_y = self.start.pose.pose.position.y
			goal_x = self.goal.pose.position.x
			goal_y = self.goal.pose.position.y 

			# convert to cell
			print ""
			print start_x, start_y, goal_x, goal_y

			# TODO: Given the robot's pose in the map frame, if you want the corresponding index into the occupancy grid map, you'd do something like this:
			start_grid_x = int((start_x - self.MAP.info.origin.position.x) / self.MAP.info.resolution)
			start_grid_y = int((start_y - self.MAP.info.origin.position.y) / self.MAP.info.resolution)

			goal_grid_x = int((goal_x - self.MAP.info.origin.position.x) / self.MAP.info.resolution)
			goal_grid_y = int((goal_y - self.MAP.info.origin.position.y) / self.MAP.info.resolution)

			print start_grid_x, start_grid_y, goal_grid_x, goal_grid_y
			print ""

			rx ,ry = self.a_star_planning(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y)
			points_list = []
			#print rx
			#print ry
			
			i = 0
			marker_pub = rospy.Publisher('/a_star', Marker, queue_size=1000)

			
			while(i < len(rx)):			


				#self.markers.points.append(p)
				#self.marker_pub.publish(self.markers)

				points = Marker()
				#points.header.frame_id = "/my_frame"
				points.header.frame_id = "/map"
				points.header.stamp = rospy.Time.now()
				points.ns = "points_and_lines"
				points.action = points.ADD
				points.pose.orientation.w = 1.0
				points.id = 1
				points.type = points.POINTS
				points.scale.x = 0.05
				points.scale.y = 0.05
				points.scale.z = 0.05
				points.color.r = 1.0
				points.color.a = 1.0

				p = Point()

				p.x = rx[i]*self.MAP.info.resolution + self.MAP.info.origin.position.x
				p.y = ry[i]*self.MAP.info.resolution + self.MAP.info.origin.position.y
				p.z = 0.0

				points_list.append(p)

				points.points = points_list

				marker_pub.publish(points)
				i = i + 1
							
			## TODO: update the goal
			#print points_list
			self.rate.sleep()


if __name__ == '__main__':
	try:
		GlobalPlanner()
	except rospy.ROSInterruptException:
		rospy.loginfo("Global Planner finished.")

