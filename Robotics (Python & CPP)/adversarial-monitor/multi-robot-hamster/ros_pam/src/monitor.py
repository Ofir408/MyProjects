#!/usr/bin/python

import random, yaml, sys, os.path
import rospy, rosnode, dynamic_reconfigure.server
from nav_msgs.msg import OccupancyGrid, Odometry
from ros_pam.msg import TriggerReport
from ros_pam.cfg import TrackerConfig
from definitions import Definitions
from pathWriter import PathWriter
from email_sender import EmailSender
from threats_map_printer import ThreatsMapPrinter
from std_msgs.msg  import Bool

## Holds the threat map (currently as OccupancyGrid msg).
#  Creates a tracker object per robot and updates the threat map from a /threat_grid topic.
class Monitor(object):
	THREAT_MAP_TOPIC_NAME = "threat_grid"

	def __init__(self, robots_to_track_dict):
		# wait until the threat map is loaded (that is, until map server became online) and set it.
		try:
			self.threat_map = rospy.wait_for_message(self.THREAT_MAP_TOPIC_NAME, OccupancyGrid)
			threats_map_printer = ThreatsMapPrinter(self.threat_map, Tracker.NORM_FACTOR)
			threats_map_printer.write_threats_prob()
		except rospy.ROSInterruptException:
			sys.exit("Received shutdown signal...")
		# subscribe to THREAT_MAP_TOPIC_NAME
		self.map_updater = rospy.Subscriber(self.THREAT_MAP_TOPIC_NAME, OccupancyGrid, self.update_threat_map_callback)
		self.trackers = []
		for robot_ns,topic_name in robots_to_track_dict.items():
			self.trackers.append(Tracker(robot_ns, topic_name, self.threat_map))

	def __del__(self):
		# sending the report email about the robot that was caught.
		"""
		es = EmailSender()
		es.send_email()
		"""

	# updates the threat map
	def update_threat_map_callback(self, map_message):
		if self.threat_map.info.map_load_time < map_message.info.map_load_time:
			self.threat_map = map_message



## Tracks a single robot location and checks whether it stepped on some threat.
#  Disables the robot (if SHOULD_KILL is True) with probability defined in the threat map for that threat and reports about it on a dedicated topic.
#  Will do nothing, if the robot occupies the same grid cell (either because it stopped, or just didn't make it to the next cell yet).
#
#  Allows to disable/enable class members via dynamic_reconfigure
# (e.g. enable/disable nodes shutdown from another node, or directly from the user).
class Tracker(object):
	SHOULD_KILL = True
	NORM_FACTOR = 100.0
	REPORT_TOPIC_NAME = "adversarial_trigger_report"
	ROBOT_CAUGHT_TOPIC_NAME = "robot_caught_in_threat"  # appears in Stopper Package


	def __init__(self, robot_ns, topic_name, threat_map):
		self.robot_ns = robot_ns
		self.threat_map = threat_map
		self.currently_occupied_cell = None
		self.reporter = rospy.Publisher(self.REPORT_TOPIC_NAME, TriggerReport, queue_size=10)
		self.robot_caught_pub = rospy.Publisher(robot_ns + "/" + self.ROBOT_CAUGHT_TOPIC_NAME, Bool, queue_size=10)
		self.robot_path = []
		print "-----------------------------------------------"
		print "Topic pose to subscribe is: " + topic_name
		print "-----------------------------------------------"
		self.pose_listener = rospy.Subscriber(topic_name, Odometry, self.tracker_callback)


	def __del__(self):
		# TODO: needed?
		self.reporter.unregister()
		self.pose_listener.unregister()


	## Kills all nodes under self.robot_ns
	#  e.g. assuming robot_ns is /robot_0, will kill all /robot_0/* nodes
	# TODO: add logic to kill -9 nodes if they didn't shutdown properly, instead of a warning.
	def kill_all_ns_nodes(self):
		names = rosnode.get_node_names(self.robot_ns)
		if len(names) == 0:
			# Something probably went wrong - warn the user.
			rospy.logwarn("Couldn't find any nodes under %s, while shutting down nodes." % self.robot_ns)
			return
		# Notify that you're about to shutdown each node,
		# to indicate that the next user initiated shutdown of a node with that name came from here.
		# Following this, a [ WARN ] message should appear, saying:
		# [ WARN] [secs, nsecs]: Shutdown request received.
		# [ WARN] [secs, nsecs]: Reason given for shutdown: [user request]
		for node_name in names:
			rospy.loginfo("Sending shutdown command to node '%s'" % node_name)
			success, fail = rosnode.kill_nodes([node_name])
			if len(fail) > 0:
				rospy.logwarn("Failed to shutdown node '%s'" % node_name)


	## Generates a Boolean message to indicate that this robot was caught in a threat.
	def publish_caught_msg(self):
		self.robot_caught_pub.publish(True)


	## Throws a biased coin, reports on a dedicated topic (if the coin throw was 'heads')
	#  Calls the kill method if self.SHOULD_KILL is True
	#  @param threat_prob - the probability of the biased coin.
	def take_action(self, threat_prob, cell):
		if threat_prob > 0 and random.random() <= threat_prob:  # checks (threat_prob > 0) to avoid extra random() calls
			self.publish_report(threat_prob, cell)
			if self.SHOULD_KILL:
				self.pose_listener.unregister()
				self.publish_caught_msg()
				# print the path of the robot to a text file.
				path_writer = PathWriter(self.robot_ns, self.robot_path)
				path_writer.write_path()
				#TODO - RETURN IT IF NEEDED : self.kill_all_ns_nodes()


	## Generates a TriggerReport() msg, fills the relevant data and publishes.
	def publish_report(self, threat_prob, cell):
		report = TriggerReport()  # std header, robot_ns, threat_prob.
		report.header.stamp = rospy.Time.now()
		report.robot_ns = self.robot_ns
		report.threat_probability = threat_prob
		report.caught_location = cell
		self.reporter.publish(report)

		


	## Finds the robot's cell in the grid from cartesian coordinates, checks the probability of that cell
	#  and calls the take_action() method (unless it's the same cell as last time).
	#
	#  Note: Currently, the only grid cell triggered is the cell that contains the center of the robot.
	#        TODO: Find all grid cells occupied by that robot and repeat the triggering process for each such cell.
	def tracker_callback(self, odom_message):
		rospy.loginfo("*** On tracker_callback ***")
		print "self.NORM_FACTOR is: " + str(self.NORM_FACTOR)
		print "odom_message.pose.pose.position is: " + str(odom_message.pose.pose.position)
		grid_col, grid_row = self.indices_from_position(odom_message.pose.pose.position)
		print ("(real_grid_col, real_grid_row) = " + str((grid_col, grid_row)))
		occupied_cell = ((grid_row - 1) * self.threat_map.info.width) + grid_col - 1  # -1 because the index starts from 0.
		if occupied_cell >= 0 and not occupied_cell == self.currently_occupied_cell:
			print "occupied_cell index is: " + str(occupied_cell)
	
			value = self.threat_map.data[occupied_cell]
			probability = value / self.NORM_FACTOR
			rospy.logdebug("Robot %s is in grid cell (%d,%d) with threat probability %f (%d in the msg)." % (self.robot_ns, grid_row, grid_col, probability, value))
			self.take_action(probability, str((grid_row, grid_col)))
			self.currently_occupied_cell = occupied_cell

	## Converts the x,y coordinates from a given geometry_msgs::Point argument
	#  into an i,j indices of the threat grid.
	#  @returns the
	def indices_from_position(self, position):
		ret = (-1, -1)
		grid_resolution = self.threat_map.info.resolution
		print "position.x is: " + str(position.x)
		print "grid_resolution is: " + str(grid_resolution)
		print "self.threat_map.info.width is: " + str(self.threat_map.info.width)

		# position can be negative, therefore adding a constant to the position.
		grid_col = int(position.x * Definitions.col_position_factor + Definitions.col_b)
		grid_row = int(position.y * Definitions.row_position_factor + Definitions.row_b)
		if 0 <= grid_row <= self.threat_map.info.height and 0 <= grid_col <= self.threat_map.info.width:
			ret = (grid_col, grid_row)
			self.robot_path.append(ret)
		else:
			rospy.logwarn("Robot position (%f,%f) is out of bound of a threat grid. Threat grid size is %dx%d and resolution is set to %f" %
						  (position.x, position.y, self.threat_map.info.width, self.threat_map.info.height, self.threat_map.info.resolution))
		return ret


def dynamic_reconfigure_callback(config, _):
	config_copy_for_log = config.copy()
	del(config_copy_for_log["groups"])
	rospy.loginfo("Dynamically reconfiguring parameters, new values are: %s" % config_copy_for_log)
	Tracker.SHOULD_KILL = config["shutdown_node_on_trigger"]
	Tracker.NORM_FACTOR = config["probability_normalization_factor"]
	print "Tracker.SHOULD_KILL is: " + str(Tracker.SHOULD_KILL)
	print "Tracker.NORM_FACTOR is: " + str(Tracker.NORM_FACTOR)
	return config

if __name__ == "__main__":
	rospy.init_node("monitor", anonymous=True)
	server = dynamic_reconfigure.server.Server(TrackerConfig, dynamic_reconfigure_callback)
	# get the robots yaml file and load it to dict
	yaml_file = sys.argv[1]

	if yaml_file is not None and os.path.exists(yaml_file):
		rospy.logdebug("Opening yaml file %s" % yaml_file)
		with open(yaml_file) as fd:
			robots_as_dict = yaml.safe_load(fd)
	else:
		sys.exit("Expected a yaml file as argument")
	Monitor(robots_as_dict)
	rospy.spin()

