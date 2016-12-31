import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
import PyKDL
from math import radians, copysign, sqrt, pow, pi

class Odometry():
	def __init__(self):
		rospy.init_node('odometry', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
		rate = 10
		self.r = rospy.Rate(rate)
		self.tf_listener = tf.TransformListener()
		self.odom_frame = '/odom'
		self.base_frame = '/base_footprint'

		sPos = Point()
		fPos = Point()
		sPos.x = 1
		sPos.y = 1
		fPos.x = 1.5
		fPos.y = -1
		# Point should be ((x,y), rotation)
		start_point = (sPos, 0)
		final_point = (fPos, 2*pi)

		# Waypoints
		pos1 = Point()
		pos1.x = -1
		pos1.y = 1
		p1 = (pos1, pi)
		self.moveTo(start_point, p1)

		pos2 = Point()
		pos2.x = -1
		pos2.y = -1
		p2 = (pos2, 3*pi/2)
		self.moveTo(p1, p2)

		self.moveTo(p2, final_point)

	def quat_to_angle(self, quat):
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]

	def normalize_angle(self, angle):
		res = angle
		while res > pi:
			res -= 2.0*pi
		while res < -pi:
			res += 2.0*pi
		return res

	def get_odom(self):
		self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(4.0))
		(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
		return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

	def moveTo(self, start, final):
		self.goForward(start, final)

	def goForward(self, start, final, linear_speed=0.2, angular_speed=1.0):
		# Two parts, rotate first and then go straight
		move_cmd = Twist()
		move_cmd.angular.z = angular_speed 
		goal_angle = final[1] - start[1]
		turn_angle = 0
		angular_tolerance = radians(2.5)
		last_angle = start[1]
		print("Start rotating to forward direction.")
		while(abs(turn_angle+angular_tolerance) < abs(goal_angle)) and not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			status = self.get_odom()

			rotation = status[1]
			print(rotation)
			delta_angle = self.normalize_angle(rotation-last_angle)
			turn_angle += delta_angle
			last_angle = rotation
		
		# Stop roation and prepare to go forward
		print("Stop rotation.")
		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)

		# Go forward
		print("Start moving to target.")
		goal_distance = sqrt(pow(final[0].x-start[0].x,2)+pow(final[0].y-start[0].y,2))
		pass_distance = 0
		move_cmd = Twist()
		move_cmd.linear.x = linear_speed
		start_pos = self.get_odom()
		while pass_distance < goal_distance and not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			status = self.get_odom()

			trans = status[0]
			pass_distance = sqrt(pow(trans.x-start_pos[0].x,2)+pow(trans.y-start_pos[0].y,2))
			print(pass_distance)	
		print("Stop moving forward.")
		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)

	def shutdown(self):
		rospy.loginfo('Stop the robot...')
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':
	Odometry()
