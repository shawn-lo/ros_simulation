import pcl
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
#from visualization_msgs.msg import 
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import PyKDL
from math import radians, copysign, sqrt, pow, pi
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Odometry():
	def __init__(self):
		rospy.init_node('odometry', anonymous=False)
		rospy.on_shutdown(self.shutdown)
                self.index1 = 1
                self.index2 = 1
                self.bridge = CvBridge()
                self.depth_info = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback)
                print('start')
                self.point_cloud = rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback2)
                print('end')
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
		start_point = (sPos, 0)#0
                self.origin = start_point
		final_point = (fPos, 2*pi)#2*pi

                # Waypoints
                waypoints = []
                waypoints.append(start_point)

		pos1 = Point()
		pos1.x = -1
		pos1.y = 1
		p1 = (pos1, -pi)
                waypoints.append(p1)
		# self.moveTo(start_point, p1)

		pos2 = Point()
		pos2.x = -1
		pos2.y = -1
		p2 = (pos2, -3*pi/2)
		# self.moveTo(p1, p2)
                waypoints.append(p2)

		# self.moveTo(p2, final_point)
                waypoints.append(final_point)
                fake_flag = True
                for i in range(len(waypoints)-1):
                    print(waypoints[i][0].x)
                    from_point = waypoints[i]
                    to_point = waypoints[i+1]
                    print('from: ', from_point)
                    print('to: ', to_point)
                    temp_point = self.moveTo(from_point, to_point)
                    if temp_point != None and fake_flag:
                        rospy.sleep(2)
                        fake_flag = False
                        waypoints.insert(i+1, temp_point)

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
	    temp=self.goForward(start, final)
            return temp

        def get_current_coordinates(self, origin):
            pos = self.get_odom()
            trans = pos[0]
            rot = pos[1]
            curX = origin[0].x + trans.x
            curY = origin[0].y + trans.y
            curRot = origin[1] + rot
            curPos = Point()
            curPos.x = curX
            curPos.y = curY
            res = (curPos, curRot)
            return res


	def goForward(self, start, final, linear_speed=0.2, angular_speed=1.0):
		#print(self.get_odom())
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
			#print(rotation)
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
                #print(start_pos)
                stop_pos = start_pos
                get_stucked = False
		while pass_distance < goal_distance and not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			status = self.get_odom()
                        print(status)
			trans = status[0]
			pass_distance = sqrt(pow(trans.x-start_pos[0].x,2)+pow(trans.y-start_pos[0].y,2))
			#print(pass_distance)	
                        # check if get obstacle, here use a fake one to test
                        cc = self.get_current_coordinates(self.origin)
                        if cc[0].x < 0 and cc[0].x > -0.1:
                            print('Get position!')
                            stop_pos = cc
                            get_stucked = True
                            #print(stop_pos)
                            break
		print("Stop moving forward.")
		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)
                if get_stucked:
                    return stop_pos
                else:
                    return None
                #print(self.get_current_coordinates(self.origin))

        def callback(self, data):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data)
                depth_arr = np.array(cv_image, dtype=np.float32)
                #np.savetxt('../../txt/%d.txt'%self.index1, depth_arr)
                cv2.normalize(depth_arr, depth_arr, 0, 1, cv2.NORM_MINMAX)
                #cv2.imwrite('../../depth_map/DEFAULT_F32/%d.png'%self.index1, depth_arr*255)
                #data.encoding="mono8"
                #cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
            except CvBridgeError as e:
                print(e)
            #cv2.imwrite('./depth_map/%d.png'%self.index1, cv_image)
            self.index1 += 1
        
        def callback2(self, data):
            #data.to_file('./test.pcd')
            pc = pc2.read_points(data, skip_nans=True, field_names=("x","y","z"))
            pc_list = []
            for p in pc:
                pc_list.append([p[0], p[1], p[2]])
            p = pcl.PointCloud()
            p.from_list(pc_list)
            #p.to_file('../%d.pcd'%(self.index2))
            #if self.index2%10 == 0:
            #    p.to_file('../%d.pcd'%(self.index2//10))
            self.index2 += 1


	def shutdown(self):
		rospy.loginfo('Stop the robot...')
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)


if __name__ == '__main__':
	Odometry()
