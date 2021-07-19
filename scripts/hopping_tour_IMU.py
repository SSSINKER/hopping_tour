#!/usr/bin/env python
import rospy
import math
from geopy.distance import lonlat, distance
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion
import time

a = 0.8 # factor for stability
MAX_DIST = 2 * a
waypoints = [[126.950599, 37.455848], [126.45555, 37.3434343], [126.54444, 37.34343444], [126.5455555, 37.43434343]]
# (lon, lat) = (x, y)
# building 37 rooftop

yaw = 0 # 0 at east, increase CCW
lo = 0 # longitude
la = 0 # latitude


def addAngle(angle1, angle2):
	result = angle1 + angle2
	if (result > 180):
		result -= 360
	elif (result < -180):
		result += 360
	return result

def getGPS(gps_data):
	global lo, la
	lo = gps_data.longitude
	la = gps_data.latitude
    
def getIMU(imu_data):
	global yaw
	orientation = imu_data.orientation
	quat = (orientation.x, orientation.y, orientation.z, orientation.w)
	euler = euler_from_quaternion(quat)
	yaw = -1 * euler[2] / math.pi * 180 # yaw in degree

def getOdom(odom_data):
	global yaw
	orientation = odom_data.pose.pose.orientation
	quat = (orientation.x, orientation.y, orientation.z, orientation.w)
	euler = euler_from_quaternion(quat)
	yaw = -1 * euler[2] / math.pi * 180 # yaw in degree


# have to know max/min cmd_vel value
def cmd_vel_mapping(dist, max_dist, target_angle, min_target_angle):
	vx = 0
	wz = 0
	if (dist > max_dist):
		vx = 0.5
	if (dist <= max_dist):
		vx = 0.5 / max_dist * dist
	
	if (90 < target_angle < 180):
		wz = -0.5
	elif (min_target_angle < target_angle < 90):
		wz = -0.5 / (90 - min_target_angle) * (target_angle - min_target_angle)
	elif (abs(target_angle) < min_target_angle):
		wz = 0
	elif (-90 < target_angle < -min_target_angle):
		wz = -0.5 / (90 - min_target_angle) * (target_angle + min_target_angle)
	elif (-180 < target_angle < 180):
		wz = 0.5
	
	return vx, wz
	

def hopping_tour(waypoints):
	global lo, la, yaw
	n_waypoint = len(waypoints)
	wp_idx = 0
	close_cnt = 0
	# min_target_angle = 180
	min_target_angle = 10

	rospy.init_node('hopping_tour', anonymous=True)
	rospy.Subscriber("fix", NavSatFix, getGPS)
	rospy.Subscriber("imu_data", Imu , getIMU)
	#rospy.Subscriber("odom", Odometry , getOdom)
	twist_pub = rospy.Publisher("cmd_vel", Twist)
	rate = rospy.Rate(10) # 10hz

	print("system preparing...")
	
	time.sleep(1)
	print("system starts!")

	while not rospy.is_shutdown():
		
		waypoint = waypoints[wp_idx]
		lo_diff = waypoint[0] - lo
		la_diff = waypoint[1] - la

		angle = math.acos(la_diff / math.sqrt(la_diff*la_diff + lo_diff*lo_diff)) / math.pi * 180 # degree

		if (lo_diff < 0):
			angle = -1 * angle

		# angle = math.atan(la_diff / lo_diff) / math.pi * 180

		target_angle = addAngle(yaw, -angle)
		print(yaw)
		print(angle)

		dist = distance(lonlat(lo, la), lonlat(*waypoint)).m
		# temp = math.asin(MAX_DIST/dist) / math.pi * 180
		# if (temp < min_target_angle):
		# 	 min_target_angle = temp
		
		twist = Twist()
		lin_vel, ang_vel = cmd_vel_mapping(dist, MAX_DIST * 1.2, target_angle, min_target_angle)
		# MAX_DIST multiply factor needs to be adjusted manually
		twist.linear = Vector3(lin_vel, 0, 0)
		twist.angular = Vector3(0, ang_vel, 0)
		twist_pub.publish(twist)

		if (wp_idx < n_waypoint):
			if (dist < MAX_DIST):
				close_cnt += 1
				print("reached at a waypoint. move to next waypoint")
		# else:
			#stop ship

		if (close_cnt >= 5):
			close_cnt = 0
			if (wp_idx < n_waypoint - 1):
				wp_idx += 1

		rate.sleep()

if __name__ == '__main__':
    try:
        hopping_tour(waypoints)
    except rospy.ROSInterruptException:
        pass