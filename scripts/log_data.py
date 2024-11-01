#!/usr/bin/env python
import rospy
import numpy as np
import csv
import threading
from queue import Queue
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# log data of imu and odometry
class LogData:
    def __init__(self, log_path='./'):
        self.imu_data = Queue()
        self.odom_data = Queue()
        self.imu_sub = rospy.Subscriber("/livox/imu", Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        self.imu_file = open(log_path + 'imu_data.csv', 'w')
        self.odom_file = open(log_path + 'odom_data.csv', 'w')
        self.imu_writer = csv.writer(self.imu_file, delimiter=' ')
        self.odom_writer = csv.writer(self.odom_file, delimiter=' ')
        self.imu_writer.writerow(['time', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'])
        self.odom_writer.writerow(['time', 'position_x', 'position_y', 'position_z'])


    def imu_callback(self, msg):
        self.imu_data.put([msg.header.stamp.to_sec(), msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    
    def odom_callback(self, msg):
        self.odom_data.put([msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    # add a thread function to write data to file
    def save_data(self):
        while not rospy.is_shutdown():
            if not self.imu_data.empty():
                self.imu_writer.writerow(self.imu_data.get())
            if not self.odom_data.empty():
                self.odom_writer.writerow(self.odom_data.get())
    
    def run(self):
        thread = threading.Thread(target=self.save_data)
        thread.start()

if __name__ == '__main__':
    rospy.init_node('log_data')
    log_data = LogData()
    rospy.sleep(1)
    log_data.run()
    rospy.spin()