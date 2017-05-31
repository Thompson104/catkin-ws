#! /usr/bin/env python
import actionlib
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import *
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from math import pi
import roslib; roslib.load_manifest('ur_driver')



JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def call_back(msg,twist_pub):
    global d_duration
    global g
    v1 = 0
    v2 = 0 
    v3 = 0
    if msg.axes[0] > 0.5:
        v1 = 0.05    
    if msg.axes[1] > 0.5:
        v2 = 0.05
    if msg.axes[2] > 0.5:
        v3 = 0.05
    if msg.axes[0] < -0.5:
        v1 = -0.05
    if msg.axes[1] < -0.5:
        v2 = -0.05
    if msg.axes[2] < -0.5:
        v3 = -0.05
    print('I hear',d_duration)


    g = JointTrajectory()
    g.joint_names =  JOINT_NAMES
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position
    '''
    g.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(d_duration))]
    '''
    

    joints_pos_new = joint_states.position
    joints_pos_new_list = list(joints_pos_new) 
    joints_pos_new_list[0] = joints_pos_new_list[0]+3
    joints_pos_new_list[1] = joints_pos_new_list[1]+3
    joints_pos_new_list[2] = joints_pos_new_list[2]+3
    joints_pos_new = tuple(joints_pos_new_list)
    v = [0]*6
    v[0] = 1*v1
    v[1] = 1*v2
    v[2] = 1*v3

    v_v = [0,0,0,0,0,0]
    v_v[0] = 2*v1
    v_v[1] = 2*v2
    v_v[2] = 2*v3

    g.points.append(
        #JointTrajectoryPoint(positions=joints_pos_new,velocities=v,accelerations = v_v,time_from_start=rospy.Duration(d_duration)))
        JointTrajectoryPoint(velocities=v,effort = v_v,time_from_start=rospy.Duration(d_duration)))
    d_duration+=0.02
    twist_pub.publish(g)

def main():
    global client
    global d_duration
    global g
    
    rospy.init_node('keys_to_twist_right')
    twist_pub = rospy.Publisher('/ur_driver/joint_speed',JointTrajectory,queue_size = 20)
    d_duration = 0.0
    rate = rospy.Rate(50)
    rospy.Subscriber('right_joy',Joy,call_back,twist_pub,queue_size = 20)
    while not rospy.is_shutdown():
        rate.sleep()
    '''
    rospy.Subscriber('left_joy',Joy,call_back,twist_pub)
    g = JointTrajectory()
    g.joint_names =  JOINT_NAMES
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position
    g.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(d_duration))]
    d_duration += 0.1
    g.points.append(
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(d_duration)))
    
    while not rospy.is_shutdown():
        twist_pub.publish(g)
        rate.sleep()
    '''

if __name__ == '__main__': main()