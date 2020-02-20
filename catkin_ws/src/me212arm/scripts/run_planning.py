#!/usr/bin/python

# 2.12 Lab 3 trajectory planning
# Peter Yu Oct 2016
# Jerry Ng Feb 2020
import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
import numpy as np

rospy.init_node("run_planning")

q0 = [0,0]

def joint1_callback(msg):
    q0[0] = msg.data

def joint2_callback(msg):
    q0[1] = msg.data

exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint2_pub = rospy.Publisher('/joint2_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)
rospy.Subscriber('/joint1_controller/position', std_msgs.msg.Float64, joint1_callback)
rospy.Subscriber('/joint2_controller/position', std_msgs.msg.Float64, joint2_callback)
use_real_arm = True 
circle = False #EDIT THIS: Change this to True when generating a circular trajectory

if __name__=="__main__":
    radius = 0.2         # (meter)
    center = [1.2, 1.1]  # (x,z) meter
    rospy.sleep(1)
    rospy.wait_for_message('/joint1_controller/position', std_msgs.msg.Float64)
    rospy.wait_for_message('/joint2_controller/position', std_msgs.msg.Float64)
    if circle:
        for theta in np.linspace(0, 4*np.pi,500):
            target_xz =   []# [??, ??] EDIT THIS: use theta, radius, and center in your code
            q_sol =  []# planner.ik( ?? ) EDIT THIS: You should be using the planner.ik function
            if q_sol is None:
                print 'no ik solution'
                rospy.sleep(0.01)
            else:
                print '(q_1,q_2)=', q_sol
                if use_real_arm:
                    exec_joint1_pub.publish(std_msgs.msg.Float64(-q_sol[0]))
                    exec_joint2_pub.publish(std_msgs.msg.Float64(-q_sol[1]))
                q0 = q_sol

            rospy.sleep(0.01)
    else: 
        target_xz =  center
        q_sol = planner.ik(target_xz, q0)
        exec_joint1_pub.publish(std_msgs.msg.Float64(-q_sol[0]))
        exec_joint2_pub.publish(std_msgs.msg.Float64(-q_sol[1]))