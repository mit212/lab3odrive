#!/usr/bin/python

# 2.12 Lab 3 trajectory planning
# Peter Yu Oct 2016
# Jerry Ng Feb 2020
import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
import numpy as np
import csv


if __name__=="__main__":
    with open('correct_values.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter = ',') 
        correct_list = []
        for row in reader:
            correct_list.append([float(row[0]),float(row[1])])
        end_point_list = [[1.1, 1.2],[1.1, 1.2],[1,1],[1.2,1],[1.3,1.1],[1.15,1]]
        q0 = [0,0]
        valid_ik = True
        epsilon = 0.0001
        for i in range(0,len(end_point_list)):
            print(correct_list[i])
            target_xz =  end_point_list[i]
            q_sol = planner.ik(target_xz, q0)
            if abs(q_sol - correct_list[i]) < epsilon:
                valid_ik = valid_ik
            else:
                valid_ik = False

        if(valid_ik):
            print('You have passed the function checker!')
        else:
            print('Your IK did not pass the function checker. Please check your implementation.')