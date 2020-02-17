#!/usr/bin/python

# 2.12 Lab 3 trajectory planning
# Jerry Ng Feb 2020

from OdriveClass import Odrive

odrivestring = '2087377B3548'
if __name__ == '__main__':
	myodrive = Odrive(odrivestring)
	myodrive.full_init()