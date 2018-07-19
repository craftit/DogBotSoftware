#!/usr/bin/env python
# 
# Author: Nic Greenway
# Copyright (c) 2016 React AI Ltd
# All rights reserved. 

# Sets the dogbot's initial pose in Gazebo


import rospy
import time
import sys, getopt
from std_msgs.msg import String, Header 
from gazebo_msgs.srv import SetModelConfiguration
from std_srvs.srv import Empty

def setInitialJointPositions(robotname, joints, positions):
    
    rospy.wait_for_service('/gazebo/set_model_configuration')
    rospy.wait_for_service('/gazebo/unpause_physics')
    
    setJoints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    
    #TODO - put a proper check in on model setup completion
    time.sleep(2)
    
    #TODO - pick up joint positions from configuration
    if len(joints)==0:
      joints = ["l_ur5_elbow_joint", "l_ur5_shoulder_lift_joint", "l_ur5_shoulder_pan_joint", "l_ur5_wrist_1_joint", "r_ur5_elbow_joint", "r_ur5_shoulder_lift_joint", "r_ur5_shoulder_pan_joint", "r_ur5_wrist_1_joint"]
    
    if len(positions)==0:
      positions = [-1.659, -1.7296, -0.8471, -1.6943, 1.659, -1.377, 0.8471, -1.6943]
    
    resp = setJoints(robotname, "robot_description", joints, positions)
    if resp:
      print "Set robot initial joint positions"
    else:
      print "Failed setting initial joint positions"
    
    #and unpause
    unPause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    if unPause():
      print "Un-paused Gazebo"
    else:
      print "Failed to un-pause Gazebo"


def usage():
    print('tb_initgazebo -r robotname -j joints -p positions')

def main(argv):
    # here we must acquire the ns from the cmd line, so that we can 
    # ensure that we use the right frame for the tf of the path msgs
    ns=''
    joints=[]
    positions=[]
    
    try:
        opts, args = getopt.getopt(argv, "hjp:", ["help", "joints=", "positions="])
    except getopt.GetoptError:
        usage()
        exit.sys()  

    for o, a in opts:
        if o == "help":
            usage()
            exit.sys()
        elif o in ("-j","-joints"):
            joints=a
        elif o in ("-j","-positions"):
            positions=a
        else:
            usage()
            exit.sys()

    setInitialJointPositions("robot", joints, positions)
    #TODO - add objects to scene if required


if __name__ == '__main__':
    try:
      main(sys.argv[1:])
    except rospy.ROSInterruptException:
      pass
      