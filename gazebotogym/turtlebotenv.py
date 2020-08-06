import gym
import rospy
#import roslaunch
import sys
import os
import signal
import subprocess
import time
from std_srvs.srv import Empty
import random
import numpy as np 
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
from gazebotogym.gazebo_envs import GazeboEnv
import skimage as skimage
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer
class env(GazeboEnv):
    def __init__(self):
        GazeboEnv.__init__(self, "turtlebot_world.launch world_file:=/home/raj/turtlebot_custom_gazebo_worlds/env_v0.world")
        self.node=rospy.init_node('myenv',anonymous=True)
        self.velocity_publisher=rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
        self.vel_msg=Twist()
        self.img_topic="/camera/rgb/image_raw"
        self.subscriber=rospy.Subscriber(self.img_topic,Image,self.callback)
        self.depth_topic="/camera/depth/image_raw"
        self.subscriber=rospy.Subscriber(self.depth_topic,Image,self.callbackd)
        self.rate=rospy.Rate(10)
        self.time=10
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.action_space.n=3
        self.reward_range=(-np.inf,np.inf)
        self.cv_bridge=CvBridge()
        self.last50actions = [0] * 50
    
    def step(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
        if action==0: # Forward
            self.vel_msg.linear.x=0.2
            self.vel_msg.angular.z=0
            self.velocity_publisher(self.vel_msg)
        elif action==1: #Left
            self.vel_msg.linear.x=0.05
            self.vel_msg.angular.z=0.2
            self.velocity_publisher(self.vel_msg)
        elif action==2:#Right
            self.vel_msg.linear.x=0.05
            self.vel_msg.angular.z=-0.2
            self.velocity_publisher(self.vel_msg)


        # Done formulation
        """data=None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass
            image_data
            done = self.calculate_observation(data)

        image_data = None
        success=False
        cv_image = None
        while image_data is None or success is False:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=5)
                h = image_data.height
                w = image_data.width
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
                #temporal fix, check image is not corrupted
                if not (cv_image[h/2,w/2,0]==178 and cv_image[h/2,w/2,1]==178 and cv_image[h/2,w/2,2]==178):
                    success = True
                else:
                    pass
                    #print("/camera/rgb/image_raw ERROR, retrying")
            except:
                pass"""
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
             print ("/gazebo/pause_physics service call failed")

        self.last50actions.pop(0) #remove oldest
        self.last50actions.append(action)
    
        #Reward formulation.

        def reset(self):
            self.last50actions=[0]*50
            rospy.wait_for_service('/gazebo/reset_simulation')
            try:
                self.reset_proxy()
            except (rospy.ServiceException) as e:
                print ("/gazebo/reset_simulation service call failed")

            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                self.unpause()
            except (rospy.ServiceException) as e:
                print("/gazebo/unpause_physics service call failed")

            # Done,formulation and procure state  from it.  

            rospy.wait_for_service('/gazebo/pause_physics')
            try:
            #resp_pause = pause.call()
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed") 

            #return state 
        

        

        


        


