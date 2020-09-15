#import gym
import rospy
#import roslaunch
import math
import sys
import os
import signal
import subprocess
import time
from std_srvs.srv import Empty
import random
import numpy as np 
#from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
#from gym.utils import seeding
#import gazebo_envs
#import skimage as skimage
#from skimage import transform, color, exposure
#from skimage.transform import rotate
#from skimage.viewer import ImageViewer
from gazebo_msgs.srv import GetModelState,GetModelProperties
import rospy 
from gazebo_msgs.msg import ModelStates
class env(object):
    def __init__(self):
        super(env,self).__init__()
        self.node=rospy.init_node('myenv',anonymous=True)
        self.velocity_publisher=rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
        self.vel_msg=Twist()
        self.rate=rospy.Rate(10)
        self.time=10
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        #self.action_space.n=3
        self.reward_range=(-np.inf,np.inf)
        self.last50actions = [0] * 50
        #self.goal_point=(0.0628782315769264, -0.9846704140830553, -0.0011307433594164584)
        self.wallx=np.array([-4.39941000724,5.07647999276])
        self.wally=np.array([-4.9845548521,2.2904751479])
        self.wallz=np.array([-0.0012307433594164584,0])
        self.action_space=[-1.0,1.0]
        self.goal_point=np.array([0.077285840849,-1.03265836299,-0.00113072189145])
        #self.observation_space=self.Statedict(self.State())
        self.goal_point_orientation=np.array([-0.00283449091717,-0.00282021435669,-0.714220062084,0.699909790594])
        #self.sub=rospy.Subscriber('/cmd_vel_mux/input/navi',Twist,self.twist)

    """def GetModelPose(self):
        model_coordinates=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        object_coordinates=model_coordinates(self.modelname,"")
        z=object_coordinates.pose.position.z
        y=object_coordinates.pose.position.y
        x=object_coordinates.pose.position.x
        return x,y,z"""
    
    

    def State(self):
        message=rospy.wait_for_message('/gazebo/model_states',ModelStates)
        pose=message.pose[1]
        #messg=rospy.wait_for_message('cmd_vel_mux/input/navi',Twist)
        #stero=rospy.wait_for_message('sphero/imu/data3',Imu)
        return pose#,stero

    def Statedict(self,pose,action=np.array([np.random.choice(1),np.random.choice(1)])):#,action=[np.random.choice(1),np.random.choice(1)]):#:,stero):
        """state_dict={"current_position":{"position":{
            "x":pose.position.x,
            "y":pose.position.y,
            "z":pose.position.z   },
           "orientation":{
            "x":pose.orientation.x,
            "y":pose.orientation.y,
            "z":pose.orientation.z,
            "w":pose.orientation.w
                    }
            }
            ,"goal":{"position":{
                "x": 0.077285840849,
                "y": -1.03265836299,
                "z": -0.00113072189145
            },
            "orientation":{
                "x": -0.00283449091717,
                "y": -0.00282021435669,
                "z": -0.714220062084,
                "w": 0.699909790594
            }
               }
        }"""
        state=np.zeros(9)
        state[0]=pose.position.x
        state[1]=pose.position.y 
        state[2]=pose.position.z
        state[3]=pose.orientation.x
        state[4]=pose.orientation.y
        state[5]=pose.orientation.z
        state[6]=pose.orientation.w
        """state[7]=0.077285840849
        state[8]=-1.03265836299
        state[9]=-0.00113072189145
        state[10]=-0.00283449091717
        state[11]=-0.00282021435669
        state[12]=-0.714220062084
        state[13]=0.699909790594"""
        """state[7]=messg.linear.x
        state[8]=messg.linear.y
        state[9]=messg.linear.z
        state[10]=stero.linear_acceleration.x
        state[11]=stero.linear_acceleration.y
        state[12]=stero.linear_acceleration.z"""
        state[7]=action[0]
        state[8]=action[1]
        #state[8]=np.array([stero.linear_acceleration.x,stero.linear_acceleration.y,stero.linear_acceleration.z])
        return state

    def reward_function(self,state):
        #image_data = Output of gazebo image from the trained network
        """x=state["current_position"]["position"]["x"]
        y=state["current_position"]["position"]["y"]
        z=state["current_position"]["position"]["z"]
        goal_x=state["goal"]["position"]["x"]
        goal_y=state["goal"]["position"]["y"]
        goal_z=state["goal"]["position"]["z"]
        goal_orient_x=state["goal"]["orientation"]["x"]
        goal_orient_y=state["goal"]["orientation"]["y"]
        goal_orient_z=state["goal"]["orientation"]["z"]
        orient_x=state["current_position"]["orientation"]["x"]
        orient_y=state["current_position"][
        self.reward_range=(-np.inf,np.inf)
        #self.cv_bridge=CvBridge()
        self.last50actions = [0] * 50
        #self.goal_point=(0.0628782315769264, -0.9846704140830553, -0.0011307433594164584)
        self.wallx=(-4.39941000724,5.07647999276)
        self.wally=(-4.9845548521,2.2904751479)
        self.wallz=(-0.0012307433594164584,0)"orientation"]["y"]
        orient_z=state["current_position"][
        self.reward_range=(-np.inf,np.inf)
        #self.cv_bridge=CvBridge()
        self.last50actions = [0] * 50
        #self.goal_point=(0.0628782315769264, -0.9846704140830553, -0.0011307433594164584)
        self.wallx=(-4.39941000724,5.07647999276)
        self.wally=(-4.9845548521,2.2904751479)
        self.wallz=(-0.0012307433594164584,0)"orientation"]["z"]
        """
        x=state[0]
        y=state[1]
        z=state[2]
        orient_x=state[3]
        orient_y=state[4]
        orient_z=state[5]
        orient_w=state[6]
        """goal_x=state[7]
        goal_y=state[8]
        goal_z=state[9]
        goal_orient_x=state[10]
        goal_orient_y=state[11]
        goal_orient_z=state[12]
        goal_orient_w=state[13]"""
        goal_x=self.goal_point[0]
        goal_y=self.goal_point[1]
        goal_z=self.goal_point[2]
        goal_orient_x=self.goal_point_orientation[0]
        goal_orient_y=self.goal_point_orientation[1]
        goal_orient_z=self.goal_point_orientation[2]
        goal_orient_w=self.goal_point_orientation[3]
    
        dist=math.sqrt(math.pow((x-goal_x),2)+math.pow((y-goal_y),2)+math.pow((z-goal_z),2))
        dist2=math.sqrt(math.pow((orient_x-goal_orient_x),2)+math.pow((orient_y-goal_orient_y),2)+math.pow((orient_z-goal_orient_z),2))
        if self.wallx[0]<=x<=self.wallx[1] and self.wally[0]<=y<=self.wally[1] and self.wallz[0]<=z<=self.wallz[1]:
            if dist<=0.1:
                return  0 #and dist2<0.01: #and  if sum of image_data10000*25
            else:
                return -1
        else:
            return -1

    
    def done(self,state):
        x=state[0]
        y=state[1]
        z=state[2]
        if self.wallx[0]<=x<=self.wallx[1] and self.wally[0]<=y<=self.wally[1] and self.wallz[0]<=z<=self.wallz[1]:
            if self.reward_function(state)==0:
                return True
            else:
                return False
        else: 
            return True
        


    def _step(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed") # Forward
        self.vel_msg.linear.x=action[0]
        self.vel_msg.angular.z=action[1]
        self.velocity_publisher.publish(self.vel_msg)
        

        # Extracting the next state after the application of the action
        nextpose=self.State()
        nextstate=self.Statedict(nextpose,action)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
             print ("/gazebo/pause_physics service call failed")
        reward=self.reward_function(nextstate)
        done=self.done(nextstate)
        self.last50actions.pop(0) #remove oldest
        self.last50actions.append(action)
        return nextstate,reward,done
        #Reward formulation.

    def _reset(self):
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

        pose=self.State()
        state=self.Statedict(pose)
        goal_pose=self.State()
        goal_state=self.Statedict(goal_pose)#,stero)
        goal_state+=np.random.choice(1)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed") 

        return state,goal_state 
    
    def sample(self):
        action=[random.choice(self.action_space),random.choice(self.action_space)]
        return action
