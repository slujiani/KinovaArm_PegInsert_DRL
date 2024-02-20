import sys
sys.path.append('/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it')
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from math import pi
from control_msgs.msg import *
from trajectory_msgs.msg import *
import actionlib
from std_srvs.srv import Empty
from tf import TransformListener
from robot import Robot
from task import peg_out
from task import peg_in
from task import pick_place
from task import screw
from task import screw_out
from Gen3Env.gen3env import gen3env
# from gen3env_zuo import gen3env
# from module import TD3,train
import gym
import numpy as np
import csv
import select
import threading
import time
# example_move_it_trajectories_run_pegin_grasp_screw


if __name__ == '__main__': 
  
  # env=gym.make(id='peg_in_hole-v0', peg_position = peg_pos, hole_position = hole_pos)
  # env=gym.make(id='peg_in_hole-v0')
  env = gen3env()

  # print('----------------')
  # print(env.get_action())


  # #peg in & out
  peg_pos = [0.31274772,0.01486048,0.01796503]  #[0.31274772 0.01486048 0.02207625 0.00698692]   [0.3137,0.0207,0.03915]
  hole_pos = [0.49814716,-0.14806058,0.12354184]  #插进去的位置 [ 0.53476997 -0.14936443  0.12828383  0.33537119]  插头在插板前面的位置[ 0.49814716 -0.14806058  0.12354184  0.33187775]
  # peg_in(robot=env.robot,peg_position=peg_pos,hole_position=hole_pos)
  peg_out(robot=env.robot,peg_position=peg_pos,hole_position=hole_pos)


  # #grasp
  # pick_pos=[0.5766, 0.0013208, 0.43364822]
  # place_pose=[0.32, 0.0013208, 0.43364822]
  # pick_place(robot=env.robot,pick_pose=pick_pos,place_pose=place_pose)



  # #screw
  # nut_pos=[0.40719028, -0.05425203, 0.02306067 ]  #[ 0.40719028 -0.05425203  0.02323915  0.51228071] [0.5000311  0.08270526 0.15489137 0.59298249] [0.31421369, 0.00457687, 0.02306067] 
  # tar_pos=[0.55560891, 0.12688822  , 0.20364476]  #[0.55261421 0.12603921 0.21349069 0.59298249] [0.55699549 0.129932   0.2064488  0.59298249] 拧螺丝的杆的脑袋的位置
  # screw(robot=env.robot,nut_pose=nut_pos,target_pose=tar_pos)
  # screw_out(robot=env.robot,nut_pose=nut_pos,target_pose=tar_pos)


