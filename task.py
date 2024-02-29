import sys
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
import time

#判断坐标是否在运动范围内
def pos_is_ok(pos = [0,0,0]):
   ret = True
   if pos[0] < 0 or pos[0] > 0.9:
      ret = False
   if ret == True and (pos[1] < -0.3 or pos[1] > 0.3):
      ret = False
   if ret == True and (pos[2] < 0 or pos[2] > 1):
      ret = False
   return ret
      
#机械臂开机动画
def open_actions(robot):
   rospy.loginfo('the start of following tasks.')
   success=True
   success &= robot.reach_named_position("home")
   success &= robot.reach_gripper_position(0)
   success &= robot.reach_gripper_position(0.8)
   success &= robot.reach_gripper_position(0)
   success &= robot.reach_gripper_position(0.8)
   success &= robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
   rospy.loginfo(str(success))
   return success
#抓不锈钢杯子
def pick_place(robot,pick_pose=[0,0,0],place_pose=[0,0,0],tolerance=0.0001,joint_rota=0,success=True):
    if pos_is_ok(pick_pose) == False:
       rospy.loginfo('pick place is out of range.')
       return False
    if pos_is_ok(place_pose) == False:
       rospy.loginfo('place place is out of range.')
       return False
    rospy.loginfo('Execute pick and place task...')
    rospy.loginfo('Go back to initial pose')
    success &= robot.reach_gripper_position(0)
    success &= robot.reach_named_position("home")
   # #  success&=robot.reach_joint_angles(j5=-pi/36)
   #  print(success)

    success &= robot.move(pose=[pick_pose[0],pick_pose[1],pick_pose[2]+0.2],tolerance=0.001)
    success &= robot.move(pose=[pick_pose[0],pick_pose[1],pick_pose[2]],tolerance=tolerance)
   #  success&=robot.reach_joint_angles(j6=joint_rota)
    
    rospy.loginfo('Arrived object pose, prepare for grabing...')
   #  print(success)
    success&=robot.reach_gripper_position(0.5)
   #  rospy.loginfo('Go to target pose')
    success &= robot.move(pose=[pick_pose[0],pick_pose[1],pick_pose[2]+0.1],tolerance=tolerance)
   #  success &= robot.move(pose=[0,0,0.2])
    success &= robot.move(pose=[place_pose[0],place_pose[1],place_pose[2]+0.1],tolerance=0.01)
    success &= robot.move(pose=[place_pose[0],place_pose[1],place_pose[2]],tolerance=tolerance)
   # #  success &= robot.move(pose=[0,0,-0.2])
    success &= robot.reach_gripper_position(0)
    print(success)
    rospy.loginfo('Task finished, back to home')
    success &= robot.reach_named_position("home")
    print(success)
    return success
#把螺母拧上
def screw(robot,nut_pose,target_pose,steer_angle=[0.0,0.0,0.0],tolerance=0.0001,success=True):
   if pos_is_ok(nut_pose) == False:
       rospy.loginfo('nut position is out of range.')
       return False
   if pos_is_ok(target_pose) == False:
       rospy.loginfo('target position is out of range.')
       return False

   rospy.loginfo('Execute screw task...')
   rospy.loginfo('Go back to initial pose')
   #复位
   torle_less=0.001
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
   success&=robot.reach_joint_angles(j5=-pi/36)
   print(success)
   #旋转
   success&=robot.reach_joint_angles(j6=steer_angle[1])
   #到螺母位置
   success &= robot.move(pose=[nut_pose[0],nut_pose[1],nut_pose[2]+0.1],tolerance=0.01)
   success &= robot.move(pose=[nut_pose[0],nut_pose[1],nut_pose[2]],tolerance=tolerance)
   rospy.loginfo('Arrive object pose, prepare for grabing nut...')
   print(success)
   #抓螺母
   success&=robot.reach_gripper_position(0.8)
   rospy.loginfo('Go to target pose')
   success &= robot.move(pose=[nut_pose[0],nut_pose[1],nut_pose[2]+0.1],tolerance=0.01)
   success &= robot.move(pose=[target_pose[0],target_pose[1],target_pose[2]+0.1],tolerance=0.01)
   success &= robot.move(pose=[target_pose[0],target_pose[1],target_pose[2]],tolerance=tolerance)
   success&=robot.move(pose=[0,0,-0.045],tolerance=tolerance)
   rospy.loginfo('Screwing')
   for _ in range(8):
      success&=robot.reach_joint_angles(j6=-pi/2)
   print(success)
   rospy.loginfo('Task finished, back to home')
   success &= robot.reach_gripper_position(0) 
   success &= robot.reach_named_position("retract")
   return success
#把螺母拧出来
def screw_out(robot,nut_pose,target_pose,tolerance=0.0001,success=True):
   if pos_is_ok(nut_pose) == False:
       rospy.loginfo('nut position is out of range.')
       return False
   if pos_is_ok(target_pose) == False:
       rospy.loginfo('target position is out of range.')
       return False
   rospy.loginfo('Execute screw task...')
   rospy.loginfo('Go back to initial pose')
   success &= robot.reach_named_position("retract")
   success&=robot.reach_gripper_position(0)
   success&=robot.reach_joint_angles(j5=-pi/36)
   print(success)
   # success &= robot.move(pose=[target_pose[0],target_pose[1],target_pose[2]+0.05],tolerance=tolerance)
   success &= robot.move(pose=[target_pose[0],target_pose[1],target_pose[2]],tolerance=tolerance)
   success&=robot.move(pose=[0,0,-0.045],tolerance=tolerance)
   rospy.loginfo('Arrive object pose, prepare for grabing nut...')
   print(success)
   success&=robot.reach_gripper_position(0.8)
   rospy.loginfo('Screwing out')
   for _ in range(8):
      success&=robot.reach_joint_angles(j6=pi/2)
   print(success)

   rospy.loginfo('Go to target pose')
   success &= robot.move(pose=[target_pose[0],target_pose[1],target_pose[2]+0.1],tolerance=tolerance)
   success &= robot.move(pose=[nut_pose[0],nut_pose[1],nut_pose[2]],tolerance=tolerance)
   
   rospy.loginfo('Task finished, back to home')
   success &= robot.reach_gripper_position(0) 
   success &= robot.reach_named_position("home")
   return success
 #插头插进插板，停留，然后拔出复位
def peg_in_and_out(robot,peg_position,hole_position,steer_angle=[0.0,0.0,0.0],tolerance=0.0001,success=True):
   if pos_is_ok(peg_position) == False:
       rospy.loginfo('peg position is out of range.')
       return False
   if pos_is_ok(hole_position) == False:
       rospy.loginfo('hole position is out of range.')
       return False
   rospy.loginfo('Execute peg in hole task...')
   rospy.loginfo('Go back to initial pose')
   torle_less=0.001
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
  #  print(joint_positions)geometry_msgs.msg.
   # success&=robot.reach_named_position('retract')
   success&=robot.reach_joint_angles(j5=-pi/36)
   print(success)
   #旋转角度抓插头
   success&=robot.reach_joint_angles(j6=steer_angle[1])
    
   #移动到插头位置
   rospy.loginfo('Arriving at peg position, perparing for grabing peg...')
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]], tolerance=torle_less)
   #抓
   success&=robot.reach_gripper_position(0.465)#465
   # time.sleep(1)
   print('moving to hole position')
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]+0.1], tolerance=torle_less)
   rospy.loginfo('Start to peg in...')


   success&=robot.move(pose=[hole_position[0],peg_position[1],peg_position[2]+0.1], tolerance=torle_less)
   success&=robot.reach_joint_angles(j6=pi/2)
   #移动到插座位置
   success&=robot.move(pose=[hole_position[0],hole_position[1],hole_position[2]], tolerance=torle_less)
   x = hole_position[1]+0.0481
   # time.sleep(1)
   #插入
   success&=robot.move(pose=[hole_position[0],x,hole_position[2]], tolerance=tolerance)
   #拔出
   x = hole_position[1]-0.0481
   success&=robot.move(pose=[hole_position[0],x,hole_position[2]], tolerance=tolerance)
   success&=robot.reach_joint_angles(j6=-pi/2)
   #中间点位
   success&=robot.move(pose=[(hole_position[0]+peg_position[0])/2,peg_position[1],peg_position[2]+0.1], tolerance=torle_less)
   #下放插头
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]+0.1], tolerance=torle_less)
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]], tolerance=torle_less)
   #松爪复位
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
   print(success)
   return success

def peg_in(robot,peg_position,hole_position,steer_angle=0.0,tolerance=0.0001,success=True):
   if pos_is_ok(peg_position) == False:
       rospy.loginfo('peg position is out of range.')
       return False
   if pos_is_ok(hole_position) == False:
       rospy.loginfo('hole position is out of range.')
       return False
   
   rospy.loginfo('Execute peg in hole task...')
   rospy.loginfo('Go back to initial pose')
   torle_less=0.001
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
  #  print(joint_positions)geometry_msgs.msg.
   # success&=robot.reach_named_position('retract')
   success&=robot.reach_joint_angles(j5=-pi/36)
   print(success)
   #旋转角度抓插头
   success&=robot.reach_joint_angles(j6=steer_angle)
    
   #移动到插头位置
   rospy.loginfo('Arriving at peg position, perparing for grabing peg...')
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]], tolerance=torle_less)
   #抓
   success&=robot.reach_gripper_position(0.465)#465
   # time.sleep(1)
   print('moving to hole position')
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]+0.1], tolerance=torle_less)
   rospy.loginfo('Start to peg in...')


   success&=robot.move(pose=[hole_position[0],peg_position[1],peg_position[2]+0.1], tolerance=torle_less)
   success&=robot.reach_joint_angles(j6=pi/2)
   #移动到插座位置
   success&=robot.move(pose=[hole_position[0],hole_position[1],hole_position[2]], tolerance=tolerance)
   x = hole_position[1]+0.0481
   # time.sleep(1)
   #插入
   success&=robot.move(pose=[hole_position[0],x,hole_position[2]], tolerance=tolerance)
   #松爪复位
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
   print(success)
   return success

def peg_in_restraint(robot,peg_position,hole_position,steer_angle=0.0,tolerance=0.0001,success=True):
   if pos_is_ok(peg_position) == False:
       rospy.loginfo('peg position is out of range.')
       return False
   if pos_is_ok(hole_position) == False:
       rospy.loginfo('hole position is out of range.')
       return False
   
   rospy.loginfo('Execute peg in hole task...')
   rospy.loginfo('Go back to initial pose')
   torle_less=0.001
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
  #  print(joint_positions)geometry_msgs.msg.
   # success&=robot.reach_named_position('retract')
   success&=robot.reach_joint_angles(j5=-pi/36)
   print(success)
   #旋转角度抓插头
   success&=robot.reach_joint_angles(j6=steer_angle)
    
   #移动到插头位置
   rospy.loginfo('Arriving at peg position, perparing for grabing peg...')
   success&=robot.mov_with_restraints(target_pos=[peg_position[0],peg_position[1],peg_position[2]])
   #抓
   success&=robot.reach_gripper_position(0.465)#465
   # time.sleep(1)
   print('moving to hole position')
   success&=robot.mov_with_restraints(target_pos=[peg_position[0],peg_position[1],peg_position[2]+0.1])
   rospy.loginfo('Start to peg in...')


   success&=robot.mov_with_restraints(target_pos=[hole_position[0],peg_position[1],peg_position[2]+0.1])
   success&=robot.reach_joint_angles(j6=pi/2)
   #移动到插座位置
   success&=robot.mov_with_restraints(target_pos=[hole_position[0],hole_position[1],hole_position[2]])
   x = hole_position[1]+0.0481
   # time.sleep(1)
   #插入
   success&=robot.mov_with_restraints(target_pos=[hole_position[0],x,hole_position[2]])
   #松爪复位
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
   print(success)
   return success

def peg_out(robot,peg_position,hole_position,tolerance=0.0001,success=True):
   if pos_is_ok(peg_position) == False:
       rospy.loginfo('peg position is out of range.')
       return False
   if pos_is_ok(hole_position) == False:
       rospy.loginfo('hole position is out of range.')
       return False
   
   rospy.loginfo('Execute peg out of hole task...')
   rospy.loginfo('Go back to initial pose')
   success&=robot.reach_gripper_position(0)
   success &= robot.reach_named_position("retract")
  #  print(joint_positions)geometry_msgs.msg.
   # success&=robot.reach_named_position('retract')
   success&=robot.reach_joint_angles(j5=-pi/36)
   print(success)
   
   success&=robot.move(pose=[hole_position[0],hole_position[1],hole_position[2]], tolerance=0.001)
   success&=robot.reach_joint_angles(j6=pi/2)
   x = hole_position[1]+0.0481
   # time.sleep(1)
   success&=robot.move(pose=[hole_position[0],x,hole_position[2]], tolerance=tolerance)
   rospy.loginfo('Start to peg out...')
   success&=robot.reach_gripper_position(0.465)#465
   x=x-0.0481
   success&=robot.move(pose=[hole_position[0],x,hole_position[2]], tolerance=tolerance)
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]+0.1], tolerance=0.001)
   success&=robot.reach_joint_angles(j6=-pi/2)
   success&=robot.move(pose=[peg_position[0],peg_position[1],peg_position[2]], tolerance=0.001)
   rospy.loginfo('Arriving at peg_position, ready to put peg down')
   success&=robot.reach_gripper_position(0)
   # time.sleep(1)
   print('task finished')
   success &= robot.reach_named_position("retract")
   #  success &= robot.reach_joint_angles(j5=-pi/36)
   
   #  success&=robot.reach_joint_angles(j5=pi/2)
   # time.sleep(0.5)
   # success&=robot.move(pose=[0.06,0,0],tolerance=tolerance)
   print(success)
   return success
