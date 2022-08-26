#!/usr/bin/env python3

#--------------------------------- Librerias ------------------------------------------

import rospy
import numpy as np
import time
import tf2_ros
import tf
import math
import smach
import ros_numpy

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
from geometry_msgs.msg import PoseStamped,Pose
from utils_evasion import *
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist, Point

#------------------------------ Inicializando Nodo ------------------------------------

rospy.init_node("nodo1")
EQUIPO = "UPS Cuenca"

r = rospy.Rate(4)
twist = Twist()

def init(node_name):
    global laser, base_vel_pub
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10) 
    laser = Laser()

#---------------------------- Suscriptor Punto Meta ------------------------------------

def reader():
   
    rospy.Subscriber("/meta_competencia",PoseStamped,queue_size=10)
    rate = rospy.Rate(1) # 1hz
    data = rospy.wait_for_message('/meta_competencia', PoseStamped, timeout=10)
    global xp, yp, zp, xo, yo, zo, wo, roll_goal, pitch_goal, theta_goal   
    xp=data.pose.position.x
    yp=data.pose.position.y
    zp=data.pose.position.z
    xo=data.pose.orientation.x
    yo=data.pose.orientation.y
    zo=data.pose.orientation.z
    wo=data.pose.orientation.w  
    (roll_goal, pitch_goal, theta_goal) = euler_from_quaternion ([xo, yo, zo, wo])   
    rate.sleep()
    
#------------------------- Funcion Acondicionamiento Laser ---------------------------
        
def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito
        right_scan=lectura[:180]
        left_scan=lectura[540:]
        front_scan=lectura[180:540]
        sd,si,sf=0,0,0
        if np.mean(left_scan)< 1.5: si=1
        if np.mean(right_scan)< 1.5: sd=1
        if np.mean(front_scan)< 1.8: sf=1
    except:
        sd,si,sf=0,0,0    
    return si,sd,sf

#---------------------------- Funcion Coordenadas ------------------------------------

def get_coords ():
    for i in range(10):   ###TF might be late, try 10 times
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans     
        except:
            trans=0    

#------------------------- Funciones Movimiento Robot --------------------------------

def move_base_vel(vx, vy, vw):
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)
def move_base(x,y,yaw,timeout):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) 
def move_forward(x,y,z,t):
    move_base(x,y,z,t)
def move_backward():
    move_base(-0.3,0,0,0.1)
def turn_left(x,y,z,t):
    move_base(x,y,z,t)
def turn_right(x,y,z,t):
    move_base(x,y,z,t)

#---------------------------------- Estados --------------------------------------    

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4','outcome5','outcome6','outcome7','outcome8'])
        self.counter = 0
    def execute(self,userdata):
        si,sd,sf=get_lectura_cuant()
        msg_cmd_vel = Twist()
        global angle_to_goal, angle_to_goal1, angle_to_goal_p, xt, yt, zt, inc_x, inc_y, theta
        punto_actual = get_coords()
        xt = punto_actual.transform.translation.x
        print("xt: " + str(xt))
        yt = punto_actual.transform.translation.y
        print("yt: " + str(yt))
        zt = punto_actual.transform.translation.z
        (roll, pitch, theta) = euler_from_quaternion ([punto_actual.transform.rotation.x, punto_actual.transform.rotation.y, punto_actual.transform.rotation.z, punto_actual.transform.rotation.w])	
        inc_x = xp - xt
        inc_y = yp - yt
        angle_to_goal = atan2(inc_y, inc_x)
        angle_to_goal1 = angle_to_goal - theta
        angle_to_goal_p = theta_goal - theta
        if (si==0 and sd==0 and sf==0): 
            if abs(inc_x) < 0.05 and abs(inc_y) < 0.05:
                if ((angle_to_goal_p >= 0 and angle_to_goal_p <np.pi) or (angle_to_goal_p <-np.pi)):
                    if abs(angle_to_goal_p) > 0.1: turn_left(0.0,0.0,0.12*np.pi,0.1)    
                elif ((angle_to_goal_p >=np.pi and angle_to_goal_p <2*np.pi) or (angle_to_goal_p < 0 and angle_to_goal_p > -(np.pi) )):
                    if abs(angle_to_goal_p) > 0.1: turn_right(0.0,0.0,-0.12*np.pi,0.1)         
                if abs(angle_to_goal_p) < 0.1:
                    punto_final = get_coords()
                    print ( 'tiempo = '+ str(punto_final.header.stamp.to_sec()) , punto_final.transform )
                    (roll_final, pitch_final, theta_final) = euler_from_quaternion ([punto_final.transform.rotation.x, punto_final.transform.rotation.y, punto_final.transform.rotation.z, punto_final.transform.rotation.w])
                    print("Theta Final: " + str(theta_final))
                    while 1:
                        move_base(0.0,0,0,0.1)    
            else:
                if ((angle_to_goal1 >= 0 and angle_to_goal1 <np.pi) or (angle_to_goal1 <-np.pi)):
                    if abs(angle_to_goal1) > 0.1: turn_left(0.0,0.0,0.12*np.pi,0.1)                 
                    if abs(angle_to_goal1) < 0.1: move_forward(0.3,0,0,0.1)
                elif ((angle_to_goal1 >=np.pi and angle_to_goal1 <2*np.pi) or (angle_to_goal1 < 0 and angle_to_goal1 > -(np.pi) )):
                    if abs(angle_to_goal1) > 0.1: turn_right(0.0,0.0,-0.12*np.pi,0.1)  
                    if abs(angle_to_goal1) < 0.1: move_forward(0.3,0,0,0.1)         
            return 'outcome1' 
        pub_cmd_vel.publish(msg_cmd_vel)
        r.sleep()   
        if (si==0 and sf==0 and sd==1): return 'outcome2'
        if (si==0 and sf==1 and sd==0): return 'outcome3'
        if (si==0 and sf==1 and sd==1): return 'outcome4'
        if (si==1 and sf==0 and sd==0): return 'outcome5'
        if (si==1 and sf==0 and sd==1): return 'outcome6'
        if (si==1 and sf==1 and sd==0): return 'outcome7'
        if (si==1 and sf==1 and sd==1): return 'outcome8'

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Derecha')
        if abs(inc_x) > 0.05 and abs(inc_y) > 0.05:
            turn_left(0.0,0.0,0.12*np.pi,0.08)
            move_forward(0.3,0,0,0.08)
        return 'outcome1'

class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Frente')
        if abs(inc_x) > 0.05 and abs(inc_y) > 0.05:
            if ((angle_to_goal1 >= 0 and angle_to_goal1 <np.pi) or (angle_to_goal1 <-np.pi)):
                print("if1")
                if abs(angle_to_goal1) > 0.1: turn_left(0.0,0.0,0.12*np.pi,0.1)                 
            elif ((angle_to_goal1 >=np.pi and angle_to_goal1 <2*np.pi) or (angle_to_goal1 < 0 and angle_to_goal1 > -(np.pi))):
                print("if2")
                if abs(angle_to_goal1) > 0.1: turn_right(0.0,0.0,-0.12*np.pi,0.1)  
        return 'outcome1'

class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Frente - Derecha')
        if abs(inc_x) > 0.05 and abs(inc_y) > 0.05:
            turn_left(0.0,0.0,0.12*np.pi,0.1)
        return 'outcome1'

class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Izquierda')
        if abs(inc_x) > 0.05 and abs(inc_y) > 0.05:
            turn_right(0.0,0.0,-0.12*np.pi,0.08)
            move_forward(0.3,0,0,0.08)
        return 'outcome1' 

class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Izquierda - Derecha')
        move_forward(0.3,0,0,0.1)
        return 'outcome1' 

class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Frente - Izquierda')
        if abs(inc_x) > 0.05 and abs(inc_y) > 0.05:
            turn_right(0.0,0.0,-0.12*np.pi,0.1)
        return 'outcome1' 

class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Reversa')
        move_backward()
        return 'outcome1' 

class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    def execute(self,userdata):
        rospy.loginfo('Estado Comparacion')
        if abs(inc_x) < 0.05 and abs(inc_y) < 0.05:
            if ((angle_to_goal_p >= 0 and angle_to_goal_p <np.pi) or (angle_to_goal_p <-np.pi)):
                if abs(angle_to_goal_p) > 0.1: turn_left(0.0,0.0,0.12*np.pi,0.1)    
            elif ((angle_to_goal_p >=np.pi and angle_to_goal_p <2*np.pi) or (angle_to_goal_p < 0 and angle_to_goal_p > -(np.pi) )):
                if abs(angle_to_goal_p) > 0.1: turn_right(0.0,0.0,-0.12*np.pi,0.1)         
            if abs(angle_to_goal_p) < 0.1:
                punto_final = get_coords()
                print ( 'tiempo = '+ str(punto_final.header.stamp.to_sec()) , punto_final.transform )
                (roll_final, pitch_final, theta_final) = euler_from_quaternion ([punto_final.transform.rotation.x, punto_final.transform.rotation.y, punto_final.transform.rotation.z, punto_final.transform.rotation.w])
                print("Theta Final: " + str(theta_final))
                while 1:
                    move_base(0.0,0,0,0.1)
        else:
            if ((angle_to_goal1 >= 0 and angle_to_goal1 <np.pi) or (angle_to_goal1 <-np.pi)):
                if abs(angle_to_goal1) > 0.1: turn_left(0.0,0.0,0.12*np.pi,0.1)                 
                if abs(angle_to_goal1) < 0.1: move_forward(0.3,0,0,0.1)
            elif ((angle_to_goal1 >=np.pi and angle_to_goal1 <2*np.pi) or (angle_to_goal1 < 0 and angle_to_goal1 > -(np.pi) )):
                if abs(angle_to_goal1) > 0.1: turn_right(0.0,0.0,-0.12*np.pi,0.1)  
                if abs(angle_to_goal1) < 0.1: move_forward(0.3,0,0,0.1)
        return 'outcome1'

def main():
    global pub_cmd_vel
    print("Meta Competencia - " + EQUIPO)
    reader()	
    print("x goal: " + str(xp))
    print("y goal: " + str(yp))
    print("theta goal: " + str(theta_goal))
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    print('Inicializando')
    rospy.sleep(1)
    punto_inicial = get_coords()
    print ( 'tiempo = '+ str(punto_inicial.header.stamp.to_sec()) , punto_inicial.transform )
    
if __name__ == '__main__':
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0
    sm.userdata.clear = False   
    with sm:
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_0', 'outcome2':'s_1','outcome3':'s_2','outcome4':'s_3','outcome5':'s_4', 'outcome6':'s_5','outcome7':'s_6','outcome8':'s_7',})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        smach.StateMachine.add("s_8",   S8(),  transitions = {'outcome1':'s_0','outcome2':'END'})
    try:
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        main()
    except rospy.ROSInterruptException:
        pass
        
outcome = sm.execute()
    
