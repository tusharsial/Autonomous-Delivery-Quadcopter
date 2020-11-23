#!/usr/bin/env python
'''
This python file runs a ROS-node of name position_control which controls the position (lattitude, longitude and altitude) of the eDrone. This file generates command values for the attitude controller.
'''
# Importing the required files
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from pid_tune.msg import PidTune
from std_msgs.msg import String
from vitarana_drone.srv import Gripper
from sensor_msgs.msg import LaserScan 
import rospy
import copy
import time
import re 
# Declaring some global variables
global c
c = 0
global initial_time
initial_time = 0.0
global range_sensor
range_sensor = [0.0, 0.0, 0.0, 0.0]
global drop
drop = 0.0
global result
result = False
global starting_point
starting_point=[19.0009248718, 71.9998318945, 22.16]
global destination
destination = [0.0, 0.0, 0.0]
# Declaring a class
class Edrone():
    count=0
    def __init__(self):
        rospy.init_node('position_controller')
        # This corresponds to current position of your drone
        self.drone_position = [0.0, 0.0, 0.0]
        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]
        self.Kp = [2600.0*50.0, 50.0*4998.0, 5000.0*0.04]
        self.Ki = [0.0, 0.0, 441.0*0.002]
        self.Kd = [608.0*16000, 14000.0*1070.0, 4250.0]
        # Creating topic for publishing command values (Throttle,roll,pitch,yaw)
        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 1500.0
        self.drone_cmd.rcRoll = 1500.0
        self.drone_cmd.rcPitch = 1500.0
        self.drone_cmd.rcYaw = 1500.0
        # Some Other variables
        self.error = [0.0, 0.0, 0.0]
        self.proportional_error = [0.0, 0.0, 0.0]
        self.derivative_error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.out_roll = 0.0
        self.out_pitch = 0.0
        self.out_throttle = 0.0
        self.sample_time = 0.060
        self.grp_check = ''
        self.drop = 0.0
        # Declaring lat_error of message type Float32 and initializing values
        self.lat_error = Float32()
        self.lat_error.data = 0.0
        # Declaring long_error of message type Float32 and initializing values
        self.long_error = Float32()
        self.long_error.data = 0.0
        # Declaring alt_error of message type Float32 and initializing values 
        self.alt_error = Float32()
        self.alt_error.data = 0.0
        # Declaring publishers
        self.drone_command_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.lat_error_pub = rospy.Publisher('/lat_error', Float32, queue_size = 1)
        self.long_error_pub = rospy.Publisher('/long_error', Float32, queue_size = 1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float32, queue_size = 1)  
        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # Commenting out PID tuner after getting the tuned values        
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        #rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        #rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)  
        rospy.Subscriber('/dest_cord', String, self.final_point_qr) 
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_top_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gp_check) 
    # Callback definations 
    def gps_callback(self, msg):
     self.drone_position[0] = msg.latitude
     self.drone_position[1] = msg.longitude
     self.drone_position[2] = msg.altitude 
    
    def roll_set_pid(self, lat):
     self.Kp[0] = lat.Kp*50
     self.Ki[0] = lat.Ki*0.08
     self.Kd[0] = lat.Kd*16000
    
    def pitch_set_pid(self, longi):
     self.Kp[1] = longi.Kp*50
     self.Ki[1] = longi.Ki*0.08
     self.Kd[1] = longi.Kd*14000
    
    def altitude_set_pid(self, alt):
     self.Kp[2] = alt.Kp*0.04
     self.Ki[2] = alt.Ki*0.002
     self.Kd[2] = alt.Kd*1
    
    def range_top_callback(self, rtop):
        global range_sensor
        range_sensor[0] = rtop.ranges[0]
        range_sensor[1] = rtop.ranges[1]
        range_sensor[2] = rtop.ranges[2]
        range_sensor[3] = rtop.ranges[3]

    def final_point_qr(self, dest_string):
       global destination
       dest_string = re.findall(r'"(.*?)"',str(dest_string))[0]
       destination = [float(idx) for idx in dest_string.split(',')]

    def gp_check(self, check):
      self.grp_check = check.data  

    # Some function definations
    def lat_to_x(self, input_latitude):
         return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
         return -105292.0089353767 * (input_longitude - 72)      
    
    def take_off(self):
         global starting_point 
         set_point = [starting_point[0],starting_point[1], (starting_point[2]+3)]
         return set_point
    
    def follow_wall(self,i,final_setpoint): 
          global c, initial_time,range_sensor, starting_point
          if c == 0:
            initial_time = time.time()
            c = 1
          print("Following wall")  
          rand_point=[0.0,0.0,0.0]
          if i==0:
            print(i)
            rand_point=[self.drone_position[0]-1E-4,self.drone_position[1]+(range_sensor[0]-5)/(-105292.0089353767),starting_point[2]+3]
          elif i==1:
            print(i)
            rand_point=[self.drone_position[0]+(range_sensor[1]-5)/110692.07029,self.drone_position[1]-1E-4,starting_point[2]+3]
          elif i==2:
            print(i)
            rand_point=[self.drone_position[0]-1E-4,self.drone_position[1]-(range_sensor[2]-5)/105292.0089353767,starting_point[2]+3]
          if i==3:
            print(i)
            rand_point=[self.drone_position[0]-(range_sensor[3]-5)/110692.07029,self.drone_position[1]+1E-4,starting_point[2]+3]
          else:
            pass
          c = 1
          return rand_point 

    def distance_calculated(self,p1, p2):
      p0 = [self.lat_to_x(self.drone_position[0]), self.long_to_y(self.drone_position[1]), self.drone_position[2]]
      u1 = abs((p2[1]-p1[1])*p0[0] - (p2[0] -p1[0])*p0[1] + p2[0]*p1[1] - p1[0]*p2[1])
      u2 =  (pow((p2[0]-p1[0]),2) + pow((p2[1]-p1[1]),2))**(0.5)
      return u1/u2          

    def final_distance_calculated(self,p1):
      u1 = (pow((p1[0]-self.lat_to_x(self.drone_position[0])),2) + pow((p1[1]-self.long_to_y(self.drone_position[1])),2))**(0.5)
      return u1

    def follow_destination(self,final_setpoint):
      global starting_point
      print("Go to final setpoint")
      final_setpoint=[final_setpoint[0],final_setpoint[1], starting_point[2]+3]
      return final_setpoint

    def follow_path(self,final_setpoint):
            global range_sensor, starting_point 
            dummy_point=[0.0,0.0,0.0]
            if range_sensor[0]<=15 and range_sensor[0]>0.5:
                if range_sensor[0]>5.1:
                  print("Moving front")
                  dummy_point=[self.drone_position[0],self.drone_position[1]+(range_sensor[0]-5)/(-105292.0089353767),starting_point[2]+3] 
                else:
                  print("Front sensor working")
                  dummy_point=self.follow_wall(0,final_setpoint)
            elif abs(range_sensor[1])<=15 and range_sensor[1]>0.5:
                if abs(range_sensor[1])>5.1:
                  print("Moving right")
                  dummy_point=[self.drone_position[0]+(range_sensor[1]-5)/110692.07029,self.drone_position[1],starting_point[2]+3]
                else:
                  print("Right sensor working")
                  dummy_point=self.follow_wall(1,final_setpoint)
            elif abs(range_sensor[2])<=15 and range_sensor[2]>0.5:
                if abs(range_sensor[2])>5.1:
                  print("Moving back")
                  dummy_point=[self.drone_position[0],self.drone_position[1]-(range_sensor[2]-5)/(-105292.0089353767),starting_point[2]+3]
                  print(dummy_point)
                else:
                  print("Back sensor working")
                  dummy_point=self.follow_wall(2,final_setpoint) 
            elif abs(range_sensor[3])<=15 and range_sensor[3]>0.5:
                if abs(range_sensor[3])>5.1:
                  print("Moving left")
                  dummy_point=[self.drone_position[0]-(range_sensor[3]-5)/110692.07029,self.drone_position[1],starting_point[2]+3]
                else:
                  print("Left sensor working")
                  dummy_point=self.follow_wall(3,final_setpoint)
            else:
              print("Go to final")
              dummy_point=[final_setpoint[0],final_setpoint[1], starting_point[2]+3]
              initial_time = 0.0
            return dummy_point   
             
    def gripper_detach(self):
      global drop
      self.act = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
      if drop == 1.0:
         result = self.act(False)   
         
    def land(self,final_setpoint):
         set_point = [final_setpoint[0],final_setpoint[1],final_setpoint[2]]
         return set_point
     
    def pid(self,dummy_point):
        global range_sensor,drop
        # Calculating setpoints for lattitude, longitude and altitude axises 
        self.error[0] = dummy_point[0] - self.drone_position[0]
        self.error[1] = dummy_point[1] - self.drone_position[1]
        self.error[2] = dummy_point[2] - self.drone_position[2]
        # Calculating proportianal errors for lattitude, longitude and altitude axises
        self.proportional_error[0] = self.Kp[0]*self.error[0] 
        self.proportional_error[1] = self.Kp[1]*self.error[1]
        self.proportional_error[2] = self.Kp[2]*self.error[2]
        # Calculating derivative errors for lattitude, longitude, and altitude axises
        self.derivative_error[0] = (self.error[0]-self.prev_error[0])*self.Kd[0]
        self.derivative_error[1] = (self.error[1]-self.prev_error[1])*self.Kd[1]
        self.derivative_error[2] = (self.error[2]-self.prev_error[2])*self.Kd[2]
        # Calculating integral errors for lattitude, longitude and altitude axises
        self.sum_error[0] = max(-3,min(3,self.sum_error[0]+self.error[0]*self.Ki[0]))
        self.sum_error[1] = max(-3,min(3,self.sum_error[1]+self.error[1]*self.Ki[1]))
        self.sum_error[2] = max(-3,min(3,self.sum_error[2]+self.error[2]*self.Ki[2]))
        # Calculating required PID outputs for lattitude,longitude and altitude axises
        self.out_roll = self.proportional_error[0]+self.derivative_error[0]+self.sum_error[0]
        self.out_pitch = self.proportional_error[1]+self.derivative_error[1]+self.sum_error[1]
        self.out_throttle = self.proportional_error[2]+self.derivative_error[2]+self.sum_error[2]
        # Calculating roll, pitch, yaw and throttle 
        self.drone_cmd.rcRoll = max(1490,min(1510,(1500 + self.out_roll)))
        self.drone_cmd.rcPitch = max(1490,min(1510,(1500 + self.out_pitch)))
        self.drone_cmd.rcThrottle = 1500 + self.out_throttle 
        # Assigning lattitude error,longitude error and altitude error to the 3 pub    
        self.lat_error.data = self.error[0]
        self.long_error.data = self.error[1]
        self.alt_error.data = self.error[2]
        # Publishing required values    
        self.drone_command_pub.publish(self.drone_cmd)
        self.lat_error_pub.publish(self.lat_error)
        self.long_error_pub.publish(self.long_error)
        self.alt_error_pub.publish(self.alt_error) 
        # Updating the previous error values     
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        # Printing the instantaneous location of the drone (lattitude, longitude and altitude) 
        self.act = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        if self.grp_check == 'True' and drop == 0.0:
         result = self.act(True)   
         drop = drop+1        
        return self.error    
        
if __name__ == '__main__':
  try:
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time
    # Algorithm for changing the setpoints accordingly to trace the path as mentioned in Task 1B
    final_setpoint=[19.0007046575, 71.9998955286, 22.1599967919]
    sp_xy = [e_drone.lat_to_x(starting_point[0]), e_drone.long_to_y(starting_point[1]), starting_point[2]+3]
    errors=[0.0, 0.0 , 0.0]
    set_point = [0.0, 0.0, 0.0]
    final_distance = 0.0
    flag = 0.0
    distance = 0.0 
    while not rospy.is_shutdown():
      while flag < 4:
        fp_xy = [e_drone.lat_to_x(final_setpoint[0]), e_drone.long_to_y(final_setpoint[1]), starting_point[2]+3]
        distance = e_drone.distance_calculated(sp_xy,fp_xy)
        final_distance = e_drone.final_distance_calculated(fp_xy) 
        
        if flag == 0:
          print("Taking off")
          set_point = e_drone.take_off()
          errors = e_drone.pid(set_point)
          if abs(errors[2]) < 0.3:
              set_point = e_drone.follow_destination(final_setpoint)
              flag = 1 
              print("lift complete")
  
        elif flag == 1 :          
          if distance < 1.5 and  time.time()-initial_time > 7 and initial_time != 0:
            print("Exit obstacle")
            c = 0
            intial_time = 0
            set_point=e_drone.follow_destination(final_setpoint)
            flag = 2
          elif final_distance < 5:
            flag = 2
            print(flag)
            print("Ignore obstacle and just go to next setpoint",flag)
          elif range_sensor[0] > 0.4 and range_sensor[1] > 0.4 and range_sensor[2] > 0.4 and range_sensor[3]>0.4 :
            print("Object detected")
            set_point=e_drone.follow_path(final_setpoint)   
          errors=e_drone.pid(set_point)
            
        elif flag == 2:
          set_point = e_drone.follow_destination(final_setpoint)
          errors = e_drone.pid(set_point)
          print(errors)
          if abs(errors[0])<0.000004 and abs(errors[1]) < 0.000004 and abs(errors[2]) < 0.08:
            flag = 3
            print("Preparing for Landing")
            print(flag)
          
        elif flag == 3 :
           set_point = [final_setpoint[0],final_setpoint[1],final_setpoint[2]]
           errors = e_drone.pid(set_point)
           print(errors)
           if abs(errors[0])<0.000002 and abs(errors[1]) < 0.000002 and abs(errors[2]) < 0.024:
             if result == True :
               print("Destination reached. Dropping the parcel.")
               e_drone.gripper_detach()
               flag = 4
               print(flag)
             elif result == False:
               final_setpoint = destination
               errors = e_drone.pid(final_setpoint)
               print("Box picked up. Preparing to move to drop location")
               flag = 1
               print(flag)  
        print("Final distance from goal",final_distance)
        r.sleep()                

  except rospy.ROSInterruptException: 
      pass
    

        