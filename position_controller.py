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
        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 1500.0
        self.drone_cmd.rcRoll = 1500.0
        self.drone_cmd.rcPitch = 1500.0
        self.drone_cmd.rcYaw = 1500.0
        # Other variables
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
        self.range_top = [0.0, 0.0, 0.0, 0.0]
        self.range_limit = [0.0, 0.0]
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
        #  -----------------------------------------------------------------------------------------------------------
        
        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        
        # Commenting out PID tuner after getting the tuned values
        
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        #rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        #rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)  
         
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
        self.range_top[0] = rtop.ranges[0]
        self.range_top[1] = rtop.ranges[1]
        self.range_top[2] = rtop.ranges[2]
        self.range_top[3] = rtop.ranges[3]
        self.range_limit[0] = rtop.range_min
        self.range_limit[1] = rtop.range_max

    def gp_check(self, check):
      self.grp_check = check.data  
    
    def lat_to_x(self, input_latitude):
         return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
         return -105292.0089353767 * (input_longitude - 72)
    
    def take_off(self, starting_point):
         set_point = [starting_point[0],starting_point[1], (starting_point[2]+1.5)]
         return set_point
    
    def follow_wall(self):
        #while range_top[0] < range_limit[1] or range_top[1] < range_limit[1] or range_top[2] < range_limit[1] or range_top[3] < range_limit[1] :
        #while range_top <= 5.01 and range_top>= 4.99:
        set_point = [self.drone_position[0],self.drone_position[1]+0.1,self.drone_position[2]]
        return set_point      
        
    def follow_path(self,final_setpoint):
        if self.range_top[3] <= 5.01:
          print(self.range_top)
          set_point = self.follow_wall()
          print('hello')
 
        else:
         set_point = [final_setpoint[0],final_setpoint[1], (final_setpoint[2]+1.5)]
        
        return set_point
    
    def land(self,final_setpoint):
         set_point = final_setpoint
         return set_point
     
    def pid(self,dummy_point):
        
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
        
        if(self.grp_check == 'True' and self.drop == 0.0):
         self.act = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
         self.result = self.act(True)   
         self.drop = self.drop+1
        # Updating the previous error values     
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        # Printing the instantaneous location of the drone (lattitude, longitude and altitude) 
        #print(dummy_point)
        print(self.range_top)  
        return self.error    
        

if __name__ == '__main__':
   
  try:
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time
    # Algorithm for changing the setpoints accordingly to trace the path as mentioned in Task 1B.
    starting_point=[19.000027107343833, 71.9999999994294, 0.30999747813892525]
    errors=[0.0, 0.0 , 0.0]
    set_point = [0.0, 0.0, 0.0]
    final_setpoint=[19.0,72.0, 0.31]
    flag = 0.0
    while not rospy.is_shutdown():
      while flag < 4:
        
        if flag == 0:
          set_point = e_drone.take_off(starting_point)
          #set_point = [e_drone.lat_to_x(set_point[0]),e_drone.long_to_y(set_point[1]), set_point[2]]
          errors = e_drone.pid(set_point)
          if abs(errors[2]) < 0.016:
              flag = 1
              print(flag) 

        
        if flag == 1 :
          set_point = e_drone.follow_path(final_setpoint)
          errors = e_drone.pid(set_point) 
          #set_point = [e_drone.lat_to_x(set_point[0]),e_drone.long_to_y(set_point[1]), set_point[2]]
          if abs(errors[0]) < 0.0000004517:
            flag = 2 
            print(flag) 
   
        elif flag == 2 :
           set_point = e_drone.land(final_setpoint)
           errors = e_drone.pid(set_point)
           #set_point = [e_drone.lat_to_x(set_point[0]),e_drone.long_to_y(set_point[1]), set_point[2]]
        r.sleep()                
       
       
  except rospy.ROSInterruptException: 
      pass
    

        
