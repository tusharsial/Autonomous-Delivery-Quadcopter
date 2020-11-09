#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_control which controls the position (lattitude, longitude and altitude) of the eDrone. This file generates command values for the attitude controller.
'''
# Importing the required files
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from pid_tune.msg import PidTune
import rospy


class Edrone():
    
    count=0
    def __init__(self):
        rospy.init_node('position_controller')
        # This corresponds to current position of your drone
        self.drone_position = [0.0, 0.0, 0.0]
        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]
        self.Kp = [2600.0*100.0, 100.0*4998, 5000.0*0.04]
        self.Ki = [0.0, 0.0, 441.0*0.002]
        self.Kd = [608.0*16000, 16000.0*1048, 4250.0]
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
        '''
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)  
        ''' 
        
    # Callback definations 
    def gps_callback(self, msg):
     
     self.drone_position[0] = msg.latitude
     self.drone_position[1] = msg.longitude
     self.drone_position[2] = msg.altitude 
    
    def roll_set_pid(self, lat):
    
     self.Kp[0] = lat.Kp*100
     self.Ki[0] = lat.Ki*100
     self.Kd[0] = lat.Kd*16000
    
    def pitch_set_pid(self, longi):
    
     self.Kp[1] = longi.Kp*100
     self.Ki[1] = longi.Ki*100
     self.Kd[1] = longi.Kd*16000
    
    def altitude_set_pid(self, alt):
    
     self.Kp[2] = alt.Kp*0.04
     self.Ki[2] = alt.Ki*0.002
     self.Kd[2] = alt.Kd*1
    

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
        self.sum_error[0] = (self.sum_error[0]+self.error[0])*self.Ki[0]
        self.sum_error[1] = (self.sum_error[1]+self.error[1])*self.Ki[1]
        self.sum_error[2] = (self.sum_error[2]+self.error[2])*self.Ki[2]
        # Calculating required PID outputs for lattitude,longitude and altitude axises
        self.out_roll = self.proportional_error[0]+self.derivative_error[0]+self.sum_error[0]
        self.out_pitch = self.proportional_error[1]+self.derivative_error[1]+self.sum_error[1]
        self.out_throttle = self.proportional_error[2]+self.derivative_error[2]+self.sum_error[2]
        # Calculating roll, pitch, yaw and throttle 
        self.drone_cmd.rcRoll = 1500 + self.out_roll
        self.drone_cmd.rcPitch = 1500 + self.out_pitch
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
        print(self.drone_position)  
        return self.error    
        

if __name__ == '__main__':
   
  try:
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time
    # Algorithm for changing the setpoints accordingly to trace the path as mentioned in Task 1B.
    starting_point=[19.0 ,72.0 , 0.31]
    height=3.0
    flag=0
    errors=[0.0, 0.0 , 0.0]
    final_setpoint=[19.0000451704,72.0, 0.31]
    dummy_point=[[starting_point[0],starting_point[1],height],[final_setpoint[0],starting_point[1],height],[final_setpoint[0],final_setpoint[1],final_setpoint[2]]]
    
    while not rospy.is_shutdown():
       
        while flag<3:
            if flag==0:
                errors=e_drone.pid(dummy_point[0])
                if abs(errors[2])<0.02 :
                    flag+=1
                    print(flag)
            elif flag==1 :
                errors=e_drone.pid(dummy_point[1])
                if abs(errors[0])<0.000000004517 :
                    flag+=1
                    print(flag)
            elif flag==2 :
                errors=e_drone.pid(dummy_point[2])
                if abs(errors[2])<0.02 :
                    flag+=1
                    print(flag)
                    exit()
            r.sleep()

       
  except rospy.ROSInterruptException: 
      pass
    
