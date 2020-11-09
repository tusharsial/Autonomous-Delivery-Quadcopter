#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone. This generates propeller speed values for the e-Drone
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0
                
        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [1485.0*0.06, 1965.0*0.04, 5000.0*5.0]
        self.Ki = [0.0, 0.0, 89.0*0.002]
        self.Kd = [3930.0*0.3, 5000.0*0.2, 5000.0*0.4]
        # Other Parameters
        self.error = [0.0, 0.0, 0.0]
        self.proportional_error = [0.0, 0.0, 0.0]
        self.derivative_error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.max_values = [1024.0, 1024.0, 1024.0, 1024.0]
        self.min_values = [0.0, 0.0, 0.0, 0.0]
        self.setpoint_throttle = 1500.0
        self.throttle_pwm = 0.0
        self.out_roll = 0.0
        self.out_yaw = 0.0
        self.out_pitch = 0.0
        self.final_roll = 0.0
        self.final_pitch = 0.0
        self.final_yaw = 0.0
        # Declaring roll_error of message type Float32 and initializing values
        self.roll_error = Float32()
        self.roll_error.data = 0.0
        # Declaring pitch_error of message type Float32 and initializing values
        self.pitch_error = Float32()
        self.pitch_error.data = 0.0
        # Declaring yaw_error of message type Float32 and initializing values 
        self.yaw_error = Float32()
        self.yaw_error.data = 0.0
        #Declaring zero_error of message type Float32 and initializing values
        self.zero_error = Float32()
        self.zero_error.data = 0.0
        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32,queue_size = 1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32,queue_size = 1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32,queue_size = 1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32,queue_size = 1)    
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        
        # Commenting out PID tuner after getting the tuned values
        '''
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
        '''
    # Callback functions
    
    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        
    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_throttle = msg.rcThrottle
        
    # Callback function for /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3
    # Callback function for /pid_tuning_pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp*0.04
        self.Ki[1] = pitch.Ki*0.006
        self.Kd[1] = pitch.Kd*0.2
    # Callback function for /pid_tuning_yaw
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp*6.0
        self.Ki[2] = yaw.Ki*0.006
        self.Kd[2] = yaw.Kd*0.5
     
   # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1]*0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2]*0.02 - 30   
        # Converting the range of 1000 to 2000 to 0 to 1024 for throttle.
        self.throttle_pwm = self.setpoint_throttle*1.024 -1024
        # Calculating setpoints for roll, yaw and pitch axises 
        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[1]
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[0]
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]
        # Calculating proportianal errors for roll, yaw and pitch axises
        self.proportional_error[0] = self.Kp[0]*self.error[0] 
        self.proportional_error[1] = self.Kp[1]*self.error[1]
        self.proportional_error[2] = self.Kp[2]*self.error[2]
        # Calculating derivative errors for roll, yaw, and pitch axises
        self.derivative_error[0] = (self.error[0]-self.prev_error[0])*self.Kd[0]
        self.derivative_error[1] = (self.error[1]-self.prev_error[1])*self.Kd[1]
        self.derivative_error[2] = (self.error[2]-self.prev_error[2])*self.Kd[2]
        # Calculating integral errors for roll, yaw and pitch axises
        self.sum_error[0] = (self.sum_error[0]+self.error[0])*self.Ki[0]
        self.sum_error[1] = (self.sum_error[1]+self.error[1])*self.Ki[1]
        self.sum_error[2] = (self.sum_error[2]+self.error[2])*self.Ki[2]
        # Calculating required PID outputs for roll,yaw and pitch axises
        self.out_roll = self.proportional_error[0]+self.derivative_error[0]+self.sum_error[0]
        self.out_pitch = self.proportional_error[1]+self.derivative_error[1]+self.sum_error[1]
        self.out_yaw = self.proportional_error[2]+self.derivative_error[2]+self.sum_error[2]
        #print(self.drone_orientation_euler[0],self.drone_orientation_euler[1])
        self.pwm_cmd.prop1 = max(0,min(1*(-self.out_roll + self.out_pitch + self.throttle_pwm - self.out_yaw),1023))
        self.pwm_cmd.prop2 = max(0,min(1*(-self.out_roll - self.out_pitch + self.throttle_pwm + self.out_yaw),1023))
        self.pwm_cmd.prop3 = max(0,min(1*(self.out_roll - self.out_pitch + self.throttle_pwm - self.out_yaw),1023))
        self.pwm_cmd.prop4 = max(0,min(1*(self.out_roll + self.out_pitch + self.throttle_pwm + self.out_yaw),1023))
        # Assigning roll error,pitch error, yaw error and zero error to the 4 pub
        self.roll_error.data = self.error[0]
        self.pitch_error.data = self.error[1]
        self.yaw_error.data = self.error[2]
        self.zero_error.data = 0.0
        # Publishing required values
        self.pwm_pub.publish(self.pwm_cmd)
        self.roll_error_pub.publish(self.roll_error)
        self.pitch_error_pub.publish(self.pitch_error)
        self.yaw_error_pub.publish(self.yaw_error)
        self.zero_error_pub.publish(self.zero_error) 
        # Updating the previous error values 
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]  

if __name__ == '__main__':
   try:
    time.sleep(7)
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # Rate in Hz based upon your desired PID sampling time
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
   except rospy.ROSInterruptException: 
      pass
