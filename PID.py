import math
import rospy
import time
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist ,Point
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class PID:
    """PID Controller"""

    def __init__(self, P=0.2, I=0.0, D=0.0,SetPoint=0.0, current_time = None, barrier = None):


        self.clear()
        #Proportional constant
        self.Kp = P
        #Integral constant
        self.Ki = I
        #Derivative constant
        self.Kd = D
        #This determines the maximum set Rate of PID response(the actual rate might not be '1/sample_time')
        self.sample_time = 0.00
        #initiates the last_time for dt calculations 
        self.current_time = current_time if current_time is not None else rospy.Time(0).now().to_nsec()
        self.last_time = self.current_time
        #the target value to be reached
        self.SetPoint = SetPoint
        #barrier for output value
        self.barrier = barrier
        #setting errorSyndrome
        self.errortype = "-"
        #setting scaller for time 
        self.timescaler = 1

        # debug
        self.print_debug_info = False
        self.active = True

        
    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0
        #Proportional constituent of the output
        self.PTerm = 0.0
        #Integral constituent of the output
        self.ITerm = 0.0
        #Derivative constituent of the output
        self.DTerm = 0.0
        #Memory for the error calulated in the previous iteration
        self.last_error = 0.0
        #windup gaurd
        self.windup_guard = 10.0
        # K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """

        # calculating current error
        if self.errortype =="+":
            self.error = self.SetPoint + feedback_value
        else:
            self.error = self.SetPoint - feedback_value

        # setting current time
        self.current_time = current_time if current_time is not None else rospy.Time(0).now().to_nsec()
        # calculation for dt(difference in time)
        delta_time = self.current_time - self.last_time
        # calculation for de(difference in error)
        delta_error = self.error - self.last_error

        if (delta_time >= self.sample_time):
            #PTerm
            self.PTerm = self.Kp * self.error
            #Iterm
            self.ITerm += self.error * (delta_time/self.timescaler)
            #Setting ITerm constrains 
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            #DTerm
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / (delta_time/self.timescaler)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = self.error
            #return value to be added to the current value
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if self.barrier:
                if (self.output < -self.barrier):
                    self.output = -self.barrier
                elif (self.output > self.barrier):
                    self.output = self.barrier
           
            if not self.active:
                self.output = 0.0
            if self.print_debug_info:
                print(" kp:{}||ki:{}||kd:{}\n error:{}\n last_error:{}\n delta_error:{}\n SetPoint:{}\n feedbackInput:{}\n P_Constituent:{}\n I_Constituent:{}\n D_Constituent:{}\n OUTPUT:{}\n".format(self.Kp,self.Ki,self.Kd,self.error,self.last_error,delta_error,self.SetPoint,feedback_value,self.PTerm,self.Ki * self.ITerm,self.Kd * self.DTerm,self.output))

        return(self.output)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def reset_SetPoint(self,value):
        '''resets setpoint(target) value'''
        self.SetPoint = value

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

    def setErrorSyndrome(self,type):
        self.errortype = type

    def show_debug_info(self,info):
        """ set show debug info to True to view the debug info """
        self.print_debug_info = info

    def deactivate(self):
        ''' to deactivate the pid output to get the output as zero else the default is active and the calculated output will be retured ''' 
        self.active = False

    def activate(self):
        """to activate pid output"""
        self.active = True

    def set_TimeScaler(self,value):
        '''to set the value of time scaler
           TimeScaler - when the time is a large number the i term and d term tend to overwhelm and result in overshoot'''
        self.timescaler = value