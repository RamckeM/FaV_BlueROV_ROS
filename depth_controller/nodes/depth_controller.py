#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float32
from std_msgs.msg import Float64

MAX_THRUST = 1

class DepthControlNode():
    def __init__(self):
        rospy.init_node("depth_controller")

        self.setpoint = 0
        self.depth = 0

        self.kp = rospy.get_param('~kp')
        self.ki = rospy.get_param('~ki')
        self.kd = rospy.get_param('~kd')
        self.offset = rospy.get_param('~offset')
        self.TIMESTEP = rospy.get_param('~timestep')
        self.antiWindup = rospy.get_param('~antiWindup')

        self.timeout = False

        self.error = 0
        self.last_error = 0
        self.integral_error = 0

        self.control_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)
        
        self.setpoint_sub = rospy.Subscriber("depth_setpoint", Float64, self.setpoint_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("depth", Float32, self.depth_callback, queue_size=1)

        self.timer = threading.Timer(5,timeout)
        self.timer.start


        self.Proportional_pub = rospy.Publisher("Proportional", Float64, queue_size=1)
        self.Integral_pub = rospy.Publisher("Integral", Float64, queue_size=1)
        self.Differential_pub = rospy.Publisher("Differential", Float64, queue_size=1)

        self.tuneKp_sub = rospy.Subscriber("tuneKp", Float64, self.tuneKp_callback, queue_size=1)
        self.tuneKi_sub = rospy.Subscriber("tuneKi", Float64, self.tuneKi_callback, queue_size=1)
        self.tuneKd_sub = rospy.Subscriber("tuneKd", Float64, self.tuneKd_callback, queue_size=1)
        self.tuneOffset_sub = rospy.Subscriber("tuneOffset", Float64, self.tuneOffset_callback, queue_size=1)
    
    def tuneKp_callback(self, msg):
        self.kp = msg.data

    def tuneKi_callback(self, msg):
        self.ki = msg.data

    def tuneKd_callback(self, msg):
        self.kd = msg.data

    def tuneOffset_callback(self, msg):
        self.offset = msg.data


    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def depth_callback(self, msg):
        self.timer
        self.timer.cancel()
        self.timer = threading.Timer(5,timeout)
        self.timer.start()
        self.timeout = False

        self.depth = msg.data


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            error = self.setpoint - self.depth
            derivative_error = (self.last_error - error) / self.TIMESTEP
            self.last_error = error

            # Check for safety area
            if self.setpoint < -0.8 or self.setpoint > -0.1 or self.depth < -0.8 or self.depth > -0.1 or self.timeout is True:
                control_output = 0.0
            else:
                control_output = self.kp * error + self.ki * self.integral_error + self.kd * derivative_error + self.offset

            
            msg1 = Float64()
            msg2 = Float64()
            msg3 = Float64()
            msg1.data = self.kp * error
            msg2.data = self.ki * self.integral_error
            msg3.data = self.kd * derivative_error
            self.Proportional_pub.publish(msg1)
            self.Integral_pub.publish(msg2)
            self.Differential_pub.publish(msg3)


            # Limiting control effort
            if control_output > 1.0:
                control_output = 1.0
            elif control_output < -1.0:
                control_output = -1.0

            # AntiWindup functionality
            if abs(control_output) > MAX_THRUST or self.setpoint < -0.8 or self.setpoint > -0.1 or self.depth < -0.8 or self.depth > -0.1 or self.timeout is True:
                if self.antiWindup is True:
                    self.integral_error = self.integral_error
                else:
                    self.integral_error = self.integral_error + error * self.TIMESTEP
            else:
                self.integral_error = self.integral_error + error * self.TIMESTEP

            msg = Float64()
            msg.data = control_output
            self.control_pub.publish(msg)
            rate.sleep()


def timeout(self):
    print("No message received for 5 seconds")
    self.timeout = True



def main():
    node = DepthControlNode()
    node.run()

if __name__ == "__main__":
    main()
