#!/usr/bin/env python
import rospy
import math
from mavros_msgs.msg import MotorSetpoint
from mavros_msgs.srv import CommandBool


class MyFirstNode():
   def __init__(self):
      rospy.init_node("motor_command_sender")
      self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint",
                                          MotorSetpoint,
                                          queue_size=1)
      self.arm_vehicle()

   def arm_vehicle(self):
      # wait until the arming serivce becomes available
      rospy.wait_for_service("mavros/cmd/arming")
      # connect to the service
      arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
      # call the service to arm the vehicle until service call was successfull
      while not arm(True).success:
            rospy.logwarn("Could not arm vehicle. Keep trying.")
            rospy.sleep(1.0)
      rospy.loginfo("Armed successfully.")

   def run(self):
      rate = rospy.Rate(30.0)

      while not rospy.is_shutdown():
            msg = MotorSetpoint()
            msg.header.stamp = rospy.Time.now()
            # since the bluerov has 8 motors, the setpoint list holds 8 values
            t = rospy.get_time()
            msg.setpoint[0] = 0
            msg.setpoint[1] = 0
            msg.setpoint[2] = 0
            msg.setpoint[3] = 0
            msg.setpoint[4] = 0.2 * math.sin(t)
            msg.setpoint[5] = -0.2 * math.sin(t)
            msg.setpoint[6] = -0.2 * math.sin(t)
            msg.setpoint[7] = 0.2 * math.sin(t)

            self.setpoint_pub.publish(msg)

            rate.sleep()


def main():
   node = MyFirstNode()
   node.run()


if __name__ == "__main__":
   main()
