#!/usr/bin/env python
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32


def pressure_callback(pressure_msg, publisher):
   pascal_per_meter = 1.0e4
   depth = -pressure_msg.fluid_pressure / pascal_per_meter + rospy.get_param('~offset')
   depth_msg = Float32()
   depth_msg.data = depth
   publisher.publish(depth_msg)


def main():
   rospy.init_node("depth_calculator")
   depth_pub = rospy.Publisher("depth", Float32, queue_size=1)
   pressure_sub = rospy.Subscriber("pressure", FluidPressure,
                                 pressure_callback, depth_pub)
   rospy.spin()


if __name__ == "__main__":
   main()
