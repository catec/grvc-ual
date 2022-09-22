#!/usr/bin/env python

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
##

import rospy
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink

"""
A simple buffer
"""
class fifo(object):
   def __init__(self):
      self.buf = []
   def write(self, data):
      self.buf += data
      return len(data)
   def read(self):
      return self.buf.pop(0)

"""
Send a mavlink message
"""
def send_message(msg, mav, pub):
   msg.pack(mav)
   rosmsg = convert_to_rosmsg(msg)
   pub.publish(rosmsg)

   print("sent message %s" % msg)

"""
Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
to use local position information without a GPS.
"""
def set_global_origin(mav, pub, lat, lon, alt):
   target_system = mav.srcSystem
   #target_system = 0   # 0 --> broadcast to everyone
   lattitude = lat
   longitude = lon
   altitude  = alt

   msg = MAV_APM.MAVLink_set_gps_global_origin_message(
         target_system,
         lattitude,
         longitude,
         altitude)

   send_message(msg, mav, pub)

"""
Send a mavlink SET_HOME_POSITION message, which should allow
us to use local position information without a GPS
"""
def set_home_position(mav, pub, lat, lon, alt):
   target_system = mav.srcSystem
   #target_system = 0  # broadcast to everyone

   lattitude = lat
   longitude = lon
   altitude  = alt

   x = 0
   y = 0
   z = 0
   q = [1, 0, 0, 0]   # w x y z

   approach_x = 0
   approach_y = 0
   approach_z = 1

   msg = MAV_APM.MAVLink_set_home_position_message(
         target_system,
         lattitude,
         longitude,
         altitude,
         x, y, z,
         q,
         approach_x,
         approach_y,
         approach_z)

   send_message(msg, mav, pub)

if __name__ == "__main__":
   try:
      rospy.init_node("origin_publisher")
      mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)

      # Set up mavlink instance
      f = fifo()
      mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

      # Global position of the origin
      origin_lat = rospy.get_param('origin_lat', 374318092)
      origin_lon = rospy.get_param('origin_lon', -5.8592880)
      origin_alt = rospy.get_param('origin_alt', 163000)

      # wait to initialize
      while mavlink_pub.get_num_connections() <= 0:
         rospy.loginfo_throttle(1, "Waiting for mavros connection")
         pass

      for _ in range(2):
         rospy.sleep(1)
         set_global_origin(mav, mavlink_pub, origin_lat, origin_lon, origin_alt)
         set_home_position(mav, mavlink_pub, origin_lat, origin_lon, origin_alt)

   except rospy.ROSInterruptException:
      pass

