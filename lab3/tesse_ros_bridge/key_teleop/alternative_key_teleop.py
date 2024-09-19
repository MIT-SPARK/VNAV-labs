### PRESEVERED FOR REFERENCE ###
### WE CURRENTLY DON'T USE THIS CODE ###
### IT WAS ORIGINALLY IN key_teleop.py ###

# class KeyTeleop(object):

#     _interface = None

#     _linear = None
#     _angular = None

#     def __init__(self, interface):
#         self._interface = interface
#         self._pub_cmd = rospy.Publisher('drive', Twist)

#         self._hz = rospy.get_param('~hz', 10)

#         self._num_steps = rospy.get_param('~turbo/steps', 100)

#         forward_min = rospy.get_param('~turbo/linear_forward_min', 0.5)
#         forward_max = rospy.get_param('~turbo/linear_forward_max', 20)
#         self._forward = Velocity(forward_min, forward_max, self._num_steps)

#         backward_min = rospy.get_param('~turbo/linear_backward_min', 0.25)
#         backward_max = rospy.get_param('~turbo/linear_backward_max', 15)
#         self._backward = Velocity(backward_min, backward_max, self._num_steps)

#         angular_min = rospy.get_param('~turbo/angular_min', 5)
#         angular_max = rospy.get_param('~turbo/angular_max', 7)
#         self._rotation = Velocity(angular_min, angular_max, self._num_steps)

#     def run(self):
#         self._linear = 0
#         self._angular = 0

#         rate = rospy.Rate(self._hz)
#         while True:
#             keycode = self._interface.read_key()
#             if keycode:
#                 if self._key_pressed(keycode):
#                     self._publish()
#             else:
#                 self._publish()
#                 rate.sleep()

#     def _get_twist(self, linear, angular):
#         twist = Twist()
#         if linear >= 0:
#             twist.linear.x = self._forward(1.0, linear)
#         else:
#             twist.linear.x = self._backward(-1.0, -linear)
#         twist.angular.z = self._rotation(math.copysign(1, angular), abs(angular))
#         return twist

#     def _key_pressed(self, keycode):
#         movement_bindings = {
#             ord('w'): ( 1,  0),
#             ord('s'): (-1,  0),
#             ord('d'): ( 0,  1),
#             ord('a'): ( 0, -1),
#         }
#         speed_bindings = {
#             ord(' '): (0, 0),
#         }
#         if keycode in movement_bindings:
#             acc = movement_bindings[keycode]
#             ok = False
#             if acc[0]:
#                 linear = self._linear + acc[0]
#                 if abs(linear) <= self._num_steps:
#                     self._linear = linear
#                     ok = True
#             if acc[1]:
#                 angular = self._angular + acc[1]
#                 if abs(angular) <= self._num_steps:
#                     self._angular = angular
#                     ok = True
#             if not ok:
#                 self._interface.beep()
#         elif keycode in speed_bindings:
#             acc = speed_bindings[keycode]
#             # Note: bounds aren't enforced here!
#             if acc[0] is not None:
#                 self._linear = acc[0]
#             if acc[1] is not None:
#                 self._angular = acc[1]

#         elif keycode == ord('q'):
#             rospy.signal_shutdown('Bye')
#         else:
#             return False

#         return True

#     def _publish(self):
#         self._interface.clear()
#         self._interface.write_line(2, 'Linear: %d, Angular: %d' % (self._linear, self._angular))
#         self._interface.write_line(5, 'Use arrow keys to move, space to stop, q to exit.')
#         self._interface.refresh()

#         twist = self._get_twist(self._linear, self._angular)
#         self._pub_cmd.publish(twist)

# class SimpleKeyTeleopAckermann(SimpleKeyTeleop):
#     def __init__(self, interface):
#         self._interface = interface
#         self._pub_cmd = rospy.Publisher('drive', AckermannDriveStamped)

#         self._hz = rospy.get_param('~hz', 10)

#         self._forward_rate =  rospy.get_param('~forward_rate', 0.8)
#         self._backward_rate = rospy.get_param('~backward_rate', 0.5)
#         self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
#         self._last_pressed = {}
#         self._angular = 0
#         self._linear = 0

#     def _get_ackermann(self, linear, angular):
#         ack = AckermannDriveStamped()
#         ack.header.stamp = rospy.Time.now()
#         ack.drive.speed = linear
#         ack.drive.steering_angle = angular
#         return ack

#     def _publish(self):
#         self._interface.clear()
#         self._interface.write_line(2, 'Linear: %f, Angular: %f' % (self._linear, self._angular))
#         self._interface.write_line(5, 'Use WASD keys to move, q to exit.')
#         self._interface.refresh()

#         ackermann = self._get_ackermann(self._linear, math.radians(self._angular))
#         self._pub_cmd.publish(ackermann)
