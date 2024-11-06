# import rclpy
# from rclpy.node import Node
# from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import TransformStamped

# class TF2Echo(Node):
#     def __init__(self):
#         super().__init__('tf2_echo_node')
        
#         # Create a TF2 buffer and listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         # Set the frames you want to monitor
#         self.target_frame = 'link_0'
#         self.source_frame = 'link_ee'
        
#         self.timer = self.create_timer(1.0, self.timer_callback)

#     def timer_callback(self):
#         try:
#             # Lookup the transform from source_frame to target_frame
#             transform: TransformStamped = self.tf_buffer.lookup_transform(
#                 self.target_frame, self.source_frame, rclpy.time.Time())
            
#             # Print the transformation information
#             self.get_logger().info(f'Transform from {self.source_frame} to {self.target_frame}:')
#             self.get_logger().info(f'  Translation: x={transform.transform.translation.x}, '
#                                    f'y={transform.transform.translation.y}, '
#                                    f'z={transform.transform.translation.z}')
#             self.get_logger().info(f'  Rotation: x={transform.transform.rotation.x}, '
#                                    f'y={transform.transform.rotation.y}, '
#                                    f'z={transform.transform.rotation.z}, '
#                                    f'w={transform.transform.rotation.w}')
#         except Exception as e:
#             self.get_logger().error(f'Could not get transform: {str(e)}')

# def main(args=None):
#     rclpy.init(args=args)
#     tf2_echo = TF2Echo()
#     rclpy.spin(tf2_echo)
#     tf2_echo.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

class TF2Echo:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tf2_echo_node', anonymous=True)
        
        # Create a TF listener
        self.listener = tf.TransformListener()
        
        # Set the frames you want to monitor
        self.target_frame = 'lbr_link_0'
        self.source_frame = 'lbr_link_ee'
        
        # Set the timer to call the callback function every second
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def timer_callback(self, event):
        try:
            # Lookup the transform from source_frame to target_frame
            (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
            
            # Print the transformation information
            rospy.loginfo(f'Transform from {self.source_frame} to {self.target_frame}:')
            rospy.loginfo(f'  Translation: x={trans[0]}, y={trans[1]}, z={trans[2]}')
            rospy.loginfo(f'  Rotation: x={rot[0]}, y={rot[1]}, z={rot[2]}, w={rot[3]}')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f'Could not get transform: {str(e)}')

if __name__ == '__main__':
    tf2_echo = TF2Echo()
    rospy.spin()
