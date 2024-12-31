#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion ,TransformStamped
import math
from tf2_ros import TransformBroadcaster

def yaw_to_quaternion(yaw):
    """Convertir un angle en quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
        




class OmniRobotOdometry(Node):
    def __init__(self):
        super().__init__('omni_robot_odometry')
        

        # Paramètres du robot
        self.Rw = 0.05  # Rayon des roues (m)
        self.L = 0.1697    # Distance entre le centre et les roues (m)

        # État initial
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Temps précédent
        self.last_time = self.get_clock().now()

        # Abonnements et publications
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        

        #initial odom_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_initial_frame()


    def joint_states_callback(self, msg):
        # Filtrer les vitesses angulaires des roues
        wheel_velocities = {
            joint: velocity
            for joint, velocity in zip(msg.name, msg.velocity)
            if joint in ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint', 'wheel_4_joint']
        }

        if len(wheel_velocities) != 4:
            self.get_logger().warn('Incomplete wheel velocities received!')
            return

        # Extraire les vitesses des roues
        w1 = wheel_velocities['wheel_1_joint']
        w2 = wheel_velocities['wheel_2_joint']
        w3 = wheel_velocities['wheel_3_joint']
        w4 = wheel_velocities['wheel_4_joint']

        # Calcul des vitesses globales
        v_x = (self.Rw / math.sqrt(2)) * (w1 - w2 - w3 + w4) / 4
        v_y = (self.Rw / math.sqrt(2)) * (w1 + w2 + w3 + w4) / 4
        omega_z = (self.Rw / (4 * self.L)) * (w1 + w2 + w3 + w4)

        # Calculer le delta temps
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Intégrer les positions pour obtenir x, y, theta
        self.x += v_x * dt * math.cos(self.theta) - v_y * dt * math.sin(self.theta)
        self.y += v_x * dt * math.sin(self.theta) + v_y * dt * math.cos(self.theta)
        self.theta += omega_z * dt

        # Publier l'odométrie
        self.publish_odometry(v_x, v_y, omega_z, current_time)


        
   

    def publish_odometry(self, v_x, v_y, omega_z, current_time):
        # Publier le message Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quat = yaw_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation = quat

        # Vélocités
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
        odom_msg.twist.twist.angular.z = omega_z

        self.odom_publisher.publish(odom_msg)

        
        # Publier la transformation TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)

        

    def publish_initial_frame(self):
        """Publier une transformation statique initiale entre 'odom' et 'base_link'."""
        t = TransformStamped()

        # Définir le header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Frame parent
        t.child_frame_id = 'base_link'  # Frame enfant

        # Position initiale
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Orientation initiale (aucune rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Envoyer la transformation
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Initial frame odom -> base_link published.')  

        


def main(args=None):
    rclpy.init(args=args)
    node_odom = OmniRobotOdometry()
    rclpy.spin(node_odom)
    


if __name__ == '__main__':
    main()
