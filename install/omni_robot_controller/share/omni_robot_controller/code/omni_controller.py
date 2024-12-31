import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class OmniRobotController(Node):
    def __init__(self):
        super().__init__('omnirobot_controller')

        # Définition des paramètres physiques du robot
        self.L = 0.125  # Distance du centre du robot à chaque roue (en mètres)
        self.Rw = 0.03  # Rayon des roues (en mètres)

        # Création du publisher pour les vitesses des roues
        self.publisher_ = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)

        # Création du subscriber pour le topic /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Initialisation des vitesses des roues
        self.wheel_vel = np.array([0.0, 0.0, 0.0, 0.0], float)

    def cmd_vel_callback(self, msg):
        """
        Callback exécuté lorsque des données sont reçues sur /cmd_vel.
        Calcule les vitesses des roues en fonction des commandes de translation et rotation.
        """
        # Extraire les vitesses linéaires et angulaires du message Twist
        vel_x = msg.linear.x  # Vitesse linéaire en x
        vel_y = msg.linear.y  # Vitesse linéaire en y
        vel_w = msg.angular.z  # Vitesse angulaire autour de z

        # Calcul des vitesses des roues via la cinématique inverse
        self.wheel_vel[0] = (vel_x * math.sin(math.pi / 4)            + vel_y * math.cos(math.pi / 4)            + self.L * vel_w) / self.Rw
        self.wheel_vel[1] = (vel_x * math.sin(math.pi / 4 + math.pi / 2) + vel_y * math.cos(math.pi / 4 + math.pi / 2) + self.L * vel_w) / self.Rw
        self.wheel_vel[2] = (vel_x * math.sin(math.pi / 4 - math.pi)   + vel_y * math.cos(math.pi / 4 - math.pi)   + self.L * vel_w) / self.Rw
        self.wheel_vel[3] = (vel_x * math.sin(math.pi / 4 - math.pi / 2) + vel_y * math.cos(math.pi / 4 - math.pi / 2) + self.L * vel_w) / self.Rw

        # Publier les vitesses des roues
        array_for_publish = Float64MultiArray(data=self.wheel_vel)
        self.publisher_.publish(array_for_publish)

        self.get_logger().info(f'Wheel velocities: {self.wheel_vel}')


def main(args=None):
    rclpy.init(args=args)

    # Lancer le contrôleur
    omnirobot_controller = OmniRobotController()

    try:
        rclpy.spin(omnirobot_controller)
    except KeyboardInterrupt:
        pass

    # Arrêter proprement le nœud
    omnirobot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
