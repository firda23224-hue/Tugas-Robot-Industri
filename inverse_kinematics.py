
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class InverseKinematics(Node):

    def __init__(self, linear_velocity, angular_velocity):
        super().__init__('inverse_kinematics')

        
        # Parameter robot
        
        self.wheel_radius = 0.03
        self.wheel_separation = 0.17

        # Input kecepatan robot
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        # Publisher ke Gazebo melalui /cmd_vel
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Hitung inverse kinematics
        self.inverse_kinematics()

    def inverse_kinematics(self):

        r = self.wheel_radius
        s = self.wheel_separation

        # Matriks Forward Kinematics


        kinematics_matrix = np.array([
            [r / 2,  r / 2],
            [-r / s, r / s]
        ])

        
        # Inverse matriks

        inverse_matrix = np.linalg.inv(kinematics_matrix)

        velocity_vector = np.array([
            self.linear_velocity,
            self.angular_velocity
        ])

        
        # Inverse Kinematics
        
        wheel_velocity = inverse_matrix @ velocity_vector

        # Kecepatan angular roda
        phi_left = wheel_velocity[0]
        phi_right = wheel_velocity[1]

        
        # Tampilkan hasil IK
        

        self.get_logger().info(
            f'Input: V = {self.linear_velocity:.3f} m/s, '
            f'omega = {self.angular_velocity:.3f} rad/s'
        )

        self.get_logger().info(
            f'phi_L = {phi_left:.3f} rad/s'
        )

        self.get_logger().info(
            f'phi_R = {phi_right:.3f} rad/s'
        )

        # Kirim perintah ke Gazebo
        # melalui topic /cmd_vel

        cmd = Twist()

        cmd.linear.x = self.linear_velocity
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.angular_velocity

        self.cmd_vel_publisher.publish(cmd)

        self.get_logger().info(
            'Perintah dikirim ke /cmd_vel'
        )


def main(args=None):

    rclpy.init(args=args)


    if len(sys.argv) != 3:

        print(
            '\nCara menjalankan:\n'
            'ros2 run robin_bringup inverse_kinematics '
            '<linear_velocity> <angular_velocity>\n\n'
            'Contoh:\n'
            'ros2 run robin_bringup inverse_kinematics 0.1 0.0\n'
        )

        rclpy.shutdown()
        return

    
    # Ambil input

    try:

        linear_velocity = float(sys.argv[1])
        angular_velocity = float(sys.argv[2])

    except ValueError:

        print('Input harus berupa angka.')

        rclpy.shutdown()
        return


    # Jalankan node IK

    node = InverseKinematics(
        linear_velocity,
        angular_velocity
    )

    # Biarkan publisher mengirim pesan
    rclpy.spin_once(
        node,
        timeout_sec=0.1
    )

    # Shutdown

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
