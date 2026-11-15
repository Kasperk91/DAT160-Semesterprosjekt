import math
import os
from std_msgs.msg import String
from std_msgs.msg import Bool

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

from .astar import astar
from .map_loader import load_map, world_to_grid, grid_to_world


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # Samme program kan brukes for begge robotene
        self.declare_parameter('robot_name', 'tb3_0')
        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.declare_parameter(
            'map_path',
            os.path.expanduser(
                '~/ros2_ws/src/multi_robot_challenge_23/'
                'maps/map_dat160_w1.yaml'
            )
        )

        map_path = self.get_parameter('map_path').value

        # Les kartet og legg inn 20 cm sikkerhetsmargin
        grid, resolution, self.origin = load_map(
            map_path,
            safety_margin=0.20
        )

        # Reduser kartoppløsningen fra 1 cm til 5 cm
        factor = 5
        h, w = grid.shape

        # En celle er blokkert dersom minst én
        # av de opprinnelige cellene er blokkert
        padded = np.pad(
            grid,
            (
                (0, (-h) % factor),
                (0, (-w) % factor)
            ),
            constant_values=1
        )

        new_h = padded.shape[0] // factor
        new_w = padded.shape[1] // factor

        self.grid = padded.reshape(
            new_h, factor, new_w, factor
        ).max(axis=(1, 3))

        self.resolution = resolution * factor
        self.height = len(self.grid)

        self.scan_sub = self.create_subscription(
            Bool,
            f'/{self.robot_name}/scan_on_arrival',
            self.scan_callback,
            10
        )

        # ROS 2-kommunikasjon
        self.publisher = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/goal_pose',
            self.goal_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            f'/{self.robot_name}/navigation_status',
            10
        )

        # Finn robotens posisjon i kartkoordinater
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # Navigasjonstilstand
        self.path = []
        self.path_index = 0
        self.state = "IDLE"
        self.scan_on_arrival = True
        self.scan_current_goal = True
        self.pending_goal = None

        # Kameraskanning
        self.scan_start_yaw = None
        self.scan_accumulated_angle = 0.0
        self.scan_previous_yaw = None

        # Hvor raskt roboten skal rotere
        self.scan_speed = 0.25

        # Hvor langt roboten skal rotere
        self.scan_target_angle = 2 * math.pi

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            f'PathFollower klar for {self.robot_name}'
        )

    def scan_callback(self, msg):

        self.scan_on_arrival = msg.data

        # Avbryt pågående skanning ved hasteoppdrag
        if not self.scan_on_arrival and self.state == "SCANNING":

            self.stop_robot()
            self.state = "IDLE"

            self.get_logger().info(
                'Kameraskanning avbrutt for hasteoppdrag.'
            )

    def robot_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{self.robot_name}/base_link',
                rclpy.time.Time()
            )

            pos = transform.transform.translation
            rot = transform.transform.rotation

            yaw = math.atan2(
                2 * (rot.w * rot.z + rot.x * rot.y),
                1 - 2 * (rot.y ** 2 + rot.z ** 2)
            )

            return pos.x, pos.y, yaw

        except TransformException:
            return None

    def fail_goal(self, reason):
        """Gi Explorer/Coordinator beskjed, også ved ugyldige mål."""
        self.get_logger().warn(reason)
        self.stop_robot()
        self.state = "IDLE"
        self.path = []
        self.pending_goal = None
        self.status_pub.publish(String(data="FAILED"))

    def goal_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.fail_goal('Målet må være i map-koordinater')
            return

        pose = self.robot_pose()
        if pose is None:
            # Vent med planlegging til Gazebo har publisert TF.
            # Tidligere ble målet bare forkastet, og Explorer ventet for alltid.
            self.pending_goal = msg
            self.stop_robot()
            self.state = "IDLE"
            self.get_logger().warn('Venter på robotposisjon før ruteplanlegging')
            return

        self.pending_goal = None
        x, y, _ = pose
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        # Viktig: IKKE sett scan_on_arrival her. Denne verdien
        # styres av Coordinator på /<robot>/scan_on_arrival.
        self.scan_current_goal = bool(self.scan_on_arrival)

        start = world_to_grid(
            x, y, self.resolution, self.origin, self.height
        )
        goal = world_to_grid(
            goal_x, goal_y, self.resolution, self.origin, self.height
        )

        self.stop_robot()
        self.state = "IDLE"
        rows, cols = self.grid.shape

        for point in (start, goal):
            row, col = point
            if not (0 <= row < rows and 0 <= col < cols):
                self.fail_goal('Start eller mål er utenfor kartet')
                return
            if self.grid[row, col] != 0:
                self.fail_goal('Start eller mål er blokkert')
                return

        self.get_logger().info(
            f'Planlegger rute til ({goal_x:.2f}, {goal_y:.2f})'
        )
        grid_path = astar(self.grid, start, goal)
        if grid_path is None:
            self.fail_goal('Fant ingen rute')
            return

        grid_path = self.simplify_path(grid_path)
        self.path = [
            grid_to_world(row, col, self.resolution, self.origin, self.height)
            for row, col in grid_path[1:]
        ]
        self.path_index = 0
        self.state = "NAVIGATING"
        self.status_pub.publish(String(data="NAVIGATING"))
        self.get_logger().info(
            f'Rute funnet: {len(self.path)} vendepunkter; '
            f'skann ved ankomst: {self.scan_current_goal}'
        )

    def line_is_free(self, a, b):

        # Undersøk cellene langs en rett linje
        dr = b[0] - a[0]
        dc = b[1] - a[1]

        steps = max(abs(dr), abs(dc)) * 2 + 1

        if steps == 1:
            return self.grid[a] == 0

        for i in range(steps + 1):
            t = i / steps

            row = round(a[0] + dr * t)
            col = round(a[1] + dc * t)

            if self.grid[row, col] != 0:
                return False

        return True

    def simplify_path(self, path):

        if len(path) < 3:
            return path

        result = [path[0]]
        index = 0

        while index < len(path) - 1:

            # Se opptil 50 kartceller fremover
            farthest = min(index + 50, len(path) - 1)

            while farthest > index + 1:
                if self.line_is_free(
                    path[index], path[farthest]
                ):
                    break

                farthest -= 1

            result.append(path[farthest])
            index = farthest

        return result

    def control_loop(self):

        # Ta igjen mål som kom før TF var tilgjengelig.
        if self.pending_goal is not None:
            if self.robot_pose() is None:
                return
            msg = self.pending_goal
            self.pending_goal = None
            self.goal_callback(msg)

        if self.state not in ("NAVIGATING", "SCANNING"):
            return

        pose = self.robot_pose()

        if pose is None:
            self.stop_robot()
            return

        x, y, yaw = pose

        # Utfør kameraskanning
        if self.state == "SCANNING":

            delta = math.atan2(
                math.sin(yaw - self.scan_previous_yaw),
                math.cos(yaw - self.scan_previous_yaw)
            )

            self.scan_accumulated_angle += abs(delta)
            self.scan_previous_yaw = yaw

            if self.scan_accumulated_angle >= self.scan_target_angle:

                self.stop_robot()
                self.state = "IDLE"

                self.status_pub.publish(
                    String(data="REACHED")
                )

                self.get_logger().info(
                    'Kameraskanning ferdig. Klar for neste mål.'
                )

                return

            cmd = Twist()
            cmd.angular.z = self.scan_speed
            self.publisher.publish(cmd)

            return

        if self.path_index >= len(self.path):

            self.stop_robot()

            if self.scan_current_goal:

                self.state = "SCANNING"
                self.scan_previous_yaw = yaw
                self.scan_accumulated_angle = 0.0

                self.get_logger().info(
                    'Starter kameraskanning.'
                )

            else:

                self.state = "IDLE"

                self.status_pub.publish(
                    String(data="REACHED")
                )

                self.get_logger().info(
                    'Målet er nådd uten skanning.'
                )

            return

        target_x, target_y = self.path[self.path_index]

        dx = target_x - x
        dy = target_y - y

        distance = math.hypot(dx, dy)

        # Gå til neste punkt når vi er nær nok
        if distance < 0.12:
            self.path_index += 1
            self.stop_robot()
            return

        desired_angle = math.atan2(dy, dx)

        angle_error = math.atan2(
            math.sin(desired_angle - yaw),
            math.cos(desired_angle - yaw)
        )

        cmd = Twist()

        # Roter først, kjør deretter rett frem
        if abs(angle_error) > 0.12:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -0.6,
                min(0.6, 1.5 * angle_error)
            )

        else:
            cmd.linear.x = min(0.20, distance)
            cmd.angular.z = 0.0

        self.publisher.publish(cmd)

    def stop_robot(self):
        self.publisher.publish(Twist())


def main(args=None):

    rclpy.init(args=args)

    node = PathFollower()

    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()