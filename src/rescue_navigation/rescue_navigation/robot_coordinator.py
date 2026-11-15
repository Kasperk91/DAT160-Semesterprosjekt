import os
import math

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException
)

from rescue_interfaces.msg import MarkerFound

from .map_loader import (
    load_map,
    world_to_grid,
    grid_to_world
)


class RobotCoordinator(Node):

    def __init__(self):
        super().__init__('robot_coordinator')

        self.robots = ['tb3_0', 'tb3_1']

        # Last inn samme kart som navigasjonsprogrammet
        self.declare_parameter(
            'map_path',
            os.path.expanduser(
                '~/ros2_ws/src/multi_robot_challenge_23/'
                'maps/map_dat160_w1.yaml'
            )
        )

        map_path = self.get_parameter('map_path').value

        grid, resolution, self.origin = load_map(
            map_path,
            safety_margin=0.20
        )

        # 5 cm per kartcelle
        factor = 5

        h, w = grid.shape

        padded = np.pad(
            grid,
            ((0, (-h) % factor), (0, (-w) % factor)),
            constant_values=1
        )

        self.grid = padded.reshape(
            padded.shape[0] // factor,
            factor,
            padded.shape[1] // factor,
            factor
        ).max(axis=(1, 3))

        self.resolution = resolution * factor
        self.height, self.width = self.grid.shape

        # Sammenhengende ledige områder
        _, self.labels = cv2.connectedComponents(
            (self.grid == 0).astype(np.uint8),
            connectivity=4
        )

        # ROS-transformasjoner
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # Motta markørfunn
        self.marker_sub = self.create_subscription(
            MarkerFound,
            '/rescue/marker_found',
            self.marker_callback,
            10
        )

        # Publiser navigasjonsmål
        self.goal_publishers = {}

        # Kontroller utforskingen
        self.pause_publishers = {}

        self.scan_publishers = {}

        for robot in self.robots:

            self.goal_publishers[robot] = (
                self.create_publisher(
                    PoseStamped,
                    f'/{robot}/goal_pose',
                    10
                )
            )

            self.scan_publishers[robot] = self.create_publisher(
                Bool,
                f'/{robot}/scan_on_arrival',
                10
            )

            self.pause_publishers[robot] = (
                self.create_publisher(
                    Bool,
                    f'/{robot}/exploration_pause',
                    10
                )
            )

        self.found_markers = set()

        self.big_fire_position = None
        self.big_fire_active = False
        self.big_fire_completed = False

        self.goals_sent = False
        self.pause_ticks = 0
        self.failed_goals = set()
        self.goal_history = {robot: [] for robot in self.robots}
        self.retries = {robot: 0 for robot in self.robots}
        self.status_subscriptions = []
        for robot in self.robots:
            self.status_subscriptions.append(self.create_subscription(
                String,
                f'/{robot}/navigation_status',
                lambda msg, r=robot: self.status_callback(r, msg),
                10
            ))

        self.timer = self.create_timer(
            0.5,
            self.control_loop
        )

        self.get_logger().info(
            'RobotCoordinator startet!'
        )

    def robot_position(self, robot):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{robot}/base_link',
                Time()
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            )

        except TransformException:
            return None

    def status_callback(self, robot, msg):

        if not self.big_fire_active or not self.goals_sent:
            return

        if msg.data == 'FAILED':
            self.failed_goals.add(robot)

            self.get_logger().warn(
                f'{robot}: brannmål mislyktes, forsøker alternativ posisjon'
            )

        elif msg.data == 'REACHED':

            position = self.robot_position(robot)

            if position is None:
                return

            distance = math.dist(
                position,
                self.big_fire_position
            )

            if distance >= 1.8:

                self.get_logger().warn(
                    f'{robot}: nådde målet, men er '
                    f'{distance:.2f} m fra brannen. '
                    'Forsøker alternativ posisjon.'
                )

                self.failed_goals.add(robot)

    def marker_callback(self, msg):

        marker_id = msg.marker_id

        if marker_id in self.found_markers:
            return

        self.found_markers.add(marker_id)

        self.get_logger().info(
            f'Markør {marker_id} funnet!'
        )

        # Stor brann
        if marker_id == 4:

            self.big_fire_position = (
                msg.position.x,
                msg.position.y,
                msg.position.z
            )

            self.big_fire_active = True

            self.get_logger().info(
                'STOR BRANN! Begge roboter til området!'
            )

    def pause_exploration(self, pause):

        for robot in self.robots:

            # Pause vanlig utforsking
            self.pause_publishers[robot].publish(
                Bool(data=pause)
            )

            # Ved pause: slå av skanning
            # Ved gjenopptakelse: slå på skanning
            self.scan_publishers[robot].publish(
                Bool(data=not pause)
            )

    def find_safe_goal(self, robot, fire_position):

        position = self.robot_position(robot)

        if position is None:
            return None

        start = world_to_grid(
            position[0],
            position[1],
            self.resolution,
            self.origin,
            self.height
        )

        row, col = start

        if not (
            0 <= row < self.height
            and 0 <= col < self.width
        ):
            return None

        component = self.labels[row, col]

        if component == 0:
            return None

        # Finn ledige kartceller innenfor 1,4 m
        # fra den store brannen.
        cells = np.argwhere(
            self.labels == component
        )

        candidates = []

        for r, c in cells:

            x, y = grid_to_world(
                int(r),
                int(c),
                self.resolution,
                self.origin,
                self.height
            )

            distance_to_fire = math.hypot(
                x - fire_position[0],
                y - fire_position[1]
            )

            # Hold god avstandsmargin, og ikke forsøk samme mål igjen.
            if distance_to_fire < 1.4 and all(
                math.hypot(x - old_x, y - old_y) > 0.45
                for old_x, old_y in self.goal_history[robot]
            ):

                distance_to_robot = math.hypot(
                    x - position[0],
                    y - position[1]
                )

                candidates.append(
                    (distance_to_robot, x, y)
                )

        if not candidates:
            return None

        # Velg nærmeste tilgjengelige kandidat
        candidates.sort()

        return candidates[0][1], candidates[0][2]

    def send_goal(self, robot, x, y):

        goal = PoseStamped()

        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.goal_publishers[robot].publish(goal)

        self.get_logger().info(
            f'{robot} sendt til ({x:.2f}, {y:.2f})'
        )

    def control_loop(self):

        if self.big_fire_completed:
            return

        if not self.big_fire_active:
            return

        # Sett vanlig utforsking på pause
        self.pause_exploration(True)

        # Gi Explorer tid til å motta pausemeldingen
        if not self.goals_sent:

            self.pause_ticks += 1

            if self.pause_ticks < 4:
                return

            # Sørg for at begge robotposisjoner finnes
            if any(
                self.robot_position(robot) is None
                for robot in self.robots
            ):
                return

            goals = {}

            for robot in self.robots:

                goal = self.find_safe_goal(
                    robot,
                    self.big_fire_position
                )

                if goal is None:
                    self.get_logger().warn(
                        f'Fant ikke egnet mål for {robot}'
                    )
                    return

                goals[robot] = goal

            for robot, goal in goals.items():

                self.goal_history[robot].append(goal)
                self.send_goal(robot, goal[0], goal[1])

            self.goals_sent = True

        # Hvis en rute mislykkes, velg en annen ledig posisjon.
        for robot in list(self.failed_goals):
            self.failed_goals.discard(robot)
            if self.retries[robot] >= 3:
                self.get_logger().error(
                    f'{robot}: ingen flere forsøk på brannoppdraget. '
                    'Gjenopptar vanlig utforsking.'
                )
                self.big_fire_active = False
                self.pause_exploration(False)
                return

            alternative = self.find_safe_goal(robot, self.big_fire_position)
            if alternative is None:
                self.get_logger().error(
                    f'{robot}: ingen alternativ brannposisjon. '
                    'Gjenopptar vanlig utforsking.'
                )
                self.big_fire_active = False
                self.pause_exploration(False)
                return
            self.retries[robot] += 1
            self.goal_history[robot].append(alternative)
            self.send_goal(robot, *alternative)

        # Undersøk om begge er nær brannen
        robots_near_fire = 0

        for robot in self.robots:

            position = self.robot_position(robot)

            if position is None:
                continue

            # Poengsystemet bruker 3D-avstand, ikke bare kartets X/Y.
            distance = math.dist(position, self.big_fire_position)

            # Litt margin innenfor konkurransens 2 m
            if distance < 1.8:
                robots_near_fire += 1

        if robots_near_fire == 2:

            self.get_logger().info(
                'Begge roboter ved den store brannen!'
            )

            self.big_fire_completed = True
            self.big_fire_active = False

            # Gjenoppta vanlig utforsking
            self.pause_exploration(False)

            self.get_logger().info(
                'Utforsking kan fortsette!'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RobotCoordinator()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()