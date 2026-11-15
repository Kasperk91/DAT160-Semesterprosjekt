import os
import math

import cv2
import numpy as np
import rclpy

from .visibility_planner import plan_scan_points
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from tf2_ros import Buffer, TransformListener, TransformException

from .map_loader import load_map, world_to_grid, grid_to_world


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        self.declare_parameter('robot_name', 'tb3_0')
        self.robot_name = self.get_parameter('robot_name').value

        self.declare_parameter(
            'map_path',
            os.path.expanduser(
                '~/ros2_ws/src/multi_robot_challenge_23/'
                'maps/map_dat160_w1.yaml'
            )
        )

        map_path = self.get_parameter('map_path').value

        self.declare_parameter('split_y', 0.0)
        self.declare_parameter('lower_robot', 'tb3_0')

        # Last inn kart med sikkerhetsmargin
        grid, resolution, self.origin = load_map(
            map_path, safety_margin=0.20
        )

        # Lag et grovere rutenett: 10 cm per celle
        factor = 10
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

        # Kartets sammenhengende, ledige områder
        _, self.labels = cv2.connectedComponents(
            (self.grid == 0).astype(np.uint8),
            connectivity=4
        )

        self.candidates = None
        self.visited = set()
        self.waiting = False
        self.finished = False
        self.paused = False
        self.current_target = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/goal_pose',
            10
        )

        self.status_sub = self.create_subscription(
            String,
            f'/{self.robot_name}/navigation_status',
            self.status_callback,
            10
        )

        self.pause_sub = self.create_subscription(
            Bool,
            f'/{self.robot_name}/exploration_pause',
            self.pause_callback,
            10
        )

        # Velg neste mål når roboten er klar
        self.timer = self.create_timer(
            1.0,
            self.explore
        )

        self.get_logger().info(
            f'Explorer startet for {self.robot_name}'
        )

    def robot_position(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{self.robot_name}/base_link',
                Time()
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

        except TransformException:
            return None

    def create_candidates(self, position):

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
            return False

        component = self.labels[row, col]

        if component == 0:
            self.get_logger().warn(
                'Roboten står i en blokkert kartcelle'
            )
            return False

        self.candidates = []

        # Velg skannepunkter ut fra synlige vegger
        self.candidates = plan_scan_points(
            grid=self.grid,
            labels=self.labels,
            component=component,
            resolution=self.resolution,
            origin=self.origin,
            height=self.height,
            robot_name=self.robot_name,
            split_y=self.get_parameter('split_y').value,
            lower_robot=self.get_parameter('lower_robot').value
        )

        self.get_logger().info(
            f'Valgte {len(self.candidates)} skannepunkter '
            f'basert på synlighet'
        )

        return True

    def status_callback(self, msg):

        if self.paused or not self.waiting:
            return

        if msg.data == "REACHED":
            self.get_logger().info('Område undersøkt')

            self.visited.add(self.current_target)
            self.waiting = False

        elif msg.data == "FAILED":
            self.get_logger().warn(
                'Kunne ikke nå området. Velger et annet.'
            )

            self.visited.add(self.current_target)
            self.waiting = False

    def pause_callback(self, msg):

        if msg.data and not self.paused:
            self.get_logger().info(
                'Utforsking satt på pause'
            )

            self.paused = True
            self.waiting = False
            self.current_target = None

        elif not msg.data and self.paused:
            self.get_logger().info(
                'Utforsking fortsetter'
            )

            self.paused = False

    def explore(self):

        if self.paused or self.waiting or self.finished:
            return

        position = self.robot_position()

        if position is None:
            return

        if self.candidates is None:
            if not self.create_candidates(position):
                return

        # Finn områder som ikke er besøkt
        remaining = [
            point for point in self.candidates
            if point not in self.visited
        ]

        if not remaining:
            self.get_logger().info(
                'Alle planlagte områder er behandlet!'
            )
            self.finished = True
            return

        # Velg det nærmeste området
        target = min(
            remaining,
            key=lambda p: math.hypot(
                p[0] - position[0],
                p[1] - position[1]
            )
        )

        self.current_target = target
        self.waiting = True

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        self.get_logger().info(
            f'Nytt mål: ({target[0]:.2f}, {target[1]:.2f})'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Explorer()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()