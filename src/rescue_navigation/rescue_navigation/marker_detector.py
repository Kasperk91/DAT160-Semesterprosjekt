import rclpy
import time

from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped
from rescue_interfaces.msg import MarkerFound

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition


class MarkerDetector(Node):

    def __init__(self):
        super().__init__('marker_detector')

        self.declare_parameter('robot_name', 'tb3_0')
        self.robot_name = self.get_parameter('robot_name').value

        # OpenCV returnerer markørposisjoner i kameraets OPTISKE ramme.
        # Gazebo-bildenes header kan feilaktig stå som 'base_footprint'.
        # Det er ikke den rammen vi kan transformere OpenCV-posen fra.
        self.declare_parameter(
            'camera_optical_frame',
            f'{self.robot_name}/camera_rgb_optical_frame'
        )
        self.camera_optical_frame = self.get_parameter(
            'camera_optical_frame'
        ).value
        self.last_warning = {}

        # Markører som er godkjent av poengsystemet
        self.reported = set()

        # Markører som venter på svar fra poengsystemet
        self.pending = set()

        # Transformasjoner mellom kamera og kart
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # Motta ArUco-markører
        self.subscription = self.create_subscription(
            ArucoMarkers,
            f'/{self.robot_name}/aruco_markers',
            self.marker_callback,
            10
        )

        # Rapporter markører til poengsystemet
        self.scoring_client = self.create_client(
            SetMarkerPosition,
            '/set_marker_position'
        )

        # Publiser godkjente funn til koordinatoren
        self.found_pub = self.create_publisher(
            MarkerFound,
            '/rescue/marker_found',
            10
        )

        self.get_logger().info(
            f'MarkerDetector startet for {self.robot_name}'
        )

    def warn_limited(self, key, message, seconds=5.0):
        now = time.monotonic()
        if now - self.last_warning.get(key, -1e12) >= seconds:
            self.get_logger().warn(message)
            self.last_warning[key] = now

    def marker_callback(self, msg):

        if len(msg.marker_ids) > 0:

            self.get_logger().info(
                f'ARUCO OPPDAGET! ID: {list(msg.marker_ids)}'
            )

        if not self.scoring_client.service_is_ready():
            self.warn_limited('scoring',
                'Poengsystemet /set_marker_position er ikke tilgjengelig')
            return

        for marker_id, pose in zip(msg.marker_ids, msg.poses):

            marker_id = int(marker_id)

            if marker_id < 0 or marker_id > 4:
                continue

            if marker_id in self.reported:
                continue

            if marker_id in self.pending:
                continue

            # Markørens posisjon relativt til kameraet
            point = PointStamped()
            point.header.stamp = msg.header.stamp
            # Ikke stol på at bildeheaderen navngir den optiske rammen.
            point.header.frame_id = self.camera_optical_frame
            point.point = pose.position

            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    self.camera_optical_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)
                )

                # Oversett til kartkoordinater
                map_point = do_transform_point(
                    point,
                    transform
                )

            except TransformException as error:
                self.warn_limited(
                    f'tf_{marker_id}',
                    f'Markør {marker_id} sett, men TF mangler: '
                    f'map <- {self.camera_optical_frame}: {error}'
                )
                continue

            # Send rapport til poengsystemet
            request = SetMarkerPosition.Request()

            request.marker_id = marker_id
            request.marker_position = map_point.point

            self.pending.add(marker_id)
            self.get_logger().info(
                f'Rapporterer markør {marker_id}: '
                f'({map_point.point.x:.2f}, '
                f'{map_point.point.y:.2f}, '
                f'{map_point.point.z:.2f}) i map'
            )

            future = self.scoring_client.call_async(request)

            future.add_done_callback(
                lambda f, mid=marker_id, pos=map_point.point:
                self.report_result(f, mid, pos)
            )

    def report_result(self, future, marker_id, position):

        self.pending.discard(marker_id)

        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Feil ved rapportering: {error}'
            )
            return

        if response.accepted:

            self.reported.add(marker_id)

            self.get_logger().info(
                f'Markør {marker_id} godkjent av poengsystemet!'
            )

            found = MarkerFound()

            found.marker_id = marker_id
            found.position = position

            self.found_pub.publish(found)

        else:
            self.get_logger().warn(
                f'Markør {marker_id} ble ikke godkjent: posisjon '
                f'({position.x:.2f}, {position.y:.2f}, {position.z:.2f})'
            )


def main(args=None):

    rclpy.init(args=args)

    node = MarkerDetector()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()