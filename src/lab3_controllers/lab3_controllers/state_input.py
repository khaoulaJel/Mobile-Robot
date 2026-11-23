"""Robot state input that accepts the message types the lab robots publish.

Gazebo: nav_msgs/Odometry on /odom.  Real robot (professor's Pi workspace): /diff_cont/odom (Odometry, wheel
odometry); also /odometry/filtered (laptop EKF) and /robot_pose (PoseStamped, SLAM map pose; some stacks use PoseWithCovarianceStamped).
state_type = auto (default) detects the type of the topic, waiting for it to appear;
or force one of: odometry | pose_stamped | pose | pose_cov.

Failures are explicit, never silent (StateInputError, the nodes exit with code 2):
  * the topic does not appear within `wait_timeout` seconds (0 = wait forever),
  * the topic has no supported message type,
  * the topic has several supported types (ambiguous): force one with state_type.
While running it also logs the frame id of the first message, warns when the frame id changes
and warns when the position jumps by more than `jump_warn` metres between two messages
(e.g. a SLAM correction), because such a jump would disturb a path controller.
The callback receives a geometry_msgs/Pose (position + orientation).
"""
import math
import time

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class StateInputError(RuntimeError):
    """The robot state source is missing, unsupported or ambiguous."""


TYPES = {
    'nav_msgs/msg/Odometry': (Odometry, lambda m: m.pose.pose, lambda m: m.header),
    'geometry_msgs/msg/PoseStamped': (PoseStamped, lambda m: m.pose, lambda m: m.header),
    'geometry_msgs/msg/Pose': (Pose, lambda m: m, lambda m: None),
    'geometry_msgs/msg/PoseWithCovarianceStamped': (
        PoseWithCovarianceStamped, lambda m: m.pose.pose, lambda m: m.header),
}
ALIASES = {
    'odometry': 'nav_msgs/msg/Odometry',
    'pose_stamped': 'geometry_msgs/msg/PoseStamped',
    'pose': 'geometry_msgs/msg/Pose',
    'pose_cov': 'geometry_msgs/msg/PoseWithCovarianceStamped',
}


class StateInput:
    def __init__(self, node, topic, state_type, callback, wait_timeout=60.0, jump_warn=0.3):
        self.node, self.topic, self.callback = node, topic, callback
        self.wait_timeout, self.jump_warn = float(wait_timeout), float(jump_warn)
        self.subscription = None
        self.timer = None
        self.type_name = None
        self.frame_id = None
        self.stamp = None
        self.last_xy = None
        self.jumps = 0
        self.t_wait = time.monotonic()
        self._seen, self._seen_count = None, 0
        if state_type != 'auto' and state_type not in ALIASES:
            raise StateInputError(f"state_type must be 'auto' or one of {sorted(ALIASES)}")
        if state_type == 'auto':
            self.timer = node.create_timer(0.5, self._poll)
            self._poll()
        else:
            self._attach(ALIASES[state_type])

    def _attach(self, type_name):
        msg_cls, pose_of, header_of = TYPES[type_name]
        self.type_name = type_name

        def on_msg(m):
            header = header_of(m)
            if header is not None:
                if self.frame_id is None:
                    self.node.get_logger().info(f'state frame_id = "{header.frame_id}"')
                elif header.frame_id != self.frame_id:
                    self.node.get_logger().warn(
                        f'state frame_id changed "{self.frame_id}" -> "{header.frame_id}"')
                self.frame_id = header.frame_id
                self.stamp = header.stamp
            pose = pose_of(m)
            xy = (pose.position.x, pose.position.y)
            if self.last_xy is not None and self.jump_warn > 0:
                jump = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
                if jump > self.jump_warn:
                    self.jumps += 1
                    self.node.get_logger().warn(
                        f'state position jumped {jump:.2f} m between two messages '
                        f'(localization correction?); path following may be disturbed',
                        throttle_duration_sec=2.0)
            self.last_xy = xy
            self.callback(pose)

        self.subscription = self.node.create_subscription(
            msg_cls, self.topic, on_msg, qos_profile_sensor_data)
        self.node.get_logger().info(f'state input: {self.topic} as {type_name}')

    def _poll(self):
        """Auto mode: wait for the topic, require a stable type set, then attach and keep watching.

        DDS discovers publishers one by one, so the type set must be unchanged for two polls before
        it is trusted; and after attaching, a second supported type appearing is still an error.
        """
        found = dict(self.node.get_topic_names_and_types()).get(self.topic)
        if self.subscription is not None:
            supported = [t for t in (found or []) if t in TYPES]
            if len(supported) > 1:
                raise StateInputError(
                    f'{self.topic} became ambiguous, it now has several supported types {supported}; '
                    f'force one with state_type:=odometry|pose_stamped|pose|pose_cov')
            return
        if not found:
            waited = time.monotonic() - self.t_wait
            if self.wait_timeout > 0 and waited > self.wait_timeout:
                raise StateInputError(
                    f'state topic {self.topic} did not appear within {self.wait_timeout:.0f} s '
                    f'(check `ros2 topic list -t` and the state_topic parameter)')
            self.node.get_logger().warn(f'waiting for state topic {self.topic} ...',
                                        throttle_duration_sec=5.0)
            self._seen, self._seen_count = None, 0
            return
        key = tuple(sorted(found))
        self._seen_count = self._seen_count + 1 if key == self._seen else 1
        self._seen = key
        if self._seen_count < 2:
            return
        supported = [t for t in found if t in TYPES]
        if not supported:
            raise StateInputError(
                f'{self.topic} has unsupported message type(s) {found}; '
                f'supported: {sorted(TYPES)}')
        if len(supported) > 1:
            raise StateInputError(
                f'{self.topic} is ambiguous, it has several supported types {supported}; '
                f'force one with state_type:=odometry|pose_stamped|pose|pose_cov')
        self._attach(supported[0])        # the timer keeps running to detect a second type later
