"""Goal-based movement for the Egrobots rover, exposed as a ROS 2 action.

Why an action and not a service
-------------------------------
Driving to a goal takes time, can fail, and the caller wants to watch it happen.
A service is a single request paired with a single response: once called there is
no channel back until it returns. That makes two of this task's requirements
structurally impossible over a service — cancelling a movement already underway
(R5) and reporting progress while it runs (R6). An action adds exactly those:

    goal  ->  feedback, feedback, feedback ...  ->  result
                        ^ and cancellable throughout

The geofence behaviour from the previous task is deliberately left on plain
services. It is an on/off switch that returns immediately, which is precisely
what a service is for. Keeping both side by side is the point: the mechanism
should follow the shape of the work, not fashion.

Threading
---------
The action's execute callback runs a loop until the rover arrives. Under the
default single-threaded executor that loop would monopolise the node and the
scan subscription would never fire, so the rover would drive blind and never
finish. The callbacks below are therefore split across reentrant groups and the
node is spun by a MultiThreadedExecutor (see main()).
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseStamped, PointStamped, TransformStamped, Point
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from tf2_ros import (Buffer, TransformListener, TransformBroadcaster,
                     LookupException, ExtrapolationException, ConnectivityException)
from tf2_geometry_msgs import do_transform_point

from egrobots_rover_interfaces.action import MoveToGoal

TF_ERRORS = (LookupException, ExtrapolationException, ConnectivityException)

CONTROL_PERIOD = 0.1        # 10 Hz, matching the LiDAR update rate


class RoverNode(Node):

    def __init__(self):
        super().__init__('rover_node')

        # --- obstacle avoidance ---
        self.declare_parameter('safe_distance', 1.2)
        self.declare_parameter('clear_distance', 1.5)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('cone_angle_deg', 30.0)
        self.declare_parameter('clearing_distance', 2.0)
        self.declare_parameter('direction_scan_deg', 90.0)

        # --- goal seeking ---
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('heading_tolerance_deg', 30.0)
        self.declare_parameter('goal_approach_distance', 1.0)
        self.declare_parameter('heading_kp', 1.5)

        # --- R7: stall detection ---
        # If the rover fails to get meaningfully closer within this window it is
        # not going to, so the goal is aborted rather than left running forever.
        # This covers a boxed-in rover, one wedged against geometry, and a target
        # that simply cannot be reached — without needing a map to reason about.
        self.declare_parameter('stall_timeout', 12.0)
        self.declare_parameter('stall_min_progress', 0.15)
        self.declare_parameter('goal_timeout', 180.0)

        # --- geofence (service-driven, kept from the previous task) ---
        self.declare_parameter('max_distance_from_origin', 5.0)
        self.declare_parameter('boundary_return_ratio', 0.9)

        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')

        # Shared state, written by callbacks on different threads.
        self._lock = threading.Lock()
        self.latest_scan = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_pose = False

        self.state = 'CRUISING'
        self.turn_direction = 1.0
        self.clear_start_x = 0.0
        self.clear_start_y = 0.0

        self.geofence_enabled = False
        self.returning = False
        self.goal_active = False

        # Perception and the control loop must run concurrently with the action's
        # execute callback, so they get their own reentrant groups.
        scan_group = ReentrantCallbackGroup()
        action_group = ReentrantCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.obstacle_publisher = self.create_publisher(PointStamped, '/detected_obstacle', 10)
        self.marker_publisher = self.create_publisher(Marker, '/geofence_marker', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10,
                                 callback_group=scan_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.1, self.update_pose_from_tf, callback_group=timer_group)
        self.create_timer(1.0, self.publish_geofence_marker, callback_group=timer_group)
        self.create_timer(CONTROL_PERIOD, self.geofence_step, callback_group=timer_group)

        self.create_service(Trigger, 'start_avoidance', self.start_callback)
        self.create_service(Trigger, 'stop_avoidance', self.stop_callback)

        self.action_server = ActionServer(
            self, MoveToGoal, 'move_to_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_group,
        )

        self.get_logger().info(
            'Rover node ready. Send a goal with:\n'
            '  ros2 action send_goal -f /move_to_goal '
            'egrobots_rover_interfaces/action/MoveToGoal '
            '"{target: {x: 5.0, y: 0.0, z: 0.0}, tolerance: 0.0}"\n'
            'Or run the geofence with: ros2 service call /start_avoidance std_srvs/srv/Trigger'
        )

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    def scan_callback(self, msg):
        with self._lock:
            self.latest_scan = msg

    def update_pose_from_tf(self):
        reference_frame = self.get_parameter('reference_frame').value
        robot_frame = self.get_parameter('robot_frame').value
        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, robot_frame, Time())
        except TF_ERRORS as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=5.0)
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        with self._lock:
            self.current_x = t.x
            self.current_y = t.y
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.have_pose = True

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = reference_frame
        pose_msg.pose.position.x = t.x
        pose_msg.pose.position.y = t.y
        pose_msg.pose.orientation = q
        self.pose_publisher.publish(pose_msg)

    def pose(self):
        with self._lock:
            return self.current_x, self.current_y, self.current_yaw, self.have_pose

    def read_cone(self):
        """Closest return within the forward cone, plus the index it came from."""
        with self._lock:
            msg = self.latest_scan
        if msg is None or not msg.ranges:
            return None, float('inf'), None

        cone_size = int(math.radians(self.get_parameter('cone_angle_deg').value)
                        / msg.angle_increment)
        centre = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        low = max(0, centre - cone_size)
        high = min(len(msg.ranges), centre + cone_size)
        indices = [i for i in range(low, high) if msg.ranges[i] > 0.0]
        if not indices:
            return msg, float('inf'), None
        closest_index = min(indices, key=lambda i: msg.ranges[i])
        return msg, msg.ranges[closest_index], closest_index

    def report_obstacle_in_reference_frame(self, msg, index):
        """Express a LiDAR return in the fixed frame rather than the sensor's."""
        reference_frame = self.get_parameter('reference_frame').value
        distance = msg.ranges[index]
        if not math.isfinite(distance):
            return
        bearing = msg.angle_min + index * msg.angle_increment

        point_in_sensor = PointStamped()
        point_in_sensor.header.frame_id = msg.header.frame_id
        point_in_sensor.point.x = distance * math.cos(bearing)
        point_in_sensor.point.y = distance * math.sin(bearing)

        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame, msg.header.frame_id, Time())
        except TF_ERRORS:
            return

        point_in_reference = do_transform_point(point_in_sensor, transform)
        point_in_reference.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_publisher.publish(point_in_reference)

        detection = TransformStamped()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = reference_frame
        detection.child_frame_id = 'detected_obstacle'
        detection.transform.translation.x = point_in_reference.point.x
        detection.transform.translation.y = point_in_reference.point.y
        detection.transform.translation.z = point_in_reference.point.z
        detection.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(detection)

    def publish_geofence_marker(self):
        radius = self.get_parameter('max_distance_from_origin').value
        marker = Marker()
        marker.header.frame_id = self.get_parameter('reference_frame').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'geofence'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.a = 1.0, 0.6, 1.0
        marker.pose.orientation.w = 1.0
        for i in range(73):
            angle = 2.0 * math.pi * i / 72.0
            marker.points.append(Point(x=radius * math.cos(angle),
                                       y=radius * math.sin(angle), z=0.05))
        self.marker_publisher.publish(marker)

    # ------------------------------------------------------------------
    # Shared motion
    # ------------------------------------------------------------------

    def choose_turn_direction(self, msg, centre, scan_deg):
        half = int(math.radians(scan_deg) / msg.angle_increment)
        n = len(msg.ranges)
        cap = lambda vals: [min(r, msg.range_max) for r in vals if r > 0.0]
        left = cap(msg.ranges[centre:min(n, centre + half)])
        right = cap(msg.ranges[max(0, centre - half):centre])
        left_min = min(left) if left else msg.range_max
        right_min = min(right) if right else msg.range_max
        if abs(left_min - right_min) > 0.05:
            return 1.0 if right_min < left_min else -1.0
        left_mean = sum(left) / len(left) if left else msg.range_max
        right_mean = sum(right) / len(right) if right else msg.range_max
        return 1.0 if right_mean < left_mean else -1.0

    def avoidance_step(self, cmd, msg, closest):
        """Run the avoidance state machine. Returns True if it owns the command.

        Obstacle avoidance outranks whatever else is driving: a collision reflex
        must not wait on a goal or a boundary rule, and it reads only the LiDAR
        so it never depends on odometry.
        """
        safe_distance = self.get_parameter('safe_distance').value
        clear_distance = self.get_parameter('clear_distance').value
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        clearing_distance = self.get_parameter('clearing_distance').value
        x, y, _, _ = self.pose()

        if self.state == 'CRUISING' and closest < safe_distance and msg is not None:
            self.state = 'TURNING'
            centre = int(round((0.0 - msg.angle_min) / msg.angle_increment))
            self.turn_direction = self.choose_turn_direction(
                msg, centre, self.get_parameter('direction_scan_deg').value)
            self.get_logger().info(
                f'Obstacle at {closest:.2f} m — turning '
                f'{"left" if self.turn_direction > 0 else "right"}')

        if self.state == 'TURNING':
            if closest > clear_distance:
                self.state = 'CLEARING'
                self.clear_start_x, self.clear_start_y = x, y
                self.get_logger().info(
                    f'Path clear ({closest:.2f} m) — driving '
                    f'{clearing_distance:.2f} m to get past it')
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * angular_speed
                return True

        if self.state == 'CLEARING':
            if closest < safe_distance:
                self.state = 'TURNING'
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * angular_speed
                return True
            if math.hypot(x - self.clear_start_x, y - self.clear_start_y) < clearing_distance:
                cmd.linear.x = linear_speed
                cmd.angular.z = 0.0
                return True
            self.state = 'CRUISING'
            self.get_logger().info('Past the obstacle — resuming')

        return False

    def steer_towards(self, cmd, desired_heading):
        """Pivot until roughly facing the target, then drive straight.

        Committing to a heading and driving straight beats steering continuously:
        rotation is where nearly all of this platform's odometry error comes from.
        """
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        heading_kp = self.get_parameter('heading_kp').value
        tolerance = math.radians(self.get_parameter('heading_tolerance_deg').value)

        _, _, yaw, _ = self.pose()
        heading_error = math.atan2(math.sin(desired_heading - yaw),
                                   math.cos(desired_heading - yaw))
        cmd.angular.z = max(-angular_speed, min(angular_speed, heading_kp * heading_error))
        cmd.linear.x = 0.0 if abs(heading_error) > tolerance else linear_speed
        return heading_error

    def stop(self):
        self.cmd_publisher.publish(Twist())

    # ------------------------------------------------------------------
    # Geofence — service driven (unchanged in spirit from the previous task)
    # ------------------------------------------------------------------

    def start_callback(self, request, response):
        if self.goal_active:
            response.success = False
            response.message = 'A move_to_goal action is running; cancel it first.'
            return response
        self.geofence_enabled = True
        self.state = 'CRUISING'
        self.returning = False
        response.success = True
        response.message = 'Geofence patrol started'
        return response

    def stop_callback(self, request, response):
        self.geofence_enabled = False
        self.stop()
        response.success = True
        response.message = 'Stopped'
        return response

    def geofence_step(self):
        if not self.geofence_enabled or self.goal_active:
            return
        _, _, _, have_pose = self.pose()
        if not have_pose:
            return

        msg, closest, closest_index = self.read_cone()
        if closest_index is not None:
            self.report_obstacle_in_reference_frame(msg, closest_index)

        max_distance = self.get_parameter('max_distance_from_origin').value
        ratio = self.get_parameter('boundary_return_ratio').value
        x, y, _, _ = self.pose()
        distance_from_origin = math.hypot(x, y)

        if self.returning and distance_from_origin < max_distance * ratio:
            self.returning = False
            self.get_logger().info(
                f'Back inside the boundary ({distance_from_origin:.2f} m)')
        elif not self.returning and distance_from_origin > max_distance:
            self.returning = True
            self.get_logger().info(
                f'Crossed the {max_distance:.1f} m boundary — heading back')

        cmd = Twist()
        if self.avoidance_step(cmd, msg, closest):
            self.cmd_publisher.publish(cmd)
            return
        if self.returning:
            self.steer_towards(cmd, math.atan2(-y, -x))
        else:
            cmd.linear.x = self.get_parameter('linear_speed').value
        self.cmd_publisher.publish(cmd)

    # ------------------------------------------------------------------
    # Action — goal driven
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        """R1. Rejecting here is itself an action feature — a service has no
        equivalent of declining the work before starting it."""
        if self.goal_active:
            self.get_logger().warn('Rejecting goal: another goal is already running')
            return GoalResponse.REJECT
        if self.geofence_enabled:
            self.get_logger().warn(
                'Rejecting goal: geofence patrol is active — call /stop_avoidance first')
            return GoalResponse.REJECT
        _, _, _, have_pose = self.pose()
        if not have_pose:
            self.get_logger().warn('Rejecting goal: no pose from TF yet')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Accepted goal: ({goal_request.target.x:.2f}, {goal_request.target.y:.2f})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """R5."""
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """R2/R3/R4/R6/R7 — the loop that drives to the goal."""
        target = goal_handle.request.target
        tolerance = goal_handle.request.tolerance
        if tolerance <= 0.0:
            tolerance = self.get_parameter('goal_tolerance').value

        approach_distance = self.get_parameter('goal_approach_distance').value
        stall_timeout = self.get_parameter('stall_timeout').value
        stall_min_progress = self.get_parameter('stall_min_progress').value
        goal_timeout = self.get_parameter('goal_timeout').value

        self.goal_active = True
        self.state = 'CRUISING'
        started = time.time()
        best_distance = float('inf')
        best_time = started

        result = MoveToGoal.Result()

        try:
            while rclpy.ok():
                x, y, _, _ = self.pose()
                distance = math.hypot(target.x - x, target.y - y)

                # --- R4: arrived ---
                if distance < tolerance:
                    self.stop()
                    goal_handle.succeed()
                    result.success = True
                    result.message = f'Reached the target ({x:.2f}, {y:.2f})'
                    self.get_logger().info(result.message)
                    break

                # --- R5: cancelled ---
                if goal_handle.is_cancel_requested:
                    self.stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled by the client'
                    self.get_logger().info(result.message)
                    break

                # --- R7: stalled or timed out ---
                if distance < best_distance - stall_min_progress:
                    best_distance, best_time = distance, time.time()
                if time.time() - best_time > stall_timeout:
                    self.stop()
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f'Aborted: no progress for {stall_timeout:.0f} s '
                        f'(stuck {distance:.2f} m from the target). '
                        'The target is likely unreachable or the rover is boxed in.')
                    self.get_logger().warn(result.message)
                    break
                if time.time() - started > goal_timeout:
                    self.stop()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Aborted: exceeded {goal_timeout:.0f} s time limit'
                    self.get_logger().warn(result.message)
                    break

                # --- R2/R3: drive, with avoidance taking priority ---
                msg, closest, closest_index = self.read_cone()
                if closest_index is not None:
                    self.report_obstacle_in_reference_frame(msg, closest_index)

                cmd = Twist()
                heading_error = 0.0
                if self.avoidance_step(cmd, msg, closest):
                    phase = self.state
                else:
                    heading_error = self.steer_towards(
                        cmd, math.atan2(target.y - y, target.x - x))
                    phase = 'TURNING' if cmd.linear.x == 0.0 else 'DRIVING'

                # Slow on approach in every phase. CLEARING drives at full speed,
                # so a target inside its clearing run would otherwise be crossed
                # at full tilt and overshot by the stopping distance.
                if cmd.linear.x > 0.0 and distance < approach_distance:
                    cmd.linear.x *= max(0.15, distance / approach_distance)
                self.cmd_publisher.publish(cmd)

                # --- R6: feedback ---
                feedback = MoveToGoal.Feedback()
                feedback.distance_remaining = distance
                feedback.heading_error_deg = math.degrees(heading_error)
                feedback.current_position = Point(x=x, y=y, z=0.0)
                feedback.state = phase
                goal_handle.publish_feedback(feedback)

                time.sleep(CONTROL_PERIOD)
        finally:
            self.goal_active = False
            self.stop()

        x, y, _, _ = self.pose()
        result.final_position = Point(x=x, y=y, z=0.0)
        result.final_distance_error = math.hypot(target.x - x, target.y - y)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RoverNode()
    # Single-threaded spin would let the action's execute loop starve the scan
    # subscription, leaving the rover blind for the whole goal.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
