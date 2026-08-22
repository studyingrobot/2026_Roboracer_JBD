import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
# minjae changes
from rclpy.parameter import Parameter
# minjae changes
from rclpy.time import Time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
# minjae changes
from rcl_interfaces.msg import SetParametersResult
# minjae changes
# minjae changes
# /auto 는 AckermannDriveStamped 로 발행하므로 Float64(ERPM/서보)는 쓰지 않는다.
# Bool = 로컬 플래너 AEB, Float32 = 플래너 곡률 속도상한.
from std_msgs.msg import Bool, Float32
# minjae changes
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class MinjaePPNode(Node):
    def __init__(self):
        super().__init__('minjae_pp_node')

        self.declare_parameter('drive_mode', 'sim')
        self.declare_parameter('enabled', False)

        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/planning/path')
        # minjae changes
        # 실차에서 VESC 로 직접 쏘지 않고 Ackermann mux(/auto)를 거친다.
        # sim 은 /drive, real 은 /auto 를 launch 가 넣어준다.
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('speed_limit_topic', '/planning/speed_limit')
        self.declare_parameter(
            'emergency_stop_topic', '/safety/emergency_stop')
        self.declare_parameter('speed_limit_timeout', 0.50)
        self.declare_parameter('use_dynamic_speed_limit', True)
        self.declare_parameter('min_command_speed', 0.0)
        # minjae changes

        self.declare_parameter('wheelbase', 0.33)

        # minjae changes
        # self.declare_parameter('lookahead_distance', 0.70)
        # minjae changes

        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_path_distance', 1.00)
        self.declare_parameter('max_heading_error', 1.0472)
        self.declare_parameter('search_back_points', 8)
        self.declare_parameter('search_forward_points', 30)

        self.declare_parameter('target_speed', 0.60)
        self.declare_parameter('min_speed', 0.25)
        self.declare_parameter('max_speed', 0.80)
        self.declare_parameter('corner_slowdown_gain', 0.55)


        # minjae changes
        # Speed-proportional lookahead: L = clamp(time * v, min, max)
        self.declare_parameter('lookahead_time', 0.30)
        self.declare_parameter('lookahead_min', 0.35)
        self.declare_parameter('lookahead_max', 1.20)

        # Curvature speed profile: v = sqrt(a_y / kappa), then a backward
        # pass so every corner is reachable at its own required speed.
        self.declare_parameter('max_lateral_acceleration', 4.0)
        self.declare_parameter('curvature_stride', 0.20)
        self.declare_parameter('speed_profile_deceleration', 2.0)
        # minjae changes

        
        # minjae changes
        # ERPM/서보 변환은 차량의 ackermann_to_vesc 가 담당하므로 제거.
        # (speed_to_erpm_gain, servo_center 등 6개 파라미터 삭제)
        # minjae changes

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('odom_timeout', 0.50)
        self.declare_parameter('path_timeout', 2.00)

        self.drive_mode = self.get_parameter('drive_mode').value
        self.enabled = bool(self.get_parameter('enabled').value)
        # minjae changes
        # set_parameters() 가 on_set_parameters 를 다시 부르는데,
        # 그때 우리 자신의 동기화 쓰기를 외부 start 요청으로 오인해
        # 거부하지 않도록 막는 플래그.
        self._syncing_enabled_param = False
        # minjae changes

        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        # minjae changes
        self.drive_topic = self.get_parameter('drive_topic').value
        self.speed_limit_topic = self.get_parameter('speed_limit_topic').value
        self.emergency_stop_topic = self.get_parameter(
            'emergency_stop_topic').value
        self.speed_limit_timeout = float(
            self.get_parameter('speed_limit_timeout').value)
        self.use_dynamic_speed_limit = bool(
            self.get_parameter('use_dynamic_speed_limit').value)
        self.min_command_speed = float(
            self.get_parameter('min_command_speed').value)
        # minjae changes

        self.wheelbase = float(self.get_parameter('wheelbase').value)

        # minjae changes
        # self.lookahead_distance = float(
        #     self.get_parameter('lookahead_distance').value)
        # minjae changes

        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.max_path_distance = float(
            self.get_parameter('max_path_distance').value)
        self.max_heading_error = float(
            self.get_parameter('max_heading_error').value)
        self.search_back_points = int(
            self.get_parameter('search_back_points').value)
        self.search_forward_points = int(
            self.get_parameter('search_forward_points').value)

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.corner_slowdown_gain = float(
            self.get_parameter('corner_slowdown_gain').value)


        # minjae changes
        self.lookahead_time = float(
            self.get_parameter('lookahead_time').value)
        self.lookahead_min = float(
            self.get_parameter('lookahead_min').value)
        self.lookahead_max = float(
            self.get_parameter('lookahead_max').value)

        self.max_lateral_acceleration = float(
            self.get_parameter('max_lateral_acceleration').value)
        self.curvature_stride = float(
            self.get_parameter('curvature_stride').value)
        self.speed_profile_deceleration = max(1e-3, float(
            self.get_parameter('speed_profile_deceleration').value))
        # minjae changes


        # minjae changes
        # ERPM/서보 변환 계수 제거 (ackermann_to_vesc 가 담당)
        # minjae changes

        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        control_rate = float(self.get_parameter('control_rate').value)

        if self.drive_mode not in ('sim', 'real'):
            raise RuntimeError("drive_mode must be 'sim' or 'real'")

        self.current_odom = None
        self.current_path = None

        # minjae changes
        self.path_curvature = None
        self.path_speed_profile = None
        # minjae changes

        # minjae changes
        self.emergency_stop_active = False
        self.speed_limit = None
        self.last_speed_limit_time = None
        # minjae changes

        self.last_odom_time = None
        self.last_path_time = None
        self.nearest_index = None
        self.last_status_message = None
        self.last_status_time = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            Path, self.path_topic, self.path_callback, 10)

        # minjae changes
        self.create_subscription(
            Float32, self.speed_limit_topic, self.speed_limit_callback, 10)
        self.create_subscription(
            Bool, self.emergency_stop_topic,
            self.emergency_stop_callback, 10)

        # sim/real 모두 AckermannDriveStamped 하나만 발행한다.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        # minjae changes

        self.enable_service = self.create_service(
            SetBool, '/control/enable', self.enable_callback)
        # minjae changes
        self.add_on_set_parameters_callback(self.on_set_parameters)
        # minjae changes
        self.timer = self.create_timer(
            1.0 / max(control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'Pure Pursuit ready (enabled=%s, pose=%s -> %s, path=%s, '
            'drive=%s)' % (
                self.enabled,
                self.global_frame_id,
                self.base_frame_id,
                self.path_topic,
                self.drive_topic,  # minjae changes
            ))
        self.get_logger().info(
            'Start/stop: ros2 service call /control/enable '
            'std_srvs/srv/SetBool "{data: true|false}"')

    def odom_callback(self, msg):
        self.current_odom = msg
        self.last_odom_time = self.get_clock().now()

    def path_callback(self, msg):
        if not msg.poses:
            return
        if self.current_path is None or len(self.current_path.poses) != len(msg.poses):
            self.nearest_index = None
        self.current_path = msg

        # minjae changes
        self.path_curvature = self.compute_path_curvature(msg)
        self.path_speed_profile = self.compute_speed_profile(
            msg.poses, self.path_curvature)
        # minjae changes

        self.last_path_time = self.get_clock().now()

    # minjae changes
    def speed_limit_callback(self, msg):
        """로컬 플래너가 회피 중 발행하는 곡률 기반 속도 상한."""
        self.speed_limit = max(0.0, float(msg.data))
        self.last_speed_limit_time = self.get_clock().now()

    def emergency_stop_callback(self, msg):
        self.emergency_stop_active = bool(msg.data)
    # minjae changes

    # minjae changes
    def enable_callback(self, request, response):
        """원래 핸들러를 돌린 뒤 결과를 파라미터에 반영한다.

        기존에는 self.enabled 만 바뀌고 파라미터는 선언값 그대로여서,
        차가 달리는 중에도 `ros2 param get enabled` 가 False 를 반환했다.
        """
        response = self._enable_callback_impl(request, response)
        self._sync_enabled_param()
        return response

    def _sync_enabled_param(self):
        """self.enabled 를 'enabled' 파라미터에 되쓴다."""
        if bool(self.get_parameter('enabled').value) == self.enabled:
            return
        self._syncing_enabled_param = True
        try:
            self.set_parameters(
                [Parameter('enabled', Parameter.Type.BOOL, self.enabled)])
        finally:
            self._syncing_enabled_param = False

    def on_set_parameters(self, params):
        """param set 으로 정지는 허용하되, 출발은 거부한다.

        출발은 서비스를 거쳐야 한다. 경로 이탈 거리와 헤딩 오차 검사가
        거기에 있어서, 파라미터로 바로 켜면 그 검사를 건너뛰게 된다.
        """
        for param in params:
            if param.name != 'enabled' or self._syncing_enabled_param:
                continue
            if param.value:
                return SetParametersResult(
                    successful=False,
                    reason='Use the /control/enable service to start; '
                           'it runs the path distance and heading checks.')
            self.enabled = False
            self.nearest_index = None
            self.publish_stop()
            self.get_logger().warn('Pure Pursuit stopped via parameter')
        return SetParametersResult(successful=True)
    # minjae changes

    def _enable_callback_impl(self, request, response):
        if not request.data:
            self.enabled = False
            self.nearest_index = None
            self.publish_stop()
            response.success = True
            response.message = 'Pure Pursuit stopped'
            self.get_logger().info(response.message)
            return response

        problem = self.readiness_problem()
        if problem is not None:
            self.enabled = False
            self.publish_stop()
            response.success = False
            response.message = 'Cannot start: ' + problem
            self.get_logger().error(response.message)
            return response

        try:
            x, y, yaw = self.lookup_vehicle_pose()
        except TransformException as error:
            self.enabled = False
            self.publish_stop()
            response.success = False
            response.message = 'Cannot start: TF unavailable: ' + str(error)
            self.get_logger().error(response.message)
            return response

        self.nearest_index = None
        _, path_distance, path_heading = self.nearest_path_state(x, y)
        heading_error = math.atan2(
            math.sin(path_heading - yaw), math.cos(path_heading - yaw))
        if path_distance > self.max_path_distance:
            self.enabled = False
            self.publish_stop()
            response.success = False
            response.message = (
                'Cannot start: vehicle is %.2f m from path (limit %.2f m)'
                % (path_distance, self.max_path_distance))
            self.get_logger().error(response.message)
            return response
        if abs(heading_error) > self.max_heading_error:
            self.enabled = False
            self.publish_stop()
            response.success = False
            response.message = (
                'Cannot start: heading error is %.1f deg (limit %.1f deg)'
                % (math.degrees(abs(heading_error)),
                   math.degrees(self.max_heading_error)))
            self.get_logger().error(response.message)
            return response

        self.enabled = True
        response.success = True
        response.message = 'Pure Pursuit enabled'
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def lookup_vehicle_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.global_frame_id,
            self.base_frame_id,
            Time(),
            timeout=Duration(seconds=0.03),
        )
        translation = transform.transform.translation
        yaw = self.quaternion_to_yaw(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def age_seconds(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def readiness_problem(self):
        if self.current_path is None or not self.current_path.poses:
            return 'no global path'
        if self.age_seconds(self.last_path_time) > self.path_timeout:
            return 'global path is stale'
        if self.current_odom is None:
            return 'no odometry'
        if self.age_seconds(self.last_odom_time) > self.odom_timeout:
            return 'odometry is stale'
        return None

    def candidate_indices(self, count):
        if self.nearest_index is None:
            return range(count)
        return [
            (self.nearest_index + offset) % count
            for offset in range(-self.search_back_points,
                                self.search_forward_points + 1)
        ]


    # minjae changes
    def current_lookahead(self, speed):
        """Preview distance proportional to speed, bounded."""
        return self.clamp(
            self.lookahead_time * speed,
            self.lookahead_min,
            self.lookahead_max)

    def compute_path_curvature(self, msg):
        """Menger curvature per point, sampled at a fixed arc-length stride."""
        poses = msg.poses
        count = len(poses)
        if count < 5:
            return None

        # The local planner resamples at 0.04 m while the raw raceline is at
        # 0.20 m. Pick a stride so the three sample points stay far enough
        # apart to give a stable radius whatever the publisher spacing is.
        spacings = sorted(
            math.hypot(
                poses[(i + 1) % count].pose.position.x
                - poses[i].pose.position.x,
                poses[(i + 1) % count].pose.position.y
                - poses[i].pose.position.y)
            for i in range(count))
        median = spacings[count // 2]
        if median < 1e-6:
            return None
        stride = max(1, int(round(self.curvature_stride / median)))
        if count < 2 * stride + 1:
            return None

        curvature = []
        for idx in range(count):
            p1 = poses[(idx - stride) % count].pose.position
            p2 = poses[idx].pose.position
            p3 = poses[(idx + stride) % count].pose.position
            a = math.hypot(p2.x - p1.x, p2.y - p1.y)
            b = math.hypot(p3.x - p2.x, p3.y - p2.y)
            c = math.hypot(p3.x - p1.x, p3.y - p1.y)
            if a < 1e-6 or b < 1e-6 or c < 1e-6:
                curvature.append(0.0)
                continue
            cross = ((p2.x - p1.x) * (p3.y - p1.y)
                     - (p2.y - p1.y) * (p3.x - p1.x))
            curvature.append(2.0 * abs(cross) / (a * b * c))
        return curvature

    def compute_speed_profile(self, poses, curvature):
        """
        Highest speed at each path point that still allows braking ahead.

        Replaces the previous fixed-distance forward window.  A window has to
        guess how far ahead to look, and that guess depends on speed, on the
        chosen deceleration and on track length -- which is why it needed a
        per-track cap.  Here the braking constraint is propagated backwards
        around the whole closed path instead, so each corner reserves exactly
        the distance it needs and nothing depends on the layout.

        Runs once per received path, not per control cycle.
        """
        if curvature is None:
            return None
        count = len(poses)
        if count < 2 or count != len(curvature):
            return None

        # Static limit from the friction circle: a_y = v^2 * kappa.
        limit = [
            min(self.max_speed,
                math.sqrt(self.max_lateral_acceleration / max(k, 1e-3)))
            for k in curvature
        ]
        spacing = [
            math.hypot(
                poses[(idx + 1) % count].pose.position.x
                - poses[idx].pose.position.x,
                poses[(idx + 1) % count].pose.position.y
                - poses[idx].pose.position.y)
            for idx in range(count)
        ]

        # The slowest point is already final -- nothing ahead can force it
        # lower -- so starting the backward lap there converges in one pass
        # even though the path is a closed loop.
        start = min(range(count), key=limit.__getitem__)
        twice_decel = 2.0 * self.speed_profile_deceleration
        for step in range(1, count):
            idx = (start - step) % count
            ahead = (idx + 1) % count
            reachable = math.sqrt(
                limit[ahead] ** 2 + twice_decel * spacing[idx])
            if reachable < limit[idx]:
                limit[idx] = reachable
        return limit

    def profile_speed(self):
        """Speed-profile value at the vehicle's current path index."""
        if self.path_speed_profile is None or self.nearest_index is None:
            return None
        if len(self.path_speed_profile) != len(self.current_path.poses):
            return None
        return self.path_speed_profile[self.nearest_index]
    # minjae changes


    def nearest_path_state(self, x, y):
        poses = self.current_path.poses
        count = len(poses)
        nearest_idx = min(
            self.candidate_indices(count),
            key=lambda idx: math.hypot(
                poses[idx].pose.position.x - x,
                poses[idx].pose.position.y - y,
            ),
        )
        nearest_dist = math.hypot(
            poses[nearest_idx].pose.position.x - x,
            poses[nearest_idx].pose.position.y - y,
        )
        previous = poses[(nearest_idx - 1) % count].pose.position
        following = poses[(nearest_idx + 1) % count].pose.position
        path_heading = math.atan2(
            following.y - previous.y, following.x - previous.x)
        return nearest_idx, nearest_dist, path_heading

    # minjae changes
    # def find_lookahead_point(self, x, y, yaw):
    def find_lookahead_point(self, x, y, yaw, speed): # minjae changes
        poses = self.current_path.poses
        count = len(poses)
        if count < 2:
            return None

        nearest_idx, nearest_dist, path_heading = self.nearest_path_state(x, y)
        self.nearest_index = nearest_idx

        if nearest_dist > self.max_path_distance:
            return None
        heading_error = math.atan2(
            math.sin(path_heading - yaw), math.cos(path_heading - yaw))
        if abs(heading_error) > self.max_heading_error:
            return None

        # minjae changes
        lookahead = self.current_lookahead(speed)
        # minjae changes

        travelled = 0.0
        previous = poses[nearest_idx].pose.position
        for offset in range(1, count + 1):
            idx = (nearest_idx + offset) % count
            point = poses[idx].pose.position
            travelled += math.hypot(point.x - previous.x, point.y - previous.y)
            previous = point

            # minjae changes
            # if travelled < self.lookahead_distance:
            if travelled < lookahead:  # minjae changes
                continue

            dx = point.x - x
            dy = point.y - y
            x_car = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_car = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if x_car > 0.0:
                return x_car, y_car, math.hypot(dx, dy), nearest_dist

        return None

    def compute_steering(self, x_car, y_car, lookahead_dist):
        if lookahead_dist < 1e-6:
            return 0.0
        curvature = 2.0 * y_car / (lookahead_dist ** 2)
        steering = math.atan(self.wheelbase * curvature)
        return self.clamp(
            steering, -self.max_steering_angle, self.max_steering_angle)


    # minjae changes
    def compute_speed(self, steering):
        steer_ratio = abs(steering) / max(self.max_steering_angle, 1e-6)
        speed = self.target_speed * (
            1.0 - self.corner_slowdown_gain * steer_ratio)

        # return self.clamp(speed, self.min_speed, self.max_speed)

        # Anticipatory limit from the precomputed braking profile.
        curve_speed = self.profile_speed()
        if curve_speed is not None:
            speed = min(speed, curve_speed)

        speed = self.clamp(speed, self.min_speed, self.max_speed)

        # minjae changes
        # 로컬 플래너가 회피 중 발행하는 곡률 기반 상한을 반영한다.
        # 발행자가 없거나 오래됐으면 상한을 걸지 않는다(안전한 쪽으로 fail).
        upper_limit = self.max_speed
        if (self.use_dynamic_speed_limit
                and self.speed_limit is not None
                and self.age_seconds(self.last_speed_limit_time)
                <= self.speed_limit_timeout):
            upper_limit = min(upper_limit, self.speed_limit)
        if upper_limit <= 0.0:
            return 0.0
        # min_speed 가 상한보다 클 수 있으므로(예: speed:=0.5, min_speed=0.7)
        # 하한을 먼저 상한으로 눌러 모순을 막는다.
        speed = self.clamp(
            speed, min(self.min_speed, upper_limit), upper_limit)
        # 액추에이터 데드밴드: 0보다 크면 실제로 굴러가는 최소 명령까지 올린다.
        if 0.0 < speed < self.min_command_speed:
            speed = min(self.min_command_speed, upper_limit)
        # minjae changes

        # Tuning aid: delete this log block once the gains are settled.
        self.get_logger().info(
            'profile_v=%.2f cmd_v=%.2f' % (
                curve_speed if curve_speed is not None else -1.0, speed),
            throttle_duration_sec=0.5)

        return speed
    # minjae changes


    # minjae changes
    # sim(/drive) 과 real(/auto) 모두 동일하게 Ackermann 으로 발행한다.
    # 실차 ERPM/서보 변환은 ackermann_to_vesc 가 담당하므로 여기서 하지 않는다.
    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def publish_stop(self):
        self.publish_drive(0.0, 0.0)
    # minjae changes

    def warn_throttled(self, message):
        now = self.get_clock().now()
        if (message != self.last_status_message or
                self.last_status_time is None or
                (now - self.last_status_time).nanoseconds > 2_000_000_000):
            self.get_logger().warn(message)
            self.last_status_message = message
            self.last_status_time = now

    def control_loop(self):
        if not self.enabled:
            self.publish_stop()
            return

        # minjae changes
        # AEB 는 TF/odom 상태와 무관하게 최우선으로 처리한다.
        if self.emergency_stop_active:
            self.publish_stop()
            self.warn_throttled('Safety stop: local planner emergency stop')
            return
        # minjae changes

        problem = self.readiness_problem()
        if problem is not None:
            self.publish_stop()
            self.warn_throttled('Safety stop: ' + problem)
            return

        try:
            x, y, yaw = self.lookup_vehicle_pose()
        except TransformException as error:
            self.publish_stop()
            self.warn_throttled('Safety stop: TF unavailable: ' + str(error))
            return


        # minjae changes
        # lookahead = self.find_lookahead_point(x, y, yaw)
        speed = max(0.0, float(self.current_odom.twist.twist.linear.x))
        lookahead = self.find_lookahead_point(x, y, yaw, speed)
        # minjae changes


        if lookahead is None:
            self.publish_stop()
            self.warn_throttled(
                'Safety stop: no valid lookahead point or vehicle too far from path')
            return

        x_car, y_car, lookahead_dist, _ = lookahead
        steering = self.compute_steering(x_car, y_car, lookahead_dist)
        self.publish_drive(self.compute_speed(steering), steering)


def main(args=None):
    rclpy.init(args=args)
    node = MinjaePPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
