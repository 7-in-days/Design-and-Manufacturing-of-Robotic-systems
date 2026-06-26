#!/usr/bin/env python3

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class OccupancyDWAPlanner:
    def __init__(self):
        """DWA 파라미터와 fallback 상태값을 초기화한다."""
        self.cruise_speed = 0.45 # 되는거
        self.avoid_speed = 0.20
        self.stop_speed = 0.0

        #self.cruise_speed = 0.7 # test
        #self.avoid_speed = 0.2
        #self.stop_speed = 0.0

        self.roi_front_cells = 90
        self.roi_side_cells = 50
        self.front_start_cells = 2
        self.fov_deg = 180.0

        self.slow_cells = 32 # 0.96m
        self.stop_cells = 15 # 0.45m

        self.max_angular_z = 0.8
        self.angular_samples = 17
        self.fallback_turn_speed = 0.6
        self.fallback_forward_speed = 0.10
        self.fallback_locked_w = 0.0
        self.fallback_lock_steps = 0
        self.fallback_clear_count = 0
        self.fallback_clear_required = 5
        self.fallback_left_bias_margin = 15

        self.sim_time = 3.0
        self.sim_dt = 0.1

        self.robot_radius_m = 0.30
        self.safety_margin_m = 0.08
        self.cost_radius_m = 0.6

        self.progress_weight = 5.0
        self.heading_weight = 0.8
        self.lateral_weight = 0.55
        self.angular_weight = 0.6
        self.obstacle_count_weight = 0.8
        self.close_distance_weight = 6.0
        self.collision_count_limit = 0
        self.max_end_yaw_abs = 0.75

    def compute_command(self, grid, resolution, center_x, center_y):

        """occupancy grid를 보고 최종 linear.x/angular.z 명령을 계산"""

        occ_rows, occ_cols = self.get_front_obstacle_cells(grid, center_x, center_y)
        nearest_cells = self.nearest_obstacle_cells(occ_rows, occ_cols, center_x, center_y)
        fixed_v = self.select_linear_speed(nearest_cells)

        if fixed_v <= 1e-6 and len(occ_rows) > 0:
            fallback_w = self.get_locked_fallback_turn(occ_rows, center_y)
            fallback_v = self.select_fallback_speed(nearest_cells)
            return fallback_v, fallback_w, nearest_cells * resolution, True, len(occ_rows)

        w_candidates = np.linspace(-self.max_angular_z, self.max_angular_z, self.angular_samples)

        best_score = -1e18
        best_cmd = (fixed_v, 0.0)
        best_min_clearance_cells = nearest_cells
        fallback_used = False

        for wz in w_candidates:
            traj = self.simulate_trajectory(fixed_v, wz) # 미래 궤적 예측
            score, min_clearance_cells, collision = self.score_trajectory_on_grid(
                traj=traj,
                angular_z=wz,
                occ_rows=occ_rows,
                occ_cols=occ_cols,
                resolution=resolution,
                center_x=center_x,
                center_y=center_y,
            )

            if collision:
                continue

            if score > best_score:
                best_score = score
                best_cmd = (float(fixed_v), float(wz))
                best_min_clearance_cells = min_clearance_cells

        if best_score < -1e17:
            fallback_w = self.get_locked_fallback_turn(occ_rows, center_y)
            best_cmd = (self.select_fallback_speed(nearest_cells), fallback_w)
            fallback_used = True
        else:
            self.update_fallback_clear_lock(occ_rows, nearest_cells)

        return best_cmd[0], best_cmd[1], best_min_clearance_cells * resolution, fallback_used, len(occ_rows)

    def get_front_obstacle_cells(self, grid, center_x, center_y):
        """sim 좌표계 기준 전방 ROI 안의 장애물 cell만 추출"""
        rows, cols = np.nonzero(grid >= 50)
        if len(rows) == 0:
            return rows, cols

        forward_cells = cols - center_x # Real이랑 반대임
        side_cells = rows - center_y
        angles = np.arctan2(side_cells, forward_cells)
        fov_rad = math.radians(self.fov_deg)

        roi_mask = (
            (forward_cells >= self.front_start_cells)
            & (forward_cells <= self.roi_front_cells)
            & (np.abs(side_cells) <= self.roi_side_cells)
            & (np.abs(angles) <= fov_rad / 2.0)
        )
        return rows[roi_mask], cols[roi_mask]

    def nearest_obstacle_cells(self, occ_rows, occ_cols, center_x, center_y):
        """ROI 장애물 중 로봇 중심에서 가장 가까운 거리를 cell 단위로 계산"""

        if len(occ_rows) == 0:
            return 999.0

        forward_cells = occ_cols - center_x
        side_cells = occ_rows - center_y
        return float(np.min(np.hypot(forward_cells, side_cells)))

    def select_linear_speed(self, nearest_cells):

        """장애물 거리 기준으로 전진 속도 설정."""

        if nearest_cells >= 999.0:
            return self.cruise_speed
        
        if nearest_cells < self.stop_cells:
            return self.stop_speed
        
        if nearest_cells < self.slow_cells:
            return self.avoid_speed
        return self.cruise_speed

    def select_fallback_speed(self, nearest_cells):

        """fallback 중 너무 가까울 때만 정지하고 그 외에는 천천히 전진."""

        if nearest_cells < self.stop_cells:
            return self.stop_speed
        
        return self.fallback_forward_speed

    def simulate_trajectory(self, v, wz):
        """후보 속도 v,wz로 미래 경로를 짧게 예측."""

        steps = max(1, int(self.sim_time / self.sim_dt))

        x = 0.0
        y = 0.0
        yaw = 0.0
        traj = [(x, y, yaw)]

        for _ in range(steps):
            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw = normalize_angle(yaw + wz * self.sim_dt)
            traj.append((x, y, yaw))

        return traj

    def score_trajectory_on_grid(self, traj, angular_z, occ_rows, occ_cols, resolution, center_x, center_y):
        """후보 경로의 충돌 여부와 장애물 cost 기반 점수를 계산."""

        if len(occ_rows) == 0:
            end_x, end_y, end_yaw = traj[-1]
            if abs(end_yaw) > self.max_end_yaw_abs:
                return -1e18, 999.0, True
            return self.forward_score(end_x, end_y, end_yaw, angular_z), 999.0, False

        collision_radius_cells = max(1, int((self.robot_radius_m + self.safety_margin_m) / resolution))
        cost_radius_cells = max(collision_radius_cells + 1, int(self.cost_radius_m / resolution))

        total_obstacle_count = 0
        min_clearance_cells = 999.0

        for step_idx, (x, y, _) in enumerate(traj):
            cell_x, cell_y = self.forward_xy_to_grid(x, y, resolution, center_x, center_y)
            dx = occ_cols - cell_x
            dy = occ_rows - cell_y
            dist_cells = np.hypot(dx, dy)

            closest = float(np.min(dist_cells))
            min_clearance_cells = min(min_clearance_cells, closest)

            collision_count = int(np.sum(dist_cells <= collision_radius_cells))
            if collision_count > self.collision_count_limit:
                return -1e18, min_clearance_cells, True

            future_weight = 1.0 + 0.08 * step_idx
            total_obstacle_count += future_weight * int(np.sum(dist_cells <= cost_radius_cells))

        end_x, end_y, end_yaw = traj[-1]
        if abs(end_yaw) > self.max_end_yaw_abs:
            return -1e18, min_clearance_cells, True

        score = self.forward_score(end_x, end_y, end_yaw, angular_z)
        score -= self.obstacle_count_weight * total_obstacle_count
        score -= self.close_distance_weight / max(min_clearance_cells, 1.0)
        return score, min_clearance_cells, False

    def forward_score(self, end_x, end_y, end_yaw, angular_z):

        """전진성, heading, lateral 이동, 회전량으로 score 생성."""

        heading_error = abs(normalize_angle(end_yaw))
        return (
            self.progress_weight * end_x
            - self.heading_weight * heading_error
            - self.lateral_weight * abs(end_y)
            - self.angular_weight * abs(angular_z)
        )

    @staticmethod 

    def forward_xy_to_grid(x, y, resolution, center_x, center_y):

        """DWA 좌표 x,y를 sim occupancy grid cell 좌표로 변환."""

        cell_x = int(round(center_x + x / resolution))
        cell_y = int(round(center_y + y / resolution))
        return cell_x, cell_y

    def choose_fallback_turn_direction(self, occ_rows, center_y):
        """fallback 회전 방향을 선택 -> 현재 sim 제출용은 왼쪽 우선."""


        if len(occ_rows) == 0:
            return 0.0
        return self.fallback_turn_speed

    def get_locked_fallback_turn(self, occ_rows, center_y):

        """fallback 방향을 고정해서 좌우 와리가리 X."""

        if abs(self.fallback_locked_w) > 1e-6:
            self.fallback_lock_steps += 1
            return self.fallback_locked_w

        self.fallback_locked_w = self.choose_fallback_turn_direction(occ_rows, center_y)
        self.fallback_lock_steps = 1

        return self.fallback_locked_w

    def update_fallback_clear_lock(self, occ_rows, nearest_cells):

        """전방 clear가 연속 확인될 때만 fallback 방향 lock을 해제."""

        front_clear = len(occ_rows) == 0 or nearest_cells >= self.slow_cells
        if front_clear:
            self.fallback_clear_count += 1
        else:
            self.fallback_clear_count = 0

        if self.fallback_clear_count >= self.fallback_clear_required:
            self.reset_fallback_lock()

    def reset_fallback_lock(self):

        """fallback 방향 lock 상태를 초기화."""
        self.fallback_locked_w = 0.0
        self.fallback_lock_steps = 0
        self.fallback_clear_count = 0


def normalize_angle(angle):
    """각도를 -pi~pi 범위로 정규화한다."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Go2SimUniversalNode(Node):
    def __init__(self):
        """ 실행 노드"""
        super().__init__('go2_sim_universal')

        self.map_topic = '/occupancy_map'
        self.cmd_topic = '/cmd_vel'
        self.control_rate_hz = 10.0
        self.map_timeout_sec = 0.5
        self.debug_log_every_n = 10
        self.occ_threshold = 50
        self.map_history_frames = 3 # 라이다 끊기는걸 방지해서 3 프레임 정도 계속 냄겨둠
        self.map_history = deque(maxlen=self.map_history_frames)

        self.latest_grid = None
        self.map_resolution = 0.03
        self.center_x = 0
        self.center_y = 0
        self.last_map_time = None
        self.loop_count = 0

        self.planner = OccupancyDWAPlanner()
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info('Go2 sim universal node started.')

    def map_callback(self, msg):
        """수신한 occupancy map을 짧게 누적해 planner 입력으로 저장."""
        grid = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))

        if self.latest_grid is not None and self.latest_grid.shape != grid.shape:
            self.map_history.clear()

        occ_mask = grid >= self.occ_threshold
        self.map_history.append(occ_mask)
        accumulated_occ = np.logical_or.reduce(list(self.map_history))

        stable_grid = grid.copy()
        stable_grid[accumulated_occ] = 100

        self.latest_grid = stable_grid
        self.map_resolution = msg.info.resolution
        self.center_x = msg.info.width // 2
        self.center_y = msg.info.height // 2
        self.last_map_time = time.time()

    def control_loop(self):
        """주기적으로 DWA 명령을 계산하고 /cmd_vel로 publish."""
        self.loop_count += 1

        if self.last_map_time is None:
            self.publish_cmd(0.0, 0.0)
            return

        if time.time() - self.last_map_time > self.map_timeout_sec:
            self.get_logger().warn('Occupancy map timeout. Stop robot.')
            self.publish_cmd(0.0, 0.0)
            return

        v, w, min_clearance, fallback_used, obstacle_count = self.planner.compute_command(
            self.latest_grid,
            self.map_resolution,
            self.center_x,
            self.center_y,
        )
        self.publish_cmd(v, w)

        if fallback_used:
            self.get_logger().warn(f'Fallback rotate w={w:.2f}')

        if self.loop_count % self.debug_log_every_n == 0:
            self.get_logger().info(
                f'obs={obstacle_count}, cmd_v={v:.2f}, cmd_w={w:.2f}, min_clearance={min_clearance:.2f}'
            )

    def publish_cmd(self, linear_x, angular_z):
        """linear.x와 angular.z를 Twist 메시지로 publish."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2SimUniversalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Stop robot.')
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
