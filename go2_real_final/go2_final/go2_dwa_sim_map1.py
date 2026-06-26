#!/usr/bin/env python3

import math

import numpy as np


class OccupancyDWAPlanner:
    def __init__(self):

        self.cruise_speed = 0.8 # 장애물이 멀거나 없을 때 기본 전진 속도 linear.x
        self.avoid_speed = 0.3 # 장애물이 가까울 때 제자리 회전하기 위해 전진 속도를 0으로 둠
        self.stop_speed = 0.0 # 너무 가까운 장애물이 있거나 fallback일 때 정지 속도

        self.roi_front_cells = 90 # 로봇 앞쪽으로 볼 cell 수, 0.03m 기준 약 2.7m
        self.roi_side_cells = 50 # 좌우로 볼 cell 수, 0.03m 기준 약 +/-1.5m
        self.front_start_cells = 2 # 로봇 바로 앞 잡음 제거용 시작 cell, 0.03m 기준 약 6cm부터 봄
        self.fov_deg = 180.0 # 전방 ROI 각도, 정면 기준 +/-90도 영역만 사용

        self.slow_cells = 24 # 가장 가까운 장애물이 이 cell보다 가까우면 avoid_speed로 감속
        self.stop_cells = 15 # 가장 가까운 장애물이 이 cell보다 가까우면 stop_speed로 정지

        # angular.z 후보 범위. 너무 크면 목적지 없이 과회전해서 이상한 방향으로 빠질 수 있음.
        self.max_angular_z = 0.8 # DWA가 테스트할 최대 회전 속도 범위, -0.8~+0.8 rad/s
        self.angular_samples = 17 # -max부터 +max까지 나눌 angular.z 후보 개수
        self.fallback_turn_speed = 0.6 # 모든 후보가 막혔을 때 제자리 회전 속도
        self.fallback_locked_w = 0.0
        self.fallback_lock_steps = 0
        self.fallback_min_lock_steps = 10
        self.fallback_left_bias_margin = 40

        # 후보 경로를 몇 초 앞까지 예측할지. dt가 작을수록 촘촘히 검사함.
        self.sim_time = 3.0 # 각 후보 속도로 몇 초 뒤까지 가상 주행해볼지
        self.sim_dt = 0.1 # 가상 주행을 몇 초 간격으로 나누어 검사할지

        # 로봇 크기/안전 여유/장애물 cost 반경. resolution에 따라 cell 개수로 변환해서 사용함.
        self.robot_radius_m = 0.30 # 로봇 반지름으로 보는 값, 충돌 반경 계산에 사용
        self.safety_margin_m = 0.10 # 로봇 반지름에 추가하는 안전 여유 거리
        self.cost_radius_m = 0.55 # 후보 경로 주변 몇 m 안의 장애물 cell을 cost로 셀지

        # cost 가중치. 장애물 관련 weight가 클수록 장애물 cell이 많은 경로를 더 강하게 피함.
        self.progress_weight = 5.0 # 앞으로 많이 간 후보에 주는 보상 가중치
        self.heading_weight = 0.8 # 최종 yaw가 정면에서 틀어진 정도에 대한 벌점 가중치
        self.lateral_weight = 0.4 # 옆으로 많이 빠지는 경로에 대한 벌점 가중치
        self.angular_weight = 0.6 # 큰 angular.z를 쓰는 후보에 대한 벌점 가중치
        self.obstacle_count_weight = 0.8 # 경로 주변 장애물 cell 개수에 대한 벌점 가중치
        self.close_distance_weight = 5.0 # 장애물과 가장 가까운 거리에 대한 벌점 가중치
        self.collision_count_limit = 0 # 충돌 반경 안에 허용할 장애물 cell 개수, 0이면 하나라도 있으면 reject
        self.max_end_yaw_abs = 0.9 # 예측 끝 yaw 허용 한계, 이보다 많이 돌면 후보 reject

    def compute_command(self, grid, resolution, center_x, center_y):

        """occupancy grid를 받아 최종 linear.x, angular.z 계산."""

        occ_rows, occ_cols = self.get_front_obstacle_cells(grid, center_x, center_y)

        nearest_cells = self.nearest_obstacle_cells(occ_rows, occ_cols, center_x, center_y)
        fixed_v = self.select_linear_speed(nearest_cells)

        if fixed_v <= 1e-6 and len(occ_rows) > 0:
            fallback_w = self.get_locked_fallback_turn(occ_rows, center_y)
            min_clearance_m = nearest_cells * resolution
            return 0.0, fallback_w, min_clearance_m, True, len(occ_rows)

        w_candidates = np.linspace(
            -self.max_angular_z,
            self.max_angular_z,
            self.angular_samples,
        )

        best_score = -1e18
        best_cmd = (fixed_v, 0.0)
        best_min_clearance_cells = nearest_cells
        fallback_used = False

        for wz in w_candidates:
            traj = self.simulate_trajectory(
                fixed_v,
                wz,
            )
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
            best_cmd = (0.0, fallback_w)
            fallback_used = True

        min_clearance_m = best_min_clearance_cells * resolution
        return best_cmd[0], best_cmd[1], min_clearance_m, fallback_used, len(occ_rows)

    def get_front_obstacle_cells(self, grid, center_x, center_y):

        """grid에서 전방 ROI 안에 있는 장애물 cell의 row/col만 추출한다."""

        rows, cols = np.nonzero(grid >= 50)

        if len(rows) == 0:
            return rows, cols

        forward_cells = cols - center_x
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
        
        """전방 ROI 장애물 중 로봇 중심과 가장 가까운 거리(cell 단위)를 계산."""

        if len(occ_rows) == 0:
            return 999.0

        forward_cells = occ_cols - center_x
        side_cells = occ_rows - center_y
        return float(np.min(np.hypot(forward_cells, side_cells)))

    def select_linear_speed(self, nearest_cells):

        """가장 가까운 장애물 거리(cell)에 따라 고정 전진 속도를 선택."""

        if nearest_cells >= 999.0:
            return self.cruise_speed

        if nearest_cells < self.stop_cells:
            return self.stop_speed

        if nearest_cells < self.slow_cells:
            return self.avoid_speed

        return self.cruise_speed

    def simulate_trajectory(self, v, wz):

        """주어진 linear.x와 angular.z로 미래 경로를 그림."""

        steps = max(1, int(self.sim_time / self.sim_dt))
        x = 0.0
        y = 0.0
        yaw = 0.0
        traj = [(x, y, yaw)]

        for _ in range(steps):
            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw += wz * self.sim_dt
            yaw = normalize_angle(yaw)
            traj.append((x, y, yaw))

        return traj

    def score_trajectory_on_grid(self, traj, angular_z, occ_rows, occ_cols, resolution, center_x, center_y):

        """후보 경로 주변 장애물 cell 개수와 진행성을 이용해 DWA 점수를 계산."""

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

        """장애물 cost를 제외한 전진/방향/측면/회전 기본 점수를 계산."""

        heading_error = abs(normalize_angle(0.0 - end_yaw))

        return (
            self.progress_weight * end_x
            - self.heading_weight * heading_error
            - self.lateral_weight * abs(end_y)
            - self.angular_weight * abs(angular_z)
        )

    @staticmethod
    def forward_xy_to_grid(x, y, resolution, center_x, center_y):

        """planner 기준 전방 x,y 좌표를 raw occupancy grid cell 좌표로 변환."""
        cell_x = int(round(center_x + x / resolution))
        cell_y = int(round(center_y + y / resolution))
        return cell_x, cell_y

    def choose_fallback_turn_direction(self, occ_rows, center_y):
        """map1에서는 fallback 시 왼쪽을 우선하되, 왼쪽이 많이 막힌 경우만 우회전."""
        if len(occ_rows) == 0:
            return 0.0

        left_count = np.sum(occ_rows > center_y)
        right_count = np.sum(occ_rows < center_y)

        if left_count > right_count + self.fallback_left_bias_margin:
            return -self.fallback_turn_speed
        return self.fallback_turn_speed

    def get_locked_fallback_turn(self, occ_rows, center_y):
        """fallback 회전 방향이 좌우로 흔들리지 않도록 최소 step 동안 고정."""

        if abs(self.fallback_locked_w) > 1e-6 and self.fallback_lock_steps < self.fallback_min_lock_steps:
            self.fallback_lock_steps += 1
            return self.fallback_locked_w

        self.fallback_locked_w = self.choose_fallback_turn_direction(occ_rows, center_y)
        self.fallback_lock_steps = 1
        return self.fallback_locked_w

    def reset_fallback_lock(self):
        self.fallback_locked_w = 0.0
        self.fallback_lock_steps = 0


def normalize_angle(angle):
    """각도를 -pi~pi 범위로 정규화."""

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
