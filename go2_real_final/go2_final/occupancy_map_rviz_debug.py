#!/usr/bin/env python3

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from go2_final.go2_control import OccupancyDWAPlanner
from go2_final.go2_control import Isaac_sim
from go2_final.go2_control import SIM_MAP_HISTORY_FRAMES


class OccupancyMapRvizDebug(Node):
    def __init__(self):
        super().__init__("occupancy_map_rviz_debug")

        self.map_topic = "/occupancy_map"
        self.marker_topic = "/occupancy_map_debug/markers"

        self.occ_threshold = 50
        self.map_history_frames = SIM_MAP_HISTORY_FRAMES
        self.map_history = deque(maxlen=self.map_history_frames)
        self.assume_robot_at_grid_center = True

        self.swap_xy = False

        if Isaac_sim:
            self.invert_x = False
            self.invert_y = True
        else:
            self.invert_x = True
            self.invert_y = False

    
        self.invert_y = False

        self.roi_x_min = 0.05
        self.roi_x_max = 3.0
        self.roi_y_abs = 2.0
        self.front_x_min = 0.05
        self.front_x_max = 1.4
        self.front_y_abs = 0.55

        self.max_visualized_obstacles = 1500
        self.log_every_sec = 1.0
        self.last_log_time = 0.0

        self.frame_id = "base"
        self.planner = OccupancyDWAPlanner()

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info("Occupancy map RViz debug node started.")
        self.get_logger().info(f"Subscribe: {self.map_topic}")
        self.get_logger().info(f"Publish:   {self.marker_topic}")

    def map_callback(self, msg: OccupancyGrid):
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        grid = self.stabilize_grid(grid)
        resolution = msg.info.resolution
        center_x = msg.info.width // 2
        center_y = msg.info.height // 2

        points = self.occupancy_grid_to_points(msg, grid)
        selected_traj, candidate_records, fallback_w = self.compute_debug_paths(
            grid,
            resolution,
            center_x,
            center_y,
        )

        roi_mask = (
            (points[:, 0] >= self.roi_x_min)
            & (points[:, 0] <= self.roi_x_max)
            & (np.abs(points[:, 1]) <= self.roi_y_abs)
        ) if points.shape[0] > 0 else np.array([], dtype=bool)

        roi_points = points[roi_mask] if points.shape[0] > 0 else points

        front_mask = (
            (roi_points[:, 0] >= self.front_x_min)
            & (roi_points[:, 0] <= self.front_x_max)
            & (np.abs(roi_points[:, 1]) <= self.front_y_abs)
        ) if roi_points.shape[0] > 0 else np.array([], dtype=bool)

        front_count = int(np.sum(front_mask)) if roi_points.shape[0] > 0 else 0
        left_count = int(np.sum(roi_points[:, 1] > self.front_y_abs)) if roi_points.shape[0] > 0 else 0
        right_count = int(np.sum(roi_points[:, 1] < -self.front_y_abs)) if roi_points.shape[0] > 0 else 0

        now = time.time()
        if now - self.last_log_time >= self.log_every_sec:
            self.last_log_time = now
            self.get_logger().info(
                f"obs_total={points.shape[0]} "
                f"roi={roi_points.shape[0]} "
                f"front={front_count} "
                f"left={left_count} "
                f"right={right_count} "
                f"frame={self.frame_id}"
            )

        self.publish_markers(
            roi_points,
            front_count,
            left_count,
            right_count,
            selected_traj,
            candidate_records,
            fallback_w,
            msg.header.stamp,
        )

    def compute_debug_paths(self, grid, resolution, center_x, center_y):
        occ_rows, occ_cols = self.planner.get_front_obstacle_cells(
            grid,
            center_x,
            center_y,
        )
        nearest_cells = self.planner.nearest_obstacle_cells(
            occ_rows,
            occ_cols,
            center_x,
            center_y,
        )
        fixed_v = self.planner.select_linear_speed(nearest_cells)

        candidate_records = []
        best_score = -1e18
        selected_traj = []
        fallback_w = None

        w_candidates = np.linspace(
            -self.planner.max_angular_z,
            self.planner.max_angular_z,
            self.planner.angular_samples,
        )

        for wz in w_candidates:
            traj = self.planner.simulate_trajectory(fixed_v, wz)
            score, min_clearance_cells, collision = self.planner.score_trajectory_on_grid(
                traj=traj,
                angular_z=wz,
                occ_rows=occ_rows,
                occ_cols=occ_cols,
                resolution=resolution,
                center_x=center_x,
                center_y=center_y,
            )

            record = {
                "w": float(wz),
                "traj": traj,
                "score": float(score),
                "collision": bool(collision),
                "min_clearance": float(min_clearance_cells * resolution),
            }
            candidate_records.append(record)

            if not collision and score > best_score:
                best_score = score
                selected_traj = traj

        if best_score < -1e17:
            fallback_w = self.choose_debug_fallback_turn(occ_rows, center_y)
            selected_traj = self.planner.simulate_trajectory(0.0, fallback_w)

        return selected_traj, candidate_records, fallback_w

    def stabilize_grid(self, grid):
        if not Isaac_sim:
            return grid

        if len(self.map_history) > 0 and self.map_history[-1].shape != grid.shape:
            self.map_history.clear()

        occ_mask = grid >= self.occ_threshold
        self.map_history.append(occ_mask)

        accumulated_occ = np.logical_or.reduce(list(self.map_history))
        stable_grid = grid.copy()
        stable_grid[accumulated_occ] = 100
        return stable_grid

    def occupancy_grid_to_points(self, msg: OccupancyGrid, grid):
        rows, cols = np.nonzero(grid >= self.occ_threshold)

        if len(rows) == 0:
            return np.empty((0, 2), dtype=np.float32)

        res = msg.info.resolution

        if self.assume_robot_at_grid_center:
            center_x = msg.info.width / 2.0
            center_y = msg.info.height / 2.0
            xs = (cols - center_x + 0.5) * res
            ys = (rows - center_y + 0.5) * res
        else:
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            xs = origin_x + (cols + 0.5) * res
            ys = origin_y + (rows + 0.5) * res

        if self.swap_xy:
            xs, ys = ys.copy(), xs.copy()
        if self.invert_x:
            xs = -xs
        if self.invert_y:
            ys = -ys

        return np.column_stack((xs, ys)).astype(np.float32)

    def publish_markers(
        self,
        points,
        front_count,
        left_count,
        right_count,
        selected_traj,
        candidate_records,
        fallback_w,
        stamp,
    ):
        markers = MarkerArray()
        markers.markers.append(self.make_delete_all_marker(stamp))
        markers.markers.append(self.make_robot_marker(stamp))
        markers.markers.append(self.make_heading_marker(stamp))
        markers.markers.append(self.make_roi_box_marker(stamp))
        markers.markers.append(self.make_front_box_marker(stamp))
        markers.markers.append(self.make_obstacle_points_marker(points, stamp))
        markers.markers.append(self.make_status_text_marker(front_count, left_count, right_count, stamp))
        markers.markers.extend(self.make_candidate_path_markers(candidate_records, stamp))
        markers.markers.append(self.make_selected_path_marker(selected_traj, fallback_w, stamp))
        self.marker_pub.publish(markers)

    def make_delete_all_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.action = Marker.DELETEALL
        return marker

    def make_robot_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "robot_center"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.z = 0.10
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        return marker

    def make_heading_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "planner_forward"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.scale.y = 0.10
        marker.scale.z = 0.12
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.points.append(self.point(0.0, 0.0, 0.16))
        marker.points.append(self.point(0.75, 0.0, 0.16))
        return marker

    def make_roi_box_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "roi_box"
        marker.id = 3
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (self.roi_x_min + self.roi_x_max) / 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.015
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.roi_x_max - self.roi_x_min
        marker.scale.y = 2.0 * self.roi_y_abs
        marker.scale.z = 0.03
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.10
        return marker

    def make_front_box_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "front_danger_box"
        marker.id = 4
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (self.front_x_min + self.front_x_max) / 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.04
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.front_x_max - self.front_x_min
        marker.scale.y = 2.0 * self.front_y_abs
        marker.scale.z = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.0
        marker.color.a = 0.22
        return marker

    def make_obstacle_points_marker(self, points, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "obstacle_points"
        marker.id = 5
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        if points.shape[0] > 0:
            step = max(1, math.ceil(points.shape[0] / self.max_visualized_obstacles))
            for x, y in points[::step]:
                marker.points.append(self.point(float(x), float(y), 0.08))

        return marker

    def make_status_text_marker(self, front_count, left_count, right_count, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "counts"
        marker.id = 6
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.25
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.55
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"front={front_count} left={left_count} right={right_count}"
        return marker

    def make_candidate_path_markers(self, candidate_records, stamp):
        markers = []
        valid_records = [record for record in candidate_records if not record["collision"]]
        collision_records = [record for record in candidate_records if record["collision"]]

        for marker_id, record in enumerate(valid_records):
            markers.append(
                self.make_path_marker(
                    record["traj"],
                    stamp,
                    "valid_candidate_paths",
                    marker_id,
                    0.15,
                    0.55,
                    1.0,
                    0.35,
                    0.012,
                    0.06,
                )
            )

        for marker_id, record in enumerate(collision_records):
            markers.append(
                self.make_path_marker(
                    record["traj"],
                    stamp,
                    "collision_candidate_paths",
                    marker_id,
                    1.0,
                    0.08,
                    0.08,
                    0.30,
                    0.010,
                    0.045,
                )
            )

        return markers

    def make_selected_path_marker(self, selected_traj, fallback_w, stamp):
        marker = self.make_path_marker(
            selected_traj,
            stamp,
            "selected_path",
            0,
            0.0,
            1.0,
            0.18,
            1.0,
            0.035,
            0.09,
        )

        if fallback_w is not None:
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.0
            marker.scale.x = 0.045
            marker.points.clear()
            marker.points.extend(self.make_fallback_arc_points(fallback_w, 0.09))

        return marker

    def make_path_marker(self, traj, stamp, ns, marker_id, r, g, b, a, width, z):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        for x, y, _ in traj:
            marker.points.append(self.point(float(x), float(y), z))

        return marker

    def make_fallback_arc_points(self, fallback_w, z):
        direction = 1.0 if fallback_w >= 0.0 else -1.0
        radius = 0.35
        points = []
        for angle in np.linspace(0.0, direction * math.pi * 0.85, 24):
            points.append(
                self.point(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    z,
                )
            )
        return points

    @staticmethod
    def choose_debug_fallback_turn(occ_rows, center_y):
        if len(occ_rows) == 0:
            return 0.0

        left_count = np.sum(occ_rows > center_y)
        right_count = np.sum(occ_rows < center_y)

        if left_count > right_count:
            return -0.6
        return 0.6

    @staticmethod
    def point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapRvizDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
