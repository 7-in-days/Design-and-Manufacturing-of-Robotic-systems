#!/usr/bin/env python3

import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

Isaac_sim = True
SIM_MAP = 1
SIM_MAP_HISTORY_FRAMES = 10

if Isaac_sim:
    if SIM_MAP == 1: # 1번 맵
        from go2_final.go2_dwa_sim_map1 import OccupancyDWAPlanner
        SIM_MAP_HISTORY_FRAMES = 10
        
    else: # 2번 맵
        from go2_final.go2_dwa_sim_map2 import OccupancyDWAPlanner
        SIM_MAP_HISTORY_FRAMES = 3
else: 
    # Real
    from go2_final.go2_dwa_real import OccupancyDWAPlanner


class Go2Control(Node):
    def __init__(self):
        super().__init__("go2_control")

        self.map_topic = "/occupancy_map"
        self.cmd_topics = ["/cmd_vel"]

        self.control_rate_hz = 10.0
        self.map_timeout_sec = 0.5
        self.debug_log_every_n = 10

        self.occ_threshold = 50
        self.unknown_as_obstacle = False
        self.frame_id = "base"
        self.loop_count = 0
        self.latest_grid = None
        self.map_history_frames = SIM_MAP_HISTORY_FRAMES # 누적 map frame 개수
        self.map_history = deque(maxlen=self.map_history_frames)
        self.map_resolution = 0.03
        self.center_x = 0
        self.center_y = 0
        self.last_map_time = None

        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_min_clearance = 999.0
        self.last_obstacle_count = 0

        self.planner = OccupancyDWAPlanner() # DWA 객체

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )

        self.cmd_pubs = [
            self.create_publisher(Twist, topic, 10)
            for topic in self.cmd_topics
        ]

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("Go2 control node started.")
        self.get_logger().info(f"Map topic: {self.map_topic}")
        self.get_logger().info(f"Cmd topics: {self.cmd_topics}")

    def map_callback(self, msg: OccupancyGrid):

        """

        occupancy map -> 로봇/라이다 프레임 중심.

        msg.info.resolution -> grid 한 칸의 실제 크기

        ex) resolution 0.03이면 grid 한 칸이 실제로 3cm x 3cm 크기임.

        msg.info.width -> grid의 가로 칸 수
        msg.info.height -> grid의 세로 칸 수 

        400개씩이라 총 12m x 12m 범위임.

        msg.info.origin -> 

        cell = distance(m) / resolution 


        여기서 distance값은 어떻게?

        전체 지도에서 로봇 위치가 바뀌는 게 아니라,
        새 LiDAR scan 기준으로 다시 로봇을 중앙에 두고 장애물을 찍음

        
        """

        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

        if Isaac_sim:
            if self.latest_grid is not None and self.latest_grid.shape != grid.shape:
                self.map_history.clear()

            occ_mask = grid >= self.occ_threshold
            self.map_history.append(occ_mask)

            accumulated_occ = np.logical_or.reduce(list(self.map_history))
            stable_grid = grid.copy()
            stable_grid[accumulated_occ] = 100
            self.latest_grid = stable_grid
        else:
            self.latest_grid = grid

        self.map_resolution = msg.info.resolution
        self.center_x = msg.info.width // 2
        self.center_y = msg.info.height // 2
        self.last_map_time = time.time()

    def control_loop(self):
        self.loop_count += 1 # 루프 몇 번 돌았나 용도

        if self.last_map_time is None: 
            self.publish_cmd(0.0, 0.0)
            return

        if time.time() - self.last_map_time > self.map_timeout_sec:
            self.get_logger().warn("Occupancy map timeout. Stop robot.") # occupancy map 안들어 올때
            self.publish_cmd(0.0, 0.0) # map 안들어오면 멈춤.
            return

        v, w, min_clearance, fallback_used, obstacle_count = self.planner.compute_command(
            self.latest_grid,
            self.map_resolution,
            self.center_x,
            self.center_y,
        ) 

        # v는 linear.x, w는 angular.z, min_clearance는 장애물과의 최소 거리, 
        # fallback_used는 DWA 후보 경로가 모두 안좋아서 제자리 회전할 때 True

        self.last_cmd_v = v # 마지막 명령 갱신
        self.last_cmd_w = w
        self.last_min_clearance = min_clearance # 장애물과의 최소 거리 갱신
        self.last_obstacle_count = obstacle_count

        self.publish_cmd(v, w) # linear x, angular z

        if fallback_used: # DWA 경로 안좋아서 제자리 회전할 때
            self.get_logger().warn(f"All DWA candidates rejected. Rotate w={w:.2f}")

        if self.loop_count % self.debug_log_every_n == 0:
            self.get_logger().info(
                f"obs={obstacle_count}, "
                f"cmd_v={v:.2f}, cmd_w={w:.2f}, "
                f"min_clearance={min_clearance:.2f}"
            )

    def publish_cmd(self, linear_x, angular_z): # 메세지 publish
        msg = Twist() 
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0

        # if Isaac_sim:
        #     msg.angular.z = -float(angular_z)
            

        # else:
        #     msg.angular.z = float(angular_z)

        msg.angular.z = float(angular_z)

        for pub in self.cmd_pubs:
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2Control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt. Stop robot.")
    finally:
        stop_msg = Twist()
        for pub in node.cmd_pubs:
            pub.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
