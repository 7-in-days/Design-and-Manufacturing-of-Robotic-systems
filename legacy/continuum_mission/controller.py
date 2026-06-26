#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Continuum robot controller using ROS2 and the low-level Dynamixel SDK.
"""

import time
from typing import Dict, List

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
import rclpy
from rclpy.node import Node

try:
    from . import config
    from .trajectory import minimum_jerk_scalar
except ImportError:
    import config
    from trajectory import minimum_jerk_scalar


ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
POSITION_MODE = 3

MIN_POSITION = 0
MAX_POSITION = 4095


class ContinuumController(Node):
    """
    Continuum 로봇 제어 클래스.

    XL430 2개 모터를 Dynamixel SDK low-level PacketHandler로 제어합니다.
    """

    def __init__(self):
        super().__init__("continuum_controller")

        self.port_name = getattr(config, "DEVICENAME", "/dev/ttyUSB0")
        self.baudrate = getattr(config, "BAUDRATE", 57600)
        self.protocol_version = config.DYNAMIXEL_PROTOCOL_VERSION
        self.motor_ids = getattr(config, "MOTOR_NUM", [0, 1])
        self.motors = {}

        self.motor_states = {
            motor_id: {
                "position": 0,
                "velocity": None,
                "current": None,
                "temperature": None,
            }
            for motor_id in self.motor_ids
        }

        self.profile_velocity = 60
        self.profile_acceleration = 20
        self.position_tolerance = 10
        self.hardware_alert_retry_count = max(
            0,
            int(getattr(config, "HARDWARE_ALERT_RETRY_COUNT", 1)),
        )
        self.reported_hardware_alerts = set()

        self.get_logger().info("Continuum Controller 초기화 중...")

        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.openPort():
            raise RuntimeError(f"포트 열기 실패: {self.port_name}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"baudrate 설정 실패: {self.baudrate}")

        self.get_logger().info(
            f"포트 연결 완료: {self.port_name}, "
            f"baudrate={self.baudrate}, protocol={self.protocol_version}"
        )

    def _to_signed(self, value: int, byte_size: int) -> int:
        sign_bit = 1 << (byte_size * 8 - 1)
        full_range = 1 << (byte_size * 8)
        return value - full_range if value & sign_bit else value

    def _comm_ok(self, motor_id: int, comm_result: int, dxl_error: int, action: str) -> bool:
        if comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(comm_result)
            self.get_logger().warning(f"모터 {motor_id}: {action} 통신 실패 ({message})")
            return False

        if dxl_error:
            message = self.packet_handler.getRxPacketError(dxl_error)
            if dxl_error == 128:
                key = (motor_id, action, dxl_error)
                if key not in self.reported_hardware_alerts:
                    self.reported_hardware_alerts.add(key)
                    self.get_logger().warning(
                        f"모터 {motor_id}: {action} hardware alert "
                        f"(dxl_error={dxl_error}, {message})"
                    )
            else:
                self.get_logger().warning(
                    f"모터 {motor_id}: {action} packet error "
                    f"(dxl_error={dxl_error}, {message})"
                )
        return True

    def _read1(self, motor_id: int, address: int, action: str):
        value, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, address
        )
        if not self._comm_ok(motor_id, comm_result, dxl_error, action):
            return None
        return value

    def _read2(self, motor_id: int, address: int, action: str, signed: bool = False):
        value, comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, address
        )
        if not self._comm_ok(motor_id, comm_result, dxl_error, action):
            return None
        return self._to_signed(value, 2) if signed else value

    def _read4(self, motor_id: int, address: int, action: str, signed: bool = False):
        value, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, address
        )
        if not self._comm_ok(motor_id, comm_result, dxl_error, action):
            return None
        return self._to_signed(value, 4) if signed else value

    def _write1(self, motor_id: int, address: int, value: int, action: str) -> bool:
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, address, value
        )
        return self._comm_ok(motor_id, comm_result, dxl_error, action)

    def _write4(self, motor_id: int, address: int, value: int, action: str) -> bool:
        comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, address, value
        )
        return self._comm_ok(motor_id, comm_result, dxl_error, action)

    def _write_goal_position_with_recovery(
        self,
        motor_id: int,
        value: int,
        action: str,
    ) -> bool:
        max_attempts = 1 + self.hardware_alert_retry_count
        for attempt in range(max_attempts):
            comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                motor_id,
                ADDR_GOAL_POSITION,
                value,
            )
            if comm_result != COMM_SUCCESS:
                self._comm_ok(motor_id, comm_result, dxl_error, action)
                return False
            if dxl_error != 128:
                self._comm_ok(motor_id, comm_result, dxl_error, action)
                return dxl_error == 0

            self._comm_ok(motor_id, comm_result, dxl_error, action)
            if attempt == max_attempts - 1:
                self.get_logger().error(
                    f" 모터 {motor_id}: hardware alert 복구 후에도 {action} 실패"
                )
                return False
            if not self.recover_from_hardware_alerts():
                return False

        return False

    def _read_hardware_error_status(self, motor_id: int) -> int:
        value = self._read1(motor_id, ADDR_HARDWARE_ERROR_STATUS, "hardware error status read")
        return 0 if value is None else value

    def _hardware_error_labels(self, status: int) -> List[str]:
        labels = []
        if status & 0x01:
            labels.append("input voltage")
        if status & 0x04:
            labels.append("overheating")
        if status & 0x08:
            labels.append("motor encoder")
        if status & 0x10:
            labels.append("electrical shock / insufficient power")
        if status & 0x20:
            labels.append("overload")
        return labels

    def reboot_motor(self, motor_id: int) -> bool:
        """Software reboot one motor and verify that communication returns."""
        self.get_logger().warning(f" 모터 {motor_id}: software reboot 요청")
        comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, motor_id)
        if not self._comm_ok(motor_id, comm_result, dxl_error, "reboot"):
            return False

        time.sleep(1.0)
        model_number, comm_result, dxl_error = self.packet_handler.ping(
            self.port_handler, motor_id
        )
        if not self._comm_ok(motor_id, comm_result, dxl_error, "post-reboot ping"):
            return False

        self.reported_hardware_alerts = {
            key for key in self.reported_hardware_alerts if key[0] != motor_id
        }
        self.get_logger().info(
            f" 모터 {motor_id}: reboot 완료, 모델 번호={model_number}"
        )
        return True

    def _initialize_motor_runtime(self, motor_id: int) -> bool:
        if not self._write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "torque off"):
            return False
        if not self._write1(motor_id, ADDR_OPERATING_MODE, POSITION_MODE, "position mode set"):
            return False

        self._write4(
            motor_id,
            ADDR_PROFILE_ACCELERATION,
            self.profile_acceleration,
            "profile acceleration set",
        )
        self._write4(
            motor_id,
            ADDR_PROFILE_VELOCITY,
            self.profile_velocity,
            "profile velocity set",
        )

        if not self._write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, "torque on"):
            return False

        initial_pos = self._read4(
            motor_id,
            ADDR_PRESENT_POSITION,
            "present position read",
            signed=True,
        )
        if initial_pos is None:
            return False

        self.motor_states[motor_id]["position"] = initial_pos
        self.get_logger().info(f" 모터 {motor_id}: 현재 위치 = {initial_pos}")
        return True

    def recover_from_hardware_alerts(self) -> bool:
        """Reboot motors with active hardware alerts and restore runtime settings."""
        alert_motors = []
        for motor_id in self.motor_ids:
            hardware_status = self._read_hardware_error_status(motor_id)
            if hardware_status:
                labels = ", ".join(self._hardware_error_labels(hardware_status)) or "unknown"
                self.get_logger().warning(
                    f" 모터 {motor_id}: 복구 대상 Hardware Error Status = "
                    f"{hardware_status} ({labels})"
                )
                alert_motors.append(motor_id)

        if not alert_motors:
            return True

        for motor_id in alert_motors:
            if not self.reboot_motor(motor_id):
                return False

        for motor_id in alert_motors:
            hardware_status = self._read_hardware_error_status(motor_id)
            if hardware_status:
                labels = ", ".join(self._hardware_error_labels(hardware_status)) or "unknown"
                self.get_logger().error(
                    f" 모터 {motor_id}: reboot 후에도 Hardware Error Status = "
                    f"{hardware_status} ({labels})"
                )
                return False
            if not self._initialize_motor_runtime(motor_id):
                return False

        self.get_logger().info(" hardware alert 복구 완료, 동작 재개")
        return True

    def check_connecting(self) -> bool:
        """
        연결된 모든 모터 확인 및 초기화.
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("모터 연결 확인 중...")
        self.get_logger().info("=" * 60)

        for motor_id in self.motor_ids:
            model_number, comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler, motor_id
            )
            if not self._comm_ok(motor_id, comm_result, dxl_error, "ping"):
                return False

            self.motors[motor_id] = True
            self.get_logger().info(f" 모터 {motor_id} 생성")
            self.get_logger().info(f"  - 모델 번호: {model_number}")

            hardware_status = self._read_hardware_error_status(motor_id)
            if hardware_status:
                labels = ", ".join(self._hardware_error_labels(hardware_status)) or "unknown"
                self.get_logger().warning(
                    f" 모터 {motor_id}: Hardware Error Status = {hardware_status} "
                    f"({labels})"
                )
                if not self.reboot_motor(motor_id):
                    return False

                hardware_status = self._read_hardware_error_status(motor_id)
                if hardware_status:
                    labels = ", ".join(self._hardware_error_labels(hardware_status)) or "unknown"
                    self.get_logger().error(
                        f" 모터 {motor_id}: reboot 후에도 Hardware Error Status = "
                        f"{hardware_status} ({labels})"
                    )
                    return False

            if not self._initialize_motor_runtime(motor_id):
                return False

        self.get_logger().info("=" * 60)
        self.get_logger().info(" 모든 모터 연결 및 초기화 완료!")
        self.get_logger().info("=" * 60)
        return True

    def get_realtime_position(self) -> Dict[int, Dict]:
        """
        모든 모터의 실시간 센서 데이터 읽기.
        """
        for motor_id in self.motor_ids:
            position = self._read4(
                motor_id,
                ADDR_PRESENT_POSITION,
                "present position read",
                signed=True,
            )
            if position is None:
                self.get_logger().warning(
                    f"모터 {motor_id}: position feedback read failed, using last state"
                )
                self.motor_states[motor_id]["timestamp"] = time.time()
                continue

            velocity = self._read4(
                motor_id,
                ADDR_PRESENT_VELOCITY,
                "present velocity read",
                signed=True,
            )
            current = self._read2(
                motor_id,
                ADDR_PRESENT_CURRENT,
                "present current read",
                signed=True,
            )
            temperature = self._read1(
                motor_id,
                ADDR_PRESENT_TEMPERATURE,
                "present temperature read",
            )

            self.motor_states[motor_id] = {
                "position": position,
                "velocity": velocity,
                "current": current,
                "temperature": temperature,
                "timestamp": time.time(),
            }

        return self.motor_states

    def print_realtime_feedback(self):
        """실시간 피드백을 보기 좋게 출력."""
        states = self.get_realtime_position()

        print("\n" + "=" * 70)
        print(f"[{time.strftime('%H:%M:%S')}] 모터 상태")
        print("=" * 70)

        for motor_id in self.motor_ids:
            state = states[motor_id]
            velocity = state["velocity"] if state["velocity"] is not None else "N/A"
            current = state["current"] if state["current"] is not None else "N/A"
            temperature = state["temperature"] if state["temperature"] is not None else "N/A"
            print(
                f"모터 {motor_id} | 위치:{state['position']:5d} | "
                f"속도:{velocity!s:>5} | 전류:{current!s:>5} | "
                f"온도:{temperature!s:>3}°C"
            )

        print("=" * 70 + "\n")

    def move_to_position(
        self,
        target_positions: Dict[int, int],
        timeout: float = 10.0,
        check_interval: float = 0.05,
    ) -> bool:
        """
        지정된 위치로 모터 이동.
        """
        self.get_logger().info("=" * 70)
        self.get_logger().info("위치 제어 시작")
        self.get_logger().info(f"목표: {target_positions}")
        self.get_logger().info("=" * 70)

        for motor_id, target_pos in target_positions.items():
            if motor_id not in self.motors:
                self.get_logger().error(f" 모터 {motor_id} 없음")
                return False
            if not (MIN_POSITION <= target_pos <= MAX_POSITION):
                self.get_logger().error(f" 모터 {motor_id}: 범위 초과 ({target_pos})")
                return False
            if not self._write_goal_position_with_recovery(
                motor_id,
                int(target_pos),
                "goal position write",
            ):
                return False
            self.get_logger().info(f" 모터 {motor_id}: 목표 위치 {target_pos} 설정")

        start_time = time.time()
        reached = {motor_id: False for motor_id in self.motor_ids}

        while not all(reached.values()):
            if time.time() - start_time > timeout:
                self.get_logger().error(f" Timeout ({timeout}초): 모든 모터가 도달하지 못함")
                return False

            states = self.get_realtime_position()
            for motor_id in self.motor_ids:
                if not reached[motor_id]:
                    current_pos = states[motor_id]["position"]
                    target_pos = target_positions[motor_id]
                    if abs(current_pos - target_pos) <= self.position_tolerance:
                        reached[motor_id] = True
                        self.get_logger().info(
                            f" 모터 {motor_id}: 도달 (위치={current_pos})"
                        )
            time.sleep(check_interval)

        self.get_logger().info(" 모든 모터가 목표 위치에 도달")
        self.get_logger().info("=" * 70)
        return True

    def move_to_position_smooth(
        self,
        target_positions: Dict[int, int],
        move_time: float = 0.6,
        rate_hz: float = 20.0,
    ) -> bool:
        """
        Minimum jerk interpolation으로 부드럽게 목표 위치까지 이동.
        """
        for motor_id, target_pos in target_positions.items():
            if motor_id not in self.motors:
                self.get_logger().error(f"Motor {motor_id} not found")
                return False
            if not (MIN_POSITION <= target_pos <= MAX_POSITION):
                self.get_logger().error(
                    f"Motor {motor_id}: Position out of range ({target_pos})"
                )
                return False

        current_states = self.get_realtime_position()
        current_positions = {
            mid: current_states[mid]["position"]
            for mid in target_positions.keys()
        }

        steps = max(1, int(move_time * rate_hz))
        dt = 1.0 / rate_hz

        for step in range(steps + 1):
            r = step / steps
            s = minimum_jerk_scalar(r)

            for motor_id, target_pos in target_positions.items():
                current_pos = current_positions[motor_id]
                interpolated_pos = int(current_pos + (target_pos - current_pos) * s)
                interpolated_pos = max(MIN_POSITION, min(MAX_POSITION, interpolated_pos))

                if not self._write_goal_position_with_recovery(
                    motor_id,
                    interpolated_pos,
                    "goal position write",
                ):
                    return False

            time.sleep(dt)

        for motor_id, target_pos in target_positions.items():
            if not self._write_goal_position_with_recovery(
                motor_id,
                int(target_pos),
                "final goal position write",
            ):
                return False

        return True

    def wait_until_stable(
        self,
        target_positions: Dict[int, int],
        position_tolerance: int = 10,
        velocity_threshold: int = 5,
        stable_duration: float = 0.25,
        timeout: float = 0.4,
    ) -> bool:
        """
        목표 위치 근처에서 일정 시간 유지되는지 확인.
        """
        start_time = time.time()
        stable_start = None

        while time.time() - start_time < timeout:
            states = self.get_realtime_position()
            all_stable = True

            for motor_id, target_pos in target_positions.items():
                current_pos = states[motor_id]["position"]
                current_vel = states[motor_id]["velocity"]

                pos_error = abs(current_pos - target_pos)
                vel_mag = abs(current_vel) if current_vel is not None else None
                velocity_ok = vel_mag is None or vel_mag <= velocity_threshold

                if not (pos_error <= position_tolerance and velocity_ok):
                    all_stable = False
                    break

            if all_stable:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= stable_duration:
                    self.get_logger().info("All motors stabilized")
                    return True
            else:
                stable_start = None

            time.sleep(0.05)

        self.get_logger().warning(
            f"Stabilization timeout ({timeout}s). "
            "Motor feedback indicates movement still ongoing. Continuing with mission..."
        )
        return False

    def move_sequence(
        self,
        positions_list: List[Dict[int, int]],
        hold_time: float = 2.0,
        timeout_per_move: float = 10.0,
    ):
        """
        순차적 위치 제어.
        """
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info(f"순차 위치 제어 시작 (총 {len(positions_list)}개 위치)")
        self.get_logger().info("=" * 70)

        for idx, target_pos in enumerate(positions_list, 1):
            self.get_logger().info(f"\n[{idx}/{len(positions_list)}] 위치 이동")
            if not self.move_to_position(target_pos, timeout=timeout_per_move):
                self.get_logger().error(f" 위치 {idx} 제어 실패")
                return False

            self.get_logger().info(f"  {hold_time}초간 유지...")
            start_hold = time.time()
            while time.time() - start_hold < hold_time:
                self.get_realtime_position()
                self.print_realtime_feedback()
                time.sleep(0.5)

        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info(" 모든 순차 제어 완료!")
        self.get_logger().info("=" * 70)
        return True

    def shutdown(self):
        """모터 토크 비활성화 및 포트 닫기."""
        self.get_logger().info("\n시스템 종료 중...")

        for motor_id in self.motor_ids:
            self._write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "torque off")
            self.get_logger().info(f" 모터 {motor_id}: 토크 비활성화 요청")

        if hasattr(self, "port_handler"):
            self.port_handler.closePort()
