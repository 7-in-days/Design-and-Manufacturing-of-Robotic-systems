#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 다이나믹셀 easy sdk 사용

import rclpy
from rclpy.node import Node
import time
from typing import Dict, List

from dynamixel_easy_sdk import Connector, OperatingMode, DxlRuntimeError

from . import config
from .trajectory import minimum_jerk_scalar

class ContinummController(Node):
    def __init__(self):
        super().__init__('continumm_controller')

        # 포트 설정 및 프로토콜 버전 설정

        self.port_name = '/dev/ttyUSB0'
        self.baudrate = 57600
        self.protocol_version = config.DYNAMIXEL_PROTOCOL_VERSION

        # 모터 설정

        self.motor_ids = config.MOTOR_NUM # config 내부에 지정

        self.motors = {}

        self.motor_states = {
            0: {'position': 0, 'velocity': 0, 'current': 0, 'temperature': 0},
            1: {'position': 0, 'velocity': 0, 'current': 0, 'temperature': 0},
        }

        self.unsupported_feedback_fields = set()

        # Profile 설정 (부드러운 움직임을 위한)
        self.profile_velocity = 100      # 속도 프로필 (0-1023)
        self.profile_acceleration = 50   # 가속도 프로필 (0-32767)
        
        # 위치 도달 판정
        self.position_tolerance = 10     # 목표값±10 범위 내 도달로 판정


        # 연결 시작!

        try:
            Connector.PROTOCOL_VERSION = self.protocol_version
            Connector._packet_handler = None
            self.connector = Connector(self.port_name, self.baudrate) # 내부 sdk에 존재
            self.get_logger().info(
                f"포트 연결 완료: {self.port_name}, "
                f"baudrate={self.baudrate}, "
                f"protocol={self.protocol_version}")
        except Exception as e:
            self.get_logger().error(f"포트 연결 실패: {e}")
            raise


    def check_connecting(self) -> bool:

        # Return 값은 모든 모터 연결 성공

        self.get_logger().info("모터 연결 확인(ping 확인)")
        

        try:
            # Broadcast Ping으로 모든 모터 확인
            found_motors = self.connector.broadcastPing()
            self.get_logger().info(f"발견된 모터 ID: {found_motors}")
            
            # 필요한 모터 확인
            for motor_id in self.motor_ids:
                if motor_id not in found_motors:
                    self.get_logger().error(f" 모터 {motor_id} 찾을 수 없음!")
                    return False
            
            # 모터 객체 생성 및 초기화
            for motor_id in self.motor_ids:
                try:
                    motor = self.connector.createMotor(motor_id)
                    self.motors[motor_id] = motor
                    
                    self.get_logger().info(f" 모터 {motor_id} 생성")
                    self.get_logger().info(f"  - 모델: {motor.model_name}")
                    self.get_logger().info(f"  - 모델 번호: {motor.model_number}")
                    
                except DxlRuntimeError as e:
                    self.get_logger().error(f" 모터 {motor_id} 생성 실패: {e}")
                    return False
            
            # 모터 설정
            for motor_id, motor in self.motors.items():
                try:
                    # 동작 모드 설정 (위치 제어)
                    motor.setOperatingMode(OperatingMode.POSITION)
                    
                    self.get_logger().info("동작 모드 설정 완료")
                    
                    # 토크 활성화
                    motor.enableTorque()
                    self.get_logger().info(f" 모터 {motor_id}: 토크 활성화")
                    
                    # 초기 위치 읽기
                    initial_pos = motor.getPresentPosition()
                    self.motor_states[motor_id]['position'] = initial_pos
                    self.get_logger().info(f" 모터 {motor_id}: 초기 위치 = {initial_pos}")
                    
                except DxlRuntimeError as e:
                    self.get_logger().error(f" 모터 {motor_id} 설정 실패: {e}")
                    return False
                
            
            self.get_logger().info("=" * 60)
            self.get_logger().info(" 모든 모터 연결 및 초기화 완료!")
            self.get_logger().info("=" * 60)
            return True
            
        except Exception as e:
            self.get_logger().error(f" 연결 확인 중 에러: {e}")
            return False
        

    
    # 실시간 위치 피드백

    def _read_optional_feedback(self, motor_id: int, field_name: str, read_func):
        """
        Read an optional sensor field.

        Some Dynamixel Easy SDK model wrappers do not support velocity/current/
        temperature helpers. In that case, keep mission execution alive and use
        None for the unsupported value.
        """
        try:
            return read_func()
        except Exception as e:
            key = (motor_id, field_name)
            if key not in self.unsupported_feedback_fields:
                self.unsupported_feedback_fields.add(key)
                self.get_logger().warning(
                    f"모터 {motor_id}: {field_name} feedback not supported ({e})")
            return None

    def _read_optional_feedback_method(self, motor_id: int, motor, field_name: str, method_name: str):
        read_func = getattr(motor, method_name, None)
        if read_func is None:
            key = (motor_id, field_name)
            if key not in self.unsupported_feedback_fields:
                self.unsupported_feedback_fields.add(key)
                self.get_logger().warning(
                    f"모터 {motor_id}: {field_name} feedback not supported "
                    f"({method_name} not available)")
            return None
        return self._read_optional_feedback(motor_id, field_name, read_func)
    
    def get_realtime_position(self) -> Dict[int, Dict]:
        """
        모든 모터의 실시간 센서 데이터 읽기
        
        Returns:
            Dict: {motor_id: {position, velocity, current, temperature}}
        """
        try:
            for motor_id, motor in self.motors.items():
                # Position feedback is required for movement and stability checks.
                position = motor.getPresentPosition()
                velocity = self._read_optional_feedback_method(
                    motor_id,
                    motor,
                    "velocity",
                    "getPresentVelocity",
                )
                current = self._read_optional_feedback_method(
                    motor_id,
                    motor,
                    "current",
                    "getPresentCurrent",
                )
                temperature = self._read_optional_feedback_method(
                    motor_id,
                    motor,
                    "temperature",
                    "getPresentTemperature",
                )
                
                # 상태 저장
                self.motor_states[motor_id] = {
                    'position': position, # position
                    'velocity': velocity, # velocity
                    'current': current, # current
                    'temperature': temperature, # temperature
                    'timestamp': time.time() # 시간
                }
            
            return self.motor_states
            
        except DxlRuntimeError as e:
            self.get_logger().error(f" 센서 읽기 실패: {e}")
            return self.motor_states
        
    





    # 종료 함수

    def shutdown(self):
        """모터 토크 비활성화 및 연결 해제"""
        self.get_logger().info("\n시스템 종료 중...")
        
        try:
            for motor_id, motor in self.motors.items():
                motor.disableTorque()
                self.get_logger().info(f" 모터 {motor_id}: 토크 비활성화")
        except Exception as e:
            self.get_logger().error(f"종료 중 에러: {e}")
                



