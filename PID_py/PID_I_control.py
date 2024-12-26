#I제어

import time

class IController:
    def __init__(self, Ki, target_value):
        self.Ki = Ki                        # 적분 게인
        self.target_value = target_value    # 목표값
        self.integral = 0                   # 적분 값 초기화
        self.previous_time = time.time()    # 이전 시간 기록

    def calculate(self, current_value):
        current_time = time.time()
        time_delta = current_time - self.previous_time  # 시간 차 계산
        error = self.target_value - current_value       # 오차 계산
        self.integral += error * time_delta # 적분 값 계산 (시간에 따른 오차 누적)
        output = self.Ki * self.integral    # I 제어에 의한 출력 계산
        self.previous_time = current_time   # 현재 시간을 이전 시간으로 업데이트
        return output

if __name__ == "__main__":                  # I 제어 예시

    target_value = 1000                     # 목표값 예시 (예: 1000 RPM)
    Ki = 0.1                                # 적분 게인 예시
    current_value = 800                     # 현재 값 (초기값)
    controller = IController(Ki, target_value)     # I 제어기 생성

    for _ in range(100):                    # 100번 반복하면서 제어
        control_output = controller.calculate(current_value)
        print(f"Current value: {current_value}, Control output: {control_output}")
        
        current_value += control_output * 0.1   # 예시로 현재 값을 제어 출력에 따라 업데이트
        time.sleep(0.1)                    

