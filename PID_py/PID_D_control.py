import time

class DController:
    def __init__(self, Kd, target_value):
        self.Kd = Kd                                    # 미분 게인
        self.target_value = target_value                # 목표값
        self.previous_error = 0                         # 이전 오차 값 초기화
        self.previous_time = time.time()                # 이전 시간 기록

    def calculate(self, current_value):
        current_time = time.time()                      # 현재 시간 기록
        time_delta = current_time - self.previous_time  # 시간 차 계산

        if time_delta == 0:                             # 시간 차가 0이면 나누기 방지
            return 0                                    # 미분 출력은 0으로 설정
        
        error = self.target_value - current_value       # 현재 오차 계산
        derivative = (error - self.previous_error) / time_delta  # 오차 변화율 계산
        output = self.Kd * derivative                   # 미분 제어 출력 계산
        
        self.previous_error = error                     # 이전 오차 업데이트
        self.previous_time = current_time               # 이전 시간 업데이트
        
        return output                                   # 출력값 반환

if __name__ == "__main__":                              # D 제어 예시

    target_value = 1000                                 # 목표값 (예: 1000 RPM)
    Kd = 0.1                                            # 미분 게인 설정
    current_value = 800                                 # 현재 값 (초기값)
    controller = DController(Kd, target_value)          # D 제어기 생성

    for _ in range(100):                                # 100번 반복하며 제어
        control_output = controller.calculate(current_value)  # D 제어 출력 계산
        print(f"Current value: {current_value}, Control output: {control_output}")  # 현재 값과 제어 출력 출력
        current_value += control_output * 0.1           # D 제어 출력에 따라 현재 값 업데이트
        time.sleep(0.1)                                 # 0.1초 대기 (시뮬레이션 시간 반영)
