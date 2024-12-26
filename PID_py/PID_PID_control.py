import time

class PIDController:
    def __init__(self, K_p, K_i, K_d, target_value, time_step=0.1, max_integral=10.0, min_integral=-10.0):
        self.K_p = K_p                            # 비례 게인
        self.K_i = K_i                            # 적분 게인
        self.K_d = K_d                            # 미분 게인
        self.target_value = target_value          # 목표값 (예: 1000 RPM)
        self.current_value = 800                  # 초기 현재 값 (예: 800 RPM)
        self.previous_error = 0                   # 이전 오차
        self.integral = 0                         # 적분값 초기화
        self.time_step = time_step                # 시간 간격
        self.max_integral = max_integral          # 적분 상한
        self.min_integral = min_integral          # 적분 하한
    
    def calculate_pid(self):
        error = self.target_value - self.current_value  # 현재 오차 계산

        # 적분 값 계산 및 제한
        self.integral += error * self.time_step
        self.integral = max(min(self.integral, self.max_integral), self.min_integral)

        # 미분 값 계산
        derivative = (error - self.previous_error) / self.time_step

        # PID 제어 출력 계산
        output = self.K_p * error + self.K_i * self.integral + self.K_d * derivative

        # 이전 오차 업데이트
        self.previous_error = error
        
        return output
    
    def update_speed(self, output):
        # 제어 출력값을 통해 현재 속도 업데이트 (가상 모터 속도 측정)
        self.current_value += output * 0.1  # 제어 출력에 따라 현재 속도 변화 시뮬레이션

def main():
    # PID 제어기 생성
    target_value = 1000  # 목표 속도 (RPM)
    K_p = 0.5            # 비례 게인
    K_i = 0.01           # 적분 게인
    K_d = 0.01           # 미분 게인
    time_step = 0.1      # 시간 간격 (초)
    
    # PID 제어기 인스턴스 생성
    controller = PIDController(K_p, K_i, K_d, target_value, time_step)

    # 시뮬레이션 시간 및 반복 횟수 설정
    simulation_time = 100  # 총 시뮬레이션 시간 (초)
    for _ in range(int(simulation_time / time_step)):  # 100초 동안 0.1초마다 실행
        
        # PID 제어 출력 계산
        control_output = controller.calculate_pid()
        
        # 모터 제어 (현재 속도 업데이트)
        controller.update_speed(control_output)

        # 모터 출력 상태 출력
        print(f"시간: {_ * time_step:.1f}초, 현재 속도: {controller.current_value:.2f}RPM, 출력: {control_output:.2f}")
        
        time.sleep(time_step)  # 0.1초 간격 대기

if __name__ == "__main__":
    main()
