bool PID::Compute()
{
   if(!inAuto) return false;                // 자동 모드가 아닐 경우 false를 반환하고,종료

   unsigned long now = millis();            // 현재 시간 설정
   unsigned long timeChange = (now - lastTime); // 지난 시간과 현재 시간의 차이 계산

   if(timeChange >= SampleTime)             // 지정한 샘플링 시간 이상이되면, 
   {
      double input = *myInput;              // 입력 값(실제값)
      double error = *mySetpoint - input;   // 목표값과 현재 입력 값의 차이
      double dInput = (input - lastInput);  // 현재 입력 값과 이전 입력 값의 차이

      outputSum += (ki * error);            // 누적 출력 값을 업데이트

                                            /* P_ON_M 이 지정된 경우 측정값에 비례항을 추가합니다. */
      if(!pOnE) outputSum -= kp * dInput;   // 비례항을 입력값에 비례해서 결정('P_ON_E'로 지정되어서, 실행 안함)

      // 출력 값의 한계 설정.
      if(outputSum > outMax) outputSum = outMax;      // 누적 출력 값이 최대값을 초과하면 최대값으로 설정.
      else if(outputSum < outMin) outputSum = outMin; // 누적 출력 값이 최소값 이하로 내려가면 최소값으로 설정.

      double output;                        // 최종 출력 값을 저장할 변수.
      if(pOnE) output = kp * error;         // P_ON_E가 지정된 경우, (output=Kp × error)오차에 비례하여 출력 값을 계산
      else output = 0;                      // P_ON_E가 지정되지 않은 경우 출력 값을 0으로 초기화

      /* PID 출력의 나머지 부분을 계산합니다. */
      output += outputSum - kd * dInput;    // 누적 출력 값과 미분 항을 사용하여 최종 출력을 계산

      // 출력 값의 한계 설정.
      if(output > outMax) output = outMax;      // 최종 출력 값이 최대값을 초과하면 최대값으로 설정
      else if(output < outMin) output = outMin; // 최종 출력 값이 최소값 이하로 내려가면 최소값으로 설정.

      *myOutput = output;                   // 최종 출력 값을 외부 변수에 저장

      /* 다음 번 계산을 위해 몇 가지 변수 기억 */
      lastInput = input;                    // 현재 입력 값을 다음 번 계산을 위해 저장
      lastTime = now;                       // 현재 시간을 다음 번 계산을 위해 저장
      return true;                          // 성공적으로 계산했음을 나타내는 true 반환.
   }
   else return false;                       // 샘플링 시간 미만인 경우 false 반환.
}
