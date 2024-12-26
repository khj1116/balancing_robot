#include <PID_v1.h>
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//------------------------------------------------------------
#define OUTPUT_TEAPOT 1               // Processing을 통해 MPU6050 센서를 Visualize 하고 싶은 경우 1, 아니면 0으로 선언합니다
#define MIN_ABS_SPEED 30              // 모터의 최저속도를 설정합니다.   0 ~ 255 값 중 선택
#define OUTPUT_READABLE_YAWPITCHROLL  // Yaw, Pitch, Roll 값을 얻기 위해 선언(기본정의)
#define INTERRUPT_PIN 2               // MPU6050 센서의 INT 핀이 꽂혀있는 번호를 설정합니다. 보통 2번
#define LED_PIN 13                    // Arudino Uno의 13번핀 LED를 동작 중에 반짝거리게 하려고 선언합니다 
 
//------------------------------------------------------------
//MPU 객체를 선언합니다

MPU6050 mpu;
bool blinkState = false;              // LED를 깜박이기 위한 변수
bool dmpReady = false;                // DMP 초기화가 성공했을 경우 true로 설정
uint8_t mpuIntStatus;                 // MPU로부터 실제 인터럽트 상태 바이트를 저장
uint8_t devStatus;                    // 각 장치 작업 후 반환되는 상태 (0 = 성공, !0 = 오류)
uint16_t packetSize;                  // 예상되는 DMP 패킷 크기 (기본값은 42 바이트)
uint16_t fifoCount;                   // 현재 FIFO에 있는 모든 바이트의 개수
uint8_t fifoBuffer[64];               // FIFO 저장 버퍼

//------------------------------------------------------------
// MPU6050 센서를 통해 쿼터니언과 오일러각, Yaw, Pitch, Roll 값을 얻기 위해 선언합니다

Quaternion q;                         // [w, x, y, z]      쿼터니언 컨테이너
VectorInt16 aa;                       // [x, y, z]         가속도 센서 측정값
VectorInt16 aaReal;                   // [x, y, z]         중력이 제거된 가속도 센서 측정값
VectorInt16 aaWorld;                  // [x, y, z]         세계 좌표계의 가속도 센서 측정값
VectorFloat gravity;                  // [x, y, z]         중력 벡터
float euler[3];                       // [psi, theta, phi] 오일러 각 컨테이너
float ypr[3];                         // [yaw, pitch, roll]요/피치/롤 컨테이너 및 중력 벡터

//------------------------------------------------------------
// Processing으로 MPU6050 센서를 Visualize 하기 위한 변수
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//------------------------------------------------------------
// PID 제어용 변수 선언
double kp = 60.;
double ki = 200.;
double kd = 1.5;
 
double originalSetpoint = 184.0;      //기울일 각도(180도기준, +,-기준으로 설정,최적의 평행각도값을 찾아라~)
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
 

double input, output;                 // PID 제어용 input, output 변수를 선언
 
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT); //DIRECT(반REVERSE),PID_v1.h, 제어기의 출력증가하면 입력도 증가. 
                                      //즉, 목표값에 도달하기 위해서 출력 값이 직접적으로 원래의 입력 값에 적용되는 방식
int ENA = 5;                          // EnA, EnB는 속도제어용(pwm)
int IN1 = 6;                          // IN1,2,3,4는 방향제어용
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);
                                      //1,1 좌측, 우측모터의 최대속도(100%, 0~1사이의 실수값)
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;   // MPU 인터럽트 핀상태 값 저장
 
void dmpDataReady() {
    mpuInterrupt = true;              // DMP 데이터 준비가 완료되었음
}
 
void setup(){
                                      
    Wire.begin();                     // I2C 초기화
    Wire.setClock(400000);            // 400kHz I2C 클락. 컴파일 문제가 발생할 경우 이 줄을 주석 처리합니다.
    Serial.begin(115200);             // 시리얼 통신 초기화
    Serial.println(F("I2C 장치 초기화 중..."));
    mpu.initialize();                 // MPU 초기화
    pinMode(INTERRUPT_PIN, INPUT);    // 인터럽트 핀을 입력으로 설정합니다.
    Serial.println(F("장치 연결 테스트 중..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 연결 성공") : F("MPU6050 연결 실패"));
    Serial.println(F("DMP 초기화 중..."));    // DMP 로드 및 구성
    devStatus = mpu.dmpInitialize();
 
    
    mpu.setXGyroOffset(220);          // 최소 감도를 기준으로 조정한 자이로 오프셋 입력
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);        // 테스트 칩의 기본값은 1688입니다.
 
 
  
    if (devStatus == 0){              // 작동했는지 확인 (0이면 정상)
        Serial.println(F("DMP 활성화 중..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("인터럽트 감지 활성화 중 (Arduino 외부 인터럽트 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP 준비 완료! 첫 번째 인터럽트를 기다립니다..."));
        dmpReady = true;
                                        // 나중에 비교할 수 있도록 예상 DMP 패킷 크기값 로드
        packetSize = mpu.dmpGetFIFOPacketSize();  // 42바이트: 쿼터니언(16바이트), 가속도계 데이터(6바이트), 자이로스코프 데이터(6바이트)
  
        pid.SetMode(AUTOMATIC);         // PID 모드를 자동으로 설정
        pid.SetSampleTime(10);          // 샘플 시간 설정
        pid.SetOutputLimits(-255, 255); // 출력 한계 설정
    }
    else{   
        // MPU6050 센서가 오작동한 경우
        // 오류!
        // 1 = 초기 메모리 로드 실패
        // 2 = DMP 구성 업데이트 실패
        // (고장 나면 대부분 코드는 1입니다)
        Serial.print(F("DMP 초기화 실패 (코드 "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 
    // 로봇이 작동 중 13번 LED를 깜빡이기 위해 OUTPUT으로 초기화합니다.
    pinMode(LED_PIN, OUTPUT);
}

  
void loop(){
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize){  //인터럽트 발생안하고, fifoCount값이 부족하다는건 MPU6050으로부터 데이터가 없음
        pid.Compute();                                // 루프를 돌면서 PID 값을 업데이트
        motorController.move(output, MIN_ABS_SPEED);  // PID 연산으로 나온 output 값을 motorController로 전송 (모터 제어)
    }

    mpuInterrupt = false;                             //MPU6050데이터 준비됐음(인터럽트가 걸렸다)
    mpuIntStatus = mpu.getIntStatus();
   
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024){  //mpuStatus==0x10 fifo overflow
        mpu.resetFIFO();                              //버퍼초기화
        Serial.println(F("FIFO 오버플로우!"));
    }

    else if (mpuIntStatus & 0x02){                    //데이터 수신준비

        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); //1패킷이 수신될때까지 실행
        mpu.getFIFOBytes(fifoBuffer, packetSize);     //패킷크기만큼 FIFO에서 읽어서 fifoBuffer에 로드

        fifoCount -= packetSize;                      //읽어온만큼의 패킷크기를 삭제                     
                                                      //loop문장실행할때 fifo영역이 한패킷이상 더 읽을수 있도록 하기 위함    
        mpu.dmpGetQuaternion(&q, fifoBuffer);         //step1 : 쿼터니언 : 회전 정보(4차원:w,x,y,z). 기울어짐, 롤, 피치를 계산하는 데 필요한 기초 정보를 제공해줌
        mpu.dmpGetGravity(&gravity, &q);              //step2 : 중력벡터 : 현재의 기울기(x,y,z 축애한 힘). 기체가 수직인지, 기울어져 있는지 등을 판단하는 데 사용. 
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);    //step3 : 위 두 값을 활용해서 Yaw, Pitch,Roll값 받음

#ifdef OUTPUT_READABLE_YAWPITCHROLL                   // 센서를 통해 구한 Yaw, Pitch, Roll 값을 Serial Monitor에 표시
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);              // Yaw를 도 단위로 변환하여 출력
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);              // Pitch를 도 단위로 변환하여 출력
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);            // Roll을 도 단위로 변환하여 출력
#endif
                                                      // PID 제어를 하기 위해 input 변수에 Pitch 값을 넣음
        input = ypr[1] * 180/M_PI + 180;              // Pitch 값을 0~360 범위로 변환하여 input에 저장(PID제어에서 필요한 실제값)

#ifdef OUTPUT_TEAPOT  // Processing으로 MPU6050 센서의 움직임을 시각화하기 위한 코드.
        // InvenSense Teapot 데모 형식으로 쿼터니언 값을 표시합니다:
        teapotPacket[2] = fifoBuffer[0]; // Q0
        teapotPacket[3] = fifoBuffer[1]; // Q1
        teapotPacket[4] = fifoBuffer[4]; // Ax
        teapotPacket[5] = fifoBuffer[5]; // Ay
        teapotPacket[6] = fifoBuffer[8]; // Gx
        teapotPacket[7] = fifoBuffer[9]; // Gy
        teapotPacket[8] = fifoBuffer[12]; // 온도 데이터
        teapotPacket[9] = fifoBuffer[13]; // 0으로 패딩
        Serial.write(teapotPacket, 14); // 패킷 전송
        teapotPacket[11]++; // 패킷 카운트, 의도적으로 0xFF에서 루프됩니다.
#endif
    }
}


// bool PID::Compute(){
//    if(!inAuto) return false;                // 자동 모드가 아닐 경우 false를 반환하고,종료
//    unsigned long now = millis();            // 현재 시간 설정
//    unsigned long timeChange = (now - lastTime); // 지난 시간과 현재 시간의 차이 계산

//    if(timeChange >= SampleTime){            // 지정한 샘플링 시간 이상이되면, 
//       double input = *myInput;              // 입력 값(실제값)
//       double error = *mySetpoint - input;   // 목표값과 현재 입력 값의 차이
//       double dInput = (input - lastInput);  // 현재 입력 값과 이전 입력 값의 차이

//       outputSum += (ki * error);            // 누적 출력 값을 업데이트

//       if(!pOnE) outputSum -= kp * dInput;   // 비례항을 입력값에 비례해서 결정('P_ON_E'로 지정되어서, 실행 안함)
//       // 출력 값의 한계 설정.
//       if(outputSum > outMax) outputSum = outMax;      // 누적 출력 값이 최대값을 초과하면 최대값으로 설정.
//       else if(outputSum < outMin) outputSum = outMin; // 누적 출력 값이 최소값 이하로 내려가면 최소값으로 설정.

//       double output;                        // 최종 출력 값을 저장할 변수.
//       if(pOnE) output = kp * error;         // P_ON_E가 지정된 경우, (output=Kp × error)오차에 비례하여 출력 값을 계산
//       else output = 0;                      // P_ON_E가 지정되지 않은 경우 출력 값을 0으로 초기화

//       output += outputSum - kd * dInput;    // 누적 출력 값과 미분 항을 사용하여 최종 출력을 계산
//       // 출력 값의 한계 설정.
//       if(output > outMax) output = outMax;      // 최종 출력 값이 최대값을 초과하면 최대값으로 설정
//       else if(output < outMin) output = outMin; // 최종 출력 값이 최소값 이하로 내려가면 최소값으로 설정.

//       *myOutput = output;                   // 최종 출력 값을 외부 변수에 저장
//       lastInput = input;                    // 현재 입력 값을 다음 번 계산을 위해 저장
//       lastTime = now;                       // 현재 시간을 다음 번 계산을 위해 저장
//       return true;                          // 성공적으로 계산했음을 나타내는 true 반환.
//    }
//    else return false;                       // 샘플링 시간 미만인 경우 false 반환.
// }

 