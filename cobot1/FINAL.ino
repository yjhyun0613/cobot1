// 1. 핀 설정
const int DIR = 4;
const int PUL = 5;
const int TRIG = 9;
const int ECHO = 2;

// 2. 시간 관리 및 상태 변수
unsigned long prevStepMicros = 0;   
unsigned long prevSensorMillis = 0; 
bool pulseState = LOW;              
bool isMotorRunning = false;  // [수정] 처음에는 모터 정지 상태로 대기
bool objectDetected = false;

// [추가] 전체 시스템 가동 여부 플래그
bool systemStarted = false; 

// 3. 센서 데이터 및 인터럽트 변수
volatile unsigned long duration = 0;
volatile unsigned long startTime = 0;
volatile bool newData = false;

// 4. 설정값
int beltSpeed = 800;          
const int stopDistance = 10;  
const int startDistance = 20; 

void echoInterrupt() {
  if (digitalRead(ECHO) == HIGH) {
    startTime = micros();
  } else {
    duration = micros() - startTime;
    newData = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // [추가] 시리얼 읽기 지연 방지
  
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  digitalWrite(DIR, HIGH); 
  
  attachInterrupt(digitalPinToInterrupt(ECHO), echoInterrupt, CHANGE);
  
  Serial.println("ARDUINO_READY_WAITING_FOR_00");
}

void loop() {
  // [핵심] 파이썬으로부터 시리얼 데이터 수신 대기
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); 
    
    // "00" 신호가 들어오면 시스템 시작!
    if (cmd == "00" && !systemStarted) {
      systemStarted = true;
      isMotorRunning = true; 
      Serial.println("GO"); 
    }
  }

  // 시스템 시작 전이면 아래 모터/센서 로직은 무시하고 대기
  if (!systemStarted) {
    return;
  }

  // --- 기존 모터 구동 및 센서 로직 ---
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  if (isMotorRunning) {
    if (currentMicros - prevStepMicros >= beltSpeed) {
      prevStepMicros = currentMicros;
      pulseState = !pulseState;
      digitalWrite(PUL, pulseState);
    }
  }

  if (currentMillis - prevSensorMillis >= 200) {
    prevSensorMillis = currentMillis;
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
  }

  if (newData) {
    long distance = duration * 0.034 / 2;
    newData = false;

    if (distance > 0 && distance < 100) {
      if (distance <= stopDistance && !objectDetected) {
        isMotorRunning = false;
        objectDetected = true;
        Serial.println("STOP"); 
      } 
      else if (distance >= startDistance && objectDetected) {
        isMotorRunning = true;
        objectDetected = false;
        Serial.println("GO"); 
      }
    }
  }
}
