#include <SparkFun_TB6612.h>

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

#define LED_BUILTIN 2

#define AIN1 22
#define AIN2 21
#define PWMA 18
#define BIN1 2
#define BIN2 4
#define PWMB 19
#define STBY 5

#define KP 3.0                  // Ganho proporcional (ajustável)
#define KD 1.5                // Ganho derivativo (ajustável)
#define M1_minimum_speed 45   // Velocidade mínima do Motor1
#define M2_minimum_speed 45   // Velocidade mínima do Motor2
#define M1_maximum_speed 200  // Velocidade máxima do Motor1
#define M2_maximum_speed 200  // Velocidade máxima do Motor2
#define MIDDLE_SENSOR 5       // Número do pino do sensor central
#define TIMEOUT 1500          // Tempo de espera em microssegundos para que as saídas dos sensores fiquem baixas
#define DEBUG 0

const int buttonPin = 15;
const int ledPin = 2;
int buttonState = 1;

QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int lastError = 0;
int last_proportional = 0;
int integral = 0;

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 33, 25, 26, 27, 14 }, SensorCount);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  buttonState = digitalRead(buttonPin);
  brake(motor1, motor2);

  while (buttonState == HIGH) {
    buttonState = digitalRead(buttonPin);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }

  manual_calibration();
  buttonState = 1;
}

void readSensors() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i];
  }

  int error = position - 2000;
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_minimum_speed + motorSpeed;
  int rightMotorSpeed = M2_minimum_speed - motorSpeed;

  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int motor1speed, int motor2speed) {
  if (motor1speed > M1_maximum_speed) motor1speed = M1_maximum_speed;
  if (motor2speed > M2_maximum_speed) motor2speed = M2_maximum_speed;
  if (motor1speed < 0) motor1speed = 0;
  if (motor2speed < 0) motor2speed = 0;
  motor1.drive(motor1speed);
  motor2.drive(motor2speed);
}

void loop() {
  waiting_start();
  readSensors();
}

void waiting_start() {
  while (buttonState == HIGH) {
    buttonState = digitalRead(buttonPin);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void manual_calibration() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i];
  }

  for (uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.maximum[i];
  }
}
