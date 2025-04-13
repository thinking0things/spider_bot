#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo motor settings
#define SERVOMIN 225  // PWM value for 70°
#define SERVOMAX 425  // PWM value for 110°
#define CENTER 325    // PWM value for 90°
#define NUM_SERVOS 8  // Number of servos
#define PERIOD 2000   // Oscillation period in milliseconds
#define DELAY_TIME 50 // Delay for smooth movement

// Ultrasonic sensor settings
#define TRIG_PIN 18  // Trigger pin
#define ECHO_PIN 19  // Echo pin

float off_set[8] = {150.0, 50.0, -150.0, -50.0, -150.0, -50.0-20., 150.0, 30.0};  
float phase[8] = {0.0, PI / 2., 0.0, PI / 2., 0.0, PI / 2., 0.0, PI / 2.};  

float amp_for[8] = {1.0, -1., 1.0, -1., 1.0, 1.0, 1.0, 1.};  
float amp[8] = {1.0, -1., 1.0, -1., 1.0, 1.0, 1.0, 1.};  
float amp_right[8] = {1.0, -1., 1.0, 1., 1.0, 1.0, 1.0, -1.};  

float pulselen[8] = {.0, 0., .0, 0., .0, .0, .0, 0.};  
float X[8] = {.0, 0., .0, 0., .0, .0, .0, 0.};  

float direction = 1;

unsigned long previousMillis = 0;
unsigned long oldMillis = 0;

const long interval = 200;  // Interval to measure distance in milliseconds

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);  
  delay(10);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for (int servo = 0; servo < NUM_SERVOS; servo++) {
    pulselen[servo] =  CENTER + off_set[servo];
    amp[servo] = amp_for[servo];
    pwm.setPWM(servo, 0, pulselen[servo]);
  }
    
  delay(2000);
}

const float FREQUENCY = .004;
const float AMPLITUDE = 50.;
unsigned long currentMillis = millis();
unsigned long innerTime = millis();

void loop() {
  oldMillis = currentMillis;
  currentMillis = millis();
  innerTime = innerTime + direction * (currentMillis - oldMillis); 

  for (int servo = 0; servo < NUM_SERVOS; servo++) {
    X[servo] = pulselen[servo] - CENTER + off_set[servo];
    pulselen[servo] = CENTER + off_set[servo] + amp[servo] * AMPLITUDE * cos(FREQUENCY * innerTime + phase[servo]);
    pwm.setPWM(servo, 0, pulselen[servo]);
  }

  // Print pulselen[0] and pulselen[1] for the Serial Plotter
  Serial.print("pulselen0 ");
  Serial.print(pulselen[0]);
  Serial.print(" pulselen1 ");
  Serial.println(pulselen[1]); // Ensure newline for the Serial Plotter

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    unsigned int distance = duration * 0.034 / 2;
    
    direction = 1.;

    if (distance > 30) {
      for (int servo = 0; servo < NUM_SERVOS; servo++) {
        amp[servo] = amp_for[servo];
      }
    } else {
      for (int servo = 0; servo < NUM_SERVOS; servo++) {
        amp[servo] = amp_right[servo];
      }
    }
  }
  
  delay(DELAY_TIME);
}

