Check out my youtube videos on this topic: 
- https://youtu.be/B26jPX9b1Yo
- https://youtu.be/9KjNRA1rg0Y?si=VYu00zZ4jRWgSpuh

🐛 Oscillating 8-Servo Robot with Obstacle Detection (ESP32 + PCA9685)

This Arduino-based project controls 8 servo motors using the PCA9685 driver to create oscillating movements — like walking or crawling — with obstacle avoidance via an ultrasonic sensor. When an object is detected nearby, the walking pattern changes direction dynamically.

🚀 Features

Controls 8 servo motors smoothly using cosine-based oscillation
Adjusts movement pattern based on distance measured via ultrasonic sensor
Easy to adapt to quadruped, hexapod, or custom robots
Outputs servo pulse values for real-time visualization via Serial Plotter
🧠 How It Works

Servo movement: Each servo oscillates around a center position using a cosine wave, with configurable amplitude, phase, and offset.
Obstacle detection: An HC-SR04 ultrasonic sensor measures distance; if an object is detected closer than 30 cm, the robot switches to a "turn right" pattern.
PWM driver: The Adafruit PCA9685 handles the 50Hz servo signal generation via I2C.
🛠️ Hardware Required

1 × ESP32 Dev Module
1 × PCA9685 16-Channel Servo Driver
8 × SG90 or similar Servo Motors
1 × HC-SR04 Ultrasonic Sensor
Jumper wires, breadboard or chassis
5V external power supply (for servos)
📦 Libraries Used

Install these via the Arduino Library Manager:

#include <Wire.h>  
#include <Adafruit_PWMServoDriver.h>
🔌 Wiring Overview

Component	Pin on ESP32
PCA9685 SDA	GPIO 21
PCA9685 SCL	GPIO 22
HC-SR04 TRIG	GPIO 18
HC-SR04 ECHO	GPIO 19
Servos are connected to PCA9685 channels 0–7.

📋 Setup Instructions

Install libraries using Arduino Library Manager.
Connect components as described above.
Upload the code using Arduino IDE.
Open Serial Plotter at 115200 baud to see servo waveforms.
Watch your robot walk and adapt!
⚙️ Customization

You can tweak the following parameters:

#define SERVOMIN 225  
#define SERVOMAX 425  
#define CENTER   325  
#define PERIOD   2000      // Movement cycle duration  
#define DELAY_TIME 50      // Controls smoothness/speed  
const float AMPLITUDE = 50.;  
const float FREQUENCY = .004;
Modify amp_for[] and amp_right[] to design different gaits.

🧪 Demo Ideas

Add a second ultrasonic sensor for front+side obstacle detection
Use an IMU to stabilize the gait
Swap cosine for learned or dynamic movement patterns
📹 YouTube Tutorial (Optional)

See this robot walk and respond to obstacles on my YouTube channel: Learning Things
💡 License

MIT License
Feel free to use, modify, and share with attribution!

Let me know if you want a version in Italian or if you're adding more features!
