#include <Wire.h>
#include <MPU6050.h>

int enA = 9;
int enB = 10;
int in1 = 8;
int in2 = 7;
int in3 = 6;
int in4 = 5;
int trigpin = 12;
int echopin = 13;
int max_speed = 200; // Maximum motor speed

int leftEncoderPin = 2;  // Signal pin for left wheel encoder
int rightEncoderPin = 3; // Signal pin for right wheel encoder
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// IMU setup
MPU6050 mpu;
int16_t ax, ay, az; // Change to int16_t
int16_t gx, gy, gz; // Change to int16_t

void setup() 
{
  Serial.begin(9600);
  Serial.println("Arduino is ready");

  // Motor control pins
  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Encoder pins
  pinMode(leftEncoderPin, INPUT);
  pinMode(rightEncoderPin, INPUT);

  // Attach interrupt service routines (ISR) for encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

  // Initialize IMU
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed");
    while (1);
  }
}

void loop() 
{
  // Measure distance using the ultrasonic sensor
 // int dist = getDistance();
 // Serial.print(dist);
 // Serial.println(" cm");

  // Send encoder and IMU data to ROS every 100ms
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= 100) {
    sendSensorDataToROS();
    lastSendTime = millis();
  }

  // Check for commands from ROS
  readCommand(); 
}

void readCommand()
{
  if (Serial.available())
  {
    // Read the incoming data as a string (in format: "linear_x,angular_z")
    String data = Serial.readStringUntil('\n');
    data.trim(); // Remove any extra spaces or newlines

    int commaIndex = data.indexOf(',');
    if (commaIndex > 0)
    {
      String linearStr = data.substring(0, commaIndex);
      String angularStr = data.substring(commaIndex + 1);

      // Convert to float values
      float linear_x = linearStr.toFloat();
      float angular_z = angularStr.toFloat();

      // Calculate motor speeds based on linear and angular velocities
      int leftSpeed = (linear_x - angular_z) * max_speed;
      int rightSpeed = (linear_x + angular_z) * max_speed;
      if(angular_z > 0)//reduce left turning speed to prevent crazy spins when turning
      {
        rightSpeed = 0;
        leftSpeed = 130;
      }
      else if(angular_z < 0)//reduce right turning speed too
      {
        leftSpeed = 0;
        rightSpeed = 130;
      
      }

      // Constrain motor speeds to be within the allowable range
      leftSpeed = constrain(leftSpeed, -max_speed, max_speed);
      rightSpeed = constrain(rightSpeed, -max_speed, max_speed);

      

      // Move the motors based on calculated speeds
      move(leftSpeed, rightSpeed);
    }
  }
}

void move(int left, int right)
{
  // Handle left motor direction and speed
  if (left < 0)
  {
    left = -left;  // Convert to positive value
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);  // Reverse direction
  }
  else
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);  // Forward direction
  }

  // Handle right motor direction and speed
  if (right < 0)
  {
    right = -right;
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);  // Reverse direction
  }
  else
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);  // Forward direction
  }

  // Apply PWM signals to set motor speed
  analogWrite(enA, left);
  analogWrite(enB, right);
}

long getDistance() 
{
  long duration, distance;

  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  duration = pulseIn(echopin, HIGH);
  distance = (duration * 0.034) / 2;

  return distance;
}

// Interrupt Service Routine (ISR) for the left encoder
void leftEncoderISR()
{
  // Check the motor direction (reverse or forward)
  if (digitalRead(in1) == LOW && digitalRead(in2) == HIGH)
  {
    // Motor is moving in reverse, decrement the encoder count
    leftEncoderCount--;
  }
  else
  {
    // Motor is moving forward, increment the encoder count
    leftEncoderCount++;
  }
}

// Interrupt Service Routine (ISR) for the right encoder
void rightEncoderISR()
{
  // Check the motor direction (reverse or forward)
  if (digitalRead(in3) == LOW && digitalRead(in4) == HIGH)
  {
    // Motor is moving in reverse, decrement the encoder count
    rightEncoderCount--;
  }
  else
  {
    // Motor is moving forward, increment the encoder count
    rightEncoderCount++;
  }
}

// Function to send encoder and IMU data to ROS
void sendSensorDataToROS()
{
  // Get IMU data
  mpu.getAcceleration(&ax, &ay, &az);  // Returns raw accelerometer data as int16_t
  mpu.getRotation(&gx, &gy, &gz);  // Returns raw gyroscope data as int16_t

  // Send the raw encoder and IMU data over serial in the format "left_ticks,right_ticks,ax,ay,az,gx,gy,gz"
  Serial.print(leftEncoderCount);
  Serial.print(",");
  Serial.print(rightEncoderCount);
  Serial.print(",");
  Serial.print(ax);  // Send raw accelerometer data
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);  // Send raw gyroscope data
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.println(gz);
}
