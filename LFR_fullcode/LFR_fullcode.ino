/*
Line Follower ItMuC & SPE Project
By: 
- Bryan Jeremias Legihono (2602147925)
- Muhammad Muflih Fasya (2602151866)

Robot Name: ???
*/

// Define motor control pins
#define AIN1 7
#define AIN2 8
#define BIN1 4
#define BIN2 3

// Define PID Constants
const float Kp = 40;
const float Ki = 0.01;
const float Kd = 8; 

void sensorRead();

void motorCtrl(float);
void forward();
void backward();
void stop();
void turnLeft();
void turnRight();
void turnLeftIn();
void turnRightIn();
void rotateCCW();
void rotateCW();

int errorCal();
uint8_t getSensorByte();

// Variable Initialization
bool IRVal[5] = {0,0,0,0,0};
float I = 0;
float lastErr = 0;

void setup() {
  Serial.begin(9600);

  // Set sensor pins as inputs
  DDRB &= ~(0b00111110); // Pin 9-13 (5 in total)

  // Set motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initial motor state (stop)
  stop();
}

// Main Loop
void loop() {  
  sensorRead();

  // Calculate error (desired position is 0)
  float err = errorCal();

  // Calculate integral
  I += err;

  // Calculate derivative
  float D = err - lastErr;

  // PID output
  float out = Kp * err + Ki * I + Kd * D;

  // Save the error for the next loop
  lastErr = err;

  motorCtrl(out);

  // // Optional: Debugging output
  // Serial.print(" Error: ");
  // Serial.print(err);
  // Serial.print(" Output: ");
  // Serial.println(out);

  // Small delay to prevent overwhelming the serial output
  delay(75);
}

// Calculates error
int errorCal(){
  int error = 0;
  uint8_t byteVal = getSensorByte();

  // Serial.print(byteVal);

  if      (byteVal == 0b00010000) error = -4;
  else if (byteVal == 0b00011000) error = -3;
  else if (byteVal == 0b00011100) error = -3;
  else if (byteVal == 0b00011110) error = -3;
  else if (byteVal == 0b00001000) error = -2;
  else if (byteVal == 0b00001100) error = -1;
  else if (byteVal == 0b00010100) error = -1;
  else if (byteVal == 0b00000100) error = 0;
  else if (byteVal == 0b00000101) error = 1;
  else if (byteVal == 0b00000110) error = 1;
  else if (byteVal == 0b00000010) error = 2;
  else if (byteVal == 0b00001111) error = 3;
  else if (byteVal == 0b00000111) error = 3;
  else if (byteVal == 0b00000011) error = 3;
  else if (byteVal == 0b00000001) error = 4;

  return error;
}

// Get byte data of sensor readings
uint8_t getSensorByte() {
  uint8_t byteVal = 0;

  for(int i = 0; i < 5; i++) {
    byteVal |= (IRVal[i] << i);
  }

  return byteVal;
}

/* Read data from sensor.
  **Note: spesific pins (9,10,11,12,13) & spesific sensor counts (5).
*/
void sensorRead() {
  for(int i = 0; i < 5; i++) {
    IRVal[i] = !(PINB & _BV(i+1));
  }
}

/* Motor Control Function */
void motorCtrl(float out) {
  if (getSensorByte() == 0b00011111) {
    stop();
    return;
  }

  if (out > 0.2) {
    if (out < 100) turnRight(); 
    else rotateCW();
  } else if (out < -0.2) {
    if (out > -100) turnLeft(); 
    else rotateCCW();
  } else {
    forward();
  }
}

/* ={ Movement Functions }= */

// Robot move forward
void forward() {
  // Motor A (left)
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Motor B (right)
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// Robot move backward
void backward() {
  // Motor A (left)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  // Motor B (right)
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// Stops the robot
void stop() {
  // Motor A (left)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Motor B (right)
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Robot turn left and move forward
void turnLeft() {
  // Motor A (left)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Motor B (right)
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// Robot turn left and move forward
void turnRight() {
  // Motor A (left)
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Motor B (right)
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Robot turn left and move backward
void turnLeftIn() {
    // Motor A (left)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  // Motor B (right)
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Robot turn right and move backward
void turnRightIn() {
    // Motor A (left)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Motor B (right)
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void rotateCCW() {
  // Motor A (left)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  // Motor B (right)
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void rotateCW() {
  // Motor A (left)
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Motor B (right)
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

