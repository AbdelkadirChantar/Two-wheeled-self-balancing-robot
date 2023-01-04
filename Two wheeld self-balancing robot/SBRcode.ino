
#include <Wire.h>


float prevgy = 0;



const int MPU_ADDRESS = 0x68; // MPU6050 I2C address

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;

float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

float elapsedTime, currentTime, previousTime,  PreviousTime, CurrentTime, ElapsedTime; // timeelapsed for complementary filter

int kp, kd, ki;
char com;

int c = 0;
int l1 = 3;
int r1 = 5;
int l2 = 6;
int r2 = 9;

/* Set these to your desired credentials. */
float Kp;
float Kd;
float Ki;
float desiredAngle;
float targetAngle ;
int maxm = 220;
int minm = 0;
#define sampleTime  0.01

int FWD;
int BWD;
int Right;
int Left;
/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
uint32_t timer;

volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile byte count = 0;

void setup() {
  
  Serial.begin(9600);


  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDRESS);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  pinMode(l1, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r2, OUTPUT);

  digitalWrite(l1, LOW);
  digitalWrite(r1, LOW);
  digitalWrite(l2, LOW);
  digitalWrite(r2, LOW);

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  /*  Wire.beginTransmission(MPU);
    Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);

    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);


    //Call this function if you need to get the IMU error values for your module*/
  //while(true) calculate_IMU_error();

  // delay(20);
}



void loop() {
  

  MPU_read_accel_data();
  MPU_read_gyro_data();


  // calculate time elapsed since last time we were here
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds


  // Complementary filter - combine acceleromter and gyro angle values
  //roll  = 0.92 * (roll  + (GyroX * elapsedTime)) + 0.08 * accAngleX;
  currentAngle =  0.92 * (currentAngle + (GyroY * elapsedTime)) + 0.08 * accAngleY;// complementary filter 
  //yaw   = gyroAngleZ;
  /*Serial.print(GyroY);
    Serial.print("   /  ");
    Serial.println(currentAngle);
  */

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  //gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  //gyroAngleZ += GyroZ * elapsedTime;


  /*
    Serial.print("    ");
    Serial.print("yaw:");
    Serial.println(yaw);
  */



  /*Serial.print(" ");
    /* made by me
    // Print the values on the serial monitor
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(accAngleY);
    Serial.print(" ");
    Serial.println(gyroAngleY);
  */

  //load all bytes from serial bus


  //sampleTime
  periodicFunc();
}
unsigned int counter = 0;
unsigned int lastTempUpdate = 0;
bool periodicFunc() {
  if ((millis() - lastTempUpdate) > (sampleTime * 1000)) {
    lastTempUpdate = millis();

    //Serial.print("currentAngle :");Serial.println(currentAngle );
    error = currentAngle - targetAngle;
    //Serial.print("error:");Serial.println(error);

    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);
    //Serial.print("error sum:");Serial.println(errorSum);

    //calculate output from P, I and D values
    motorPower = (Kp * (error)) + ( Ki * (errorSum) * sampleTime) + (Kd * (currentAngle - prevAngle) / sampleTime);
    prevAngle = currentAngle;

    motorPower = constrain(motorPower, -maxm, maxm);

    // Serial.print("motor power:");Serial.println(motorPower);


    if (motorPower >= 0) {
      moveFW(motorPower, 100, 100);

    }
    else if (motorPower < 0) {
      moveBW(-motorPower, 100, 100);

    }
    else
    {
      stopMotors();
    }
    counter++;

    





    if (counter % 100 == 0)
    {
      Serial.print("Angle: ");
      Serial.println(currentAngle);
      Serial.print("Output: ");
      Serial.println(motorPower);

    }
  }

}






void MPU_read_accel_data()
{
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  // Calculating Roll and Pitch from the accelerometer data
  // accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI)      - (-1.44);  // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - (-4.36); // AccErrorY ~(-1.58)

}


void MPU_read_gyro_data()
{
  // === Read gyroscope data === //
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  //GyroX = GyroX - (30.27); // GyroErrorX = ~ (-2.12)
  GyroY = GyroY - (-3.17);  // GyroErrorY = ~ (4.12)
  // GyroZ = GyroZ - (-2.86);  // GyroErrorZ = ~ (1.20)
  //GyroY = 0.8 * GyroY + 0.2 * prevgy;
  //prevgy = GyroY;
}


void calculate_IMU_error()
{

  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 500)
  {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    //  AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  //Divide the sum by 200 to get the error value
  //AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;

  c = 0;

  // Read gyro values 200 times
  while (c < 800)
  {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);

    c++;

  }

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print the error values on the Serial Monitor

  Serial.print("AccErrorX: ");
  Serial.print(AccErrorX);
  Serial.print(" | AccErrorY: ");
  Serial.print(AccErrorY);
  Serial.print(" | GyroErrorX: ");
  Serial.print(GyroErrorX);
  Serial.print(" | GyroErrorY: ");
  Serial.print(GyroErrorY);
  Serial.print(" | GyroErrorZ: ");
  Serial.println(GyroErrorZ);


  delay(1000);
}

void moveFW(int Speed, int left, int right) {
  if (Speed < minm && Speed != 0.00) {
    Speed = minm;
  }
  int vLeft = ((Speed * left) / 100);
  int vRight = (Speed * right) / 100;
  analogWrite(l1, vLeft);
  analogWrite(r1, 0);
  analogWrite(l2, 0);
  analogWrite(r2, vRight);

}
void moveBW(int Speed, int left, int right) {
  if (Speed < minm && Speed != 0.00) {
    Speed = minm;
  }
  int vLeft = (Speed * left) / 100;
  int vRight = (Speed * right) / 100;
  analogWrite(l2, vLeft);
  analogWrite(r2, 0);
  analogWrite(l1, 0);
  analogWrite(r1, vRight);
}
void stopMotors() {
  analogWrite(l1, 0);
  analogWrite(r1, 0);
  analogWrite(l2, 0);
  analogWrite(r2, 0);

}







