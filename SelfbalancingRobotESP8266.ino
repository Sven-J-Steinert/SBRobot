/*
 
 ==================================================================
 ===                    SELFBALANCING ROBOT                     ===
 ==================================================================
 This Code is based on the work of Matthew
 http://www.brokking.net/yabr_main.html
 and Jeff Rowberg
 https://github.com/jrowberg
 modified and implemented by Niklas B. and Sven S.
 created for TU Darmstadt in 2019
 ==================================================================
*/
 
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "ESP8266WiFi.h"
#include "WiFiUdp.h"
 
MPU6050 mpu;
 
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION
 
// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER
 
// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
 
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL
 
// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL
 
// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT
 
 
// ================================================================
// ===                        Pin setting                       ===
// ================================================================

// GYRO  SCL=D1  SDA=D2
#define INTERRUPT_PIN 2 // D4 but i didnt connect the INT pin from the gyro

// MOTOR 1
#define DIR1 0    // D3
#define STEP1 16  // D0

// MOTOR 2
#define DIR2 12   // D6
#define STEP2 14  // D5

// MULTIPLEX
const int S0 = 13;  // D7       
const int S1 = 15;  // D8   

 
// ================================================================
// ===                         Variables                        ===
// ================================================================
 
// Wifi UDP
const char *ssid = "Robot_02";
const char *pass = "letmeaccessyourdata";
unsigned int localPort = 2000; // local port to listen for UDP packets
IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);
WiFiUDP udp;
byte incomingPacket[1];  // buffer for incoming packets
char controll = 0;       // data variable
 
 
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
 
double angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
double pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
 
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
 
byte start;
 
//Various settings
double pid_p_gain = 4;                                       //Gain setting for the P-controller (15)
double pid_i_gain = .5;                                      //Gain setting for the I-controller (1.5)
double pid_d_gain = 5;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)
 
unsigned long loop_timer;
 
long last;
long delta;
 
//Timer
void initTimer();
void timerCall();
void initGPIO();
void initMultiplexer();
void readMultiplexer(double *p, double *i, double *d, double *v);
void initNetwork();
void readDir();
 
 
 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 
 
 
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
 
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       // Wire.begin(D2,D1);
        Wire.begin(19,18);
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 
    Serial.begin(115200);
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
 
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
 
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
/*
    // Sören
    mpu.setXGyroOffset(-37);
    mpu.setYGyroOffset(-5);
    mpu.setZGyroOffset(-62);
    mpu.setXAccelOffset(-4101);
    mpu.setYAccelOffset(-977);
    mpu.setZAccelOffset(621);
*/
 
    // Ella
    mpu.setXGyroOffset(81);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(25);
    mpu.setXAccelOffset(-515);
    mpu.setYAccelOffset(-999);
    mpu.setZAccelOffset(659);
 
 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 
    initGPIO();
    initNetwork();
    initTimer();
}
 
 
 
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
 
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
       
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif
 
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            angle_gyro = ypr[1] * 180/M_PI -88 -5.2;
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
        #endif
 
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif
 
        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
   
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
 
 
    }
    //angle_gyro = ypr[1] * 180/M_PI -90;
    //Serial.print(angle_gyro);
 
    readDir();
 
    if(start == 0 && angle_acc > -5 && angle_acc < 5){                         //If the accelerometer angle is almost 0 eigentlich 0.5
        start = 1;                                                                 //Load the accelerometer angle in the angle_gyro variable
                                                                               //Set the start variable to start the PID controller
  }
 
   // read PID values
      double p = 0;
      double i = 0;
      double d = 0;
      double v = 0;
      readMultiplexer(&p,&i,&d, &v);
      /*
      //  Sören
      pid_p_gain = 10+15*p/1024;
      pid_i_gain = 0.8+.7*i/1024;
      pid_d_gain = 15+10*d/1024;
    */
      //  Ella
      pid_p_gain = 15+15*p/1024;
      pid_i_gain = 0.6+.7*i/1024;
      pid_d_gain = 20+10*d/1024;
    //Serial.println(pid_setpoint);
    Serial.print(" P=");Serial.print(pid_p_gain);Serial.print(" I=");Serial.print(pid_i_gain);Serial.print(" D=");Serial.println(pid_d_gain);
    //PID
    pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
    //Serial.print(" "); Serial.print(pid_error_temp);
    if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;
 
    pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
    if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
    else if(pid_i_mem < -400)pid_i_mem = -400;
    //Calculate the PID output value
    pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
    if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
    else if(pid_output < -400)pid_output = -400;
 
    pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop
 
    if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced
   
    if(angle_gyro > 30 || angle_gyro < -30 || start == 0 ){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }
   
   
    pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
    pid_output_right = pid_output;
 
    // REMOTE
   
    if(controll=='8'||controll=='1' ||controll=='5')
    {
      if(pid_setpoint < 2)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
      if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
    }
    else if(controll=='7'||controll=='3'||controll=='6')
    {
      if(pid_setpoint > -2)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
      if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    }
    else
    {
      pid_setpoint = 0;
    }
   
    if(controll=='5'||controll=='2'||controll=='6')
    {
      pid_output_left -= 60;
      pid_output_right += 60;
 
    }
    else if(controll=='8'||controll=='4'||controll=='7')
    {
      pid_output_left +=60;
      pid_output_right -= 60;
    }
   
   
    // PID MAIN
    if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
    }
 
    if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
    else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;
 
    if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
    else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;
 
    //Calculate the needed pulse time for the left and right stepper motor controllers
    if(pid_output_left > 0)left_motor = 400 - pid_output_left;
    else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
    else left_motor = 0;
 
    if(pid_output_right > 0)right_motor = 400 - pid_output_right;
    else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
    else right_motor = 0;
 
    throttle_left_motor = left_motor;
    throttle_right_motor = right_motor;
 
   
   
    while(loop_timer > micros())
    {
        yield();
    }
    loop_timer += 4000;
 
}
 
 
void initTimer()
{
  timer1_isr_init();
  timer1_attachInterrupt(timerCall);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); // 5 ticks/us -> 100 ticks für 20us
  //timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(200);
  //timer1_write(ESP.getCycleCount()+1600L);
}
 
 
 
void ICACHE_RAM_ATTR timerCall()
{
  noInterrupts();
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      digitalWrite(DIR1, HIGH);                                              //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else  digitalWrite(DIR1, LOW);                                           //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1){digitalWrite(STEP1, HIGH);}        //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2){digitalWrite(STEP1, LOW);}        //Set output 2 low because the pulse only has to last for 20us
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
   
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      digitalWrite(DIR2, LOW);                                              //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else if (throttle_right_motor_memory > 0)  digitalWrite(DIR2, HIGH);                                          //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)digitalWrite(STEP2, HIGH);      //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)digitalWrite(STEP2, LOW);       //Set output 4 low because the pulse only has to last for 20us
 
  long temp = micros();
  delta = temp - last;
  last = temp;
  interrupts();
}
 
 
void initGPIO()
{
  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(A0, INPUT);
}
 
void readMultiplexer(double *p, double *i, double *d, double *v)
{
  //P
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  *p = analogRead(A0);
 
  //I
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  *i = analogRead(A0);
 
  //D
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  *d = analogRead(A0);
 
}
 
IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);
 
void initNetwork()
{
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 // udp.begin(2000);
}
 
void readDir()
{
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    int len = udp.read(incomingPacket, 7);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    char c = incomingPacket[0];
   
    if(0>=0&&c<='8')
    {
      controll = c;
    }
    else if(c == 'c')
    {
     
    }
    else if(c == 'z')
    {
     
    }
    Serial.print(" ");
    Serial.print(controll);}}
