
//-----------------------------------------------------------------------------------------------------------------------------------//
//Wheel chair Mobility for post operative patient//
//-----------------------------------------------------------------------------------------------------------------------------------//

//There are 3 modes to control the wheel chair
//1. EEG + Head Gyro - Emotive
//2. Voice Command   - Phone App
//3. Gesture         - Phone App
//4. Webcam          - Eye tracking -> Sleep detection using Webcam
//5. ECG             - Heart rate monitoring


//Additional Sensors Planned
//1. Sonars
//2. Biometric Fingerprint sensor

//-----------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------DMP Init-----------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------//

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
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



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
float ypr_prev[3];
float y_drift;
bool calibrate = 0;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
int j = 1;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void DMPInit()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  //Serial.begin(115200);
  //Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

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

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
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

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void DMPValues()
{

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
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
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (!calibrate)
    {
      Serial.print(ypr[0] * 180 / M_PI); Serial.print("\t"); Serial.print(ypr_prev[0] * 180 / M_PI); Serial.print("\n");
      j++;
      if (j == 50)
      {
        j = 0;

        if ((ypr[0] - ypr_prev[0]) * 180 / M_PI < .02)
        {
          calibrate = 1;
          y_drift = ypr[0];
        }
        else if ((ypr[0] - ypr_prev[0]) * 180 / M_PI > .02)
        {

        }
        ypr_prev[0] = ypr[0];
      }
    }

    else if (calibrate)
    {
      //Serial.println("C");
      ypr[0] = ypr[0] - y_drift;
    }

    //Serial.print("ypr\t");
    //Serial.println(ypr[0] * 180 / M_PI);
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

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);


    //   Serial.print("ypr\t");
    //    Serial.print(ypr[0] * 180/M_PI);
    //  Serial.print("\n");

  }
}


#define MOTOR_SPEED 110/2

int LEDPIN = 13;
void EEG_mode();
void Voice_mode();
void Gesture_mode();

//Left Motors
#define MOTOR_DIR1 12     // Front
#define MOTOR_PWM1 11
#define MOTOR_DIR2 10     // Back
#define MOTOR_PWM2 9
//right Motors
#define MOTOR_DIR3 8      // Front
#define MOTOR_PWM3 7
#define MOTOR_DIR4 6      // Back
#define MOTOR_PWM4 5

#define PWM 50
uint8_t datatype;
uint8_t state;
uint8_t power;
int head_angle;         //value from the EEG Gyro
#define tolerance 15
#define kp 1
int enable = 1;
#define THRESH 50
char voicecommand;
int accx[3];
bool eegenable = 0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);
  CLKPR = 0;
  DMPInit();
  // Set the pins to output.
  pinMode(LEDPIN, OUTPUT);
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  pinMode(MOTOR_DIR3, OUTPUT);
  pinMode(MOTOR_PWM3, OUTPUT);
  pinMode(MOTOR_DIR4, OUTPUT);
  pinMode(MOTOR_PWM4, OUTPUT);
  // And set these to a initial value to make sure.
  digitalWrite(MOTOR_DIR1, LOW);
  digitalWrite(MOTOR_PWM1, LOW);
  digitalWrite(MOTOR_DIR2, LOW);
  digitalWrite(MOTOR_PWM2, LOW);
  digitalWrite(MOTOR_DIR3, LOW);
  digitalWrite(MOTOR_PWM3, LOW);
  digitalWrite(MOTOR_DIR4, LOW);
  digitalWrite(MOTOR_PWM4, LOW);
}
void loop() {
  //left(50);
  //front(50);
  if(eegenable)
  {
    EEG_mode();
  }
  else
  {
   if(Serial2.available())
  {
    datatype = Serial2.read();
    //Serial.print(datatype);
    if (datatype == 'A') {
      while (!Serial2.available());
      accx[0] = Serial2.read() - 100;
      while (!Serial2.available());
      accx[1] = Serial2.read() - 100;
      while (!Serial2.available());
      accx[2] = Serial2.read() - 100;
      if( abs(accx[0]) >= abs(accx[1]))
      {
        if(accx[0] > 0)
        {
          left(50);
          Serial.println("left");
        }
        else {right(50);
        Serial.println("Right");
        }
      }

      else
      {
        if(accx[1] < 0)
        {
          front(50);
          Serial.println("Front");
        }

        else {halt();
        Serial.println("Halt");
        }
      }
      
      Serial.print("AccX[0]: ");
      Serial.println((int)accx[0]);
      Serial.print("AccX[1]: ");
      Serial.println((int)accx[1]);
      Serial.print("AccX[2]: ");
      Serial.println((int)accx[2]);
      Serial.print('\n');
    }
    else if (datatype == 'V') {
      while (!Serial2.available());
      voicecommand = Serial2.read();
      Serial.print(voicecommand);
      switch (voicecommand)
      {
        case ('O'):
        {
          eegenable = 1;
          Serial.println("EEG Enabled");
          break;
        }
        case ('F'):
          {
            front(50);
            Serial.println("Front");
            break;
          }
        case ('B'):
          {
            halt();
            break;
          }
        case ('L'):
          {
            left(50);
            Serial.println("Left");
            break;
          }
        case ('R'):
          {
            right(50);
            Serial.println("Right");
            break;
          }
        case ('S'):
          {
            halt();
            Serial.println("Halt");
            break;
          }
      }
      }
    }
  }
    //else EEG_mode();
  }
  //-----------------------------------------------------------------------------------------------------------------------------------//
  //                                                          Data Receive                                                           //
  //-----------------------------------------------------------------------------------------------------------------------------------//

  void serialdata1()
  {
    if (Serial1.available())
    {
      datatype = Serial1.read();
      //Serial.println(a);
      if ( datatype == 'e')
      {
        while (!Serial1.available());
        state = Serial1.read();
        while (!Serial1.available());
        power = Serial1.read();
        if (state == 2 && power > THRESH)
        {
          enable = 1;
        }
        else enable = 0;
        //      Serial.print("State: ");
        //      Serial.print(state);
        //      Serial.print('\t');
        //      Serial.print("Power: ");
        //      Serial.println(power);
      }
      else if ( datatype == 'g')
      {
        while (!Serial1.available());
        head_angle = Serial1.read() - 120;
        //      Serial.print("head_angle: ");
        //      Serial.println(head_angle);
        //      Serial.print('\n');
      }
    }
  }

  void serialdata2()
  {
    if (Serial2.available()) {
      datatype = Serial2.read();
      if (datatype == 'V')
      {
        while (!Serial2.available());
        voicecommand = Serial2.read();
      }
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------------------//
  //                                                          Mode checking                                                            //
  //-----------------------------------------------------------------------------------------------------------------------------------//
  char check_mode(char *data1, char *data2)
  {

  }

  //-----------------------------------------------------------------------------------------------------------------------------------//
  //Function for different modes
  //-----------------------------------------------------------------------------------------------------------------------------------//

  //Function 1 - EEG Mode

  void EEG_mode()
  {

    DMPValues();
    if (calibrate)
    {
      serialdata1();
      if (head_angle != -17)
      {
        Serial.print("head_angle: ");
        Serial.print(head_angle);
        Serial.print("\t");
        Serial.print("Bot angle: ");
        Serial.print(ypr[0]);
        Serial.print('\n');

        Serial.print("State: ");
        Serial.print(state);
        Serial.print('\t');
        Serial.print("Power: ");
        Serial.println(power);

      }
    }

    if (head_angle == -17);

    else if (enable && calibrate && head_angle != -17)
    {
      if (head_angle - ypr[0] > tolerance) {
        right(50);
        Serial.println("right");
      }
      else if (head_angle - ypr[0] < tolerance && head_angle - ypr[0] > 0) {
        front(50);
        Serial.println("front");
      }
      else if (head_angle - ypr[0] < -tolerance) {
        left(50);
        Serial.println("left");
      }
      else if (head_angle - ypr[0] > -tolerance && head_angle - ypr[0] < 0) {
        front(50);
        Serial.println("front");
      }
      else  {
        halt();
        Serial.println("halt");
      }
    }
    else {
      halt();
      Serial.println("halt");
    }
  }



  //Function 2 - Voice mode
  void Voice_mode()
  {

    return;
  }

  //Function 3 - Gesture mode
  void Gesture_mode()
  {
    return;
  }

  //-----------------------------------------------------------------------------------------------------------------------------------//
  //                                                             Motor commands                                                        //
  //-----------------------------------------------------------------------------------------------------------------------------------//

  void left(int pwm)
  {
    //if (enable)
    {
      digitalWrite(MOTOR_DIR1, LOW);
      analogWrite(MOTOR_PWM1, pwm);
      digitalWrite(MOTOR_DIR2, HIGH);
      analogWrite(MOTOR_PWM2, pwm);
      digitalWrite(MOTOR_DIR3, LOW);
      analogWrite(MOTOR_PWM3, pwm);
      digitalWrite(MOTOR_DIR4, HIGH);
      analogWrite(MOTOR_PWM4, pwm);
    }
  }

  void right(int pwm)
  {
    //if (enable)
    {
      //digitalWrite(MOTOR_DIR1, HIGH);
      analogWrite(MOTOR_PWM1, pwm);
      digitalWrite(MOTOR_DIR2, LOW);
      analogWrite(MOTOR_PWM2, pwm);
      digitalWrite(MOTOR_DIR3, HIGH);
      analogWrite(MOTOR_PWM3, pwm);
      digitalWrite(MOTOR_DIR4, LOW);
      analogWrite(MOTOR_PWM4, pwm);
    }
  }

  void back(int pwm)
  {
    //if (enable)
    {
      digitalWrite(MOTOR_DIR1, HIGH);
      analogWrite(MOTOR_PWM1, pwm);
      digitalWrite(MOTOR_DIR2, LOW);
      analogWrite(MOTOR_PWM2, pwm);
      digitalWrite(MOTOR_DIR3, HIGH);
      analogWrite(MOTOR_PWM3, pwm);
      digitalWrite(MOTOR_DIR4, HIGH);
      analogWrite(MOTOR_PWM4, pwm);
    }
  }

  void front(int pwm)
  {
    /// if (enable)
    {
      digitalWrite(MOTOR_DIR1, LOW);
      analogWrite(MOTOR_PWM1, pwm);
      digitalWrite(MOTOR_DIR2, HIGH);
      analogWrite(MOTOR_PWM2, pwm);
      digitalWrite(MOTOR_DIR3, LOW);
      analogWrite(MOTOR_PWM3, pwm);
      digitalWrite(MOTOR_DIR4, LOW);
      analogWrite(MOTOR_PWM4, pwm);
    }
  }

  void halt()
  {
    analogWrite(MOTOR_PWM1, LOW);
    analogWrite(MOTOR_PWM2, LOW);
    analogWrite(MOTOR_PWM3, LOW);
    analogWrite(MOTOR_PWM4, LOW);
  }

