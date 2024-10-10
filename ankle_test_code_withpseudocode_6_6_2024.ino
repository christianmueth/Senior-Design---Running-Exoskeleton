/*By: Christian Mueth, Shuhan Liu*/

//Title: Ankle Exoskeleton Sensor and Motor Controls
//File Title: ankle_test_code_withpseudocode_6_6_2024

//For: ME Team #1 senior design (2024) Running Exoskeleton
//Members: Christian Mueth, DeAndre Thomas, Jaison Dasika Joseph Fleenor, Cole Gala, Shuhan Liu,
//Advisor: Dr. Alan Asbeck
//**Note: Prior existing ARL Lab code/resources were used to help develop this control code
//**Note: a separate code was developed for hip actuation, using a separate microcontroller

//Physical function of "Ankle Exoskeleton Sensor and Motor Controls":
//This code oscillates rotations for an exoskeleton motor attached to the back of the user, to actuate a supportive ankle exoskeleton
//Rotating at a CCW angle actuates a supportive ankle exoskeleton mechanism on the left leg
//Rotating at a CW angle actuates a supportive ankle exoskeleton mechanism on the right leg 

//Sensor control scheme:
//MPU6050 sensors are attached to a user's thighs, and actively read in gyroscope angular velocity data along the x-axis
//For active handling of the gyroscope data, the code saves data from the previous loop and data from the current loop
//Difference of previous vs. current loop data is taken during each loop iteration, to determine whether there is a negative trend
//Once differences are calculated, data is then handled in a state-machine.
//If there is a negative difference trend, characteristic angular velocity thresholds from user motion are then used to trigger control of the AK109 motor
//Proportional-Derivative control is also used to allow for smooth, proportional actuation. PD control also allows for spring-like compliance upon actuation.

//Electronics: MPU6050 IMU/gyroscope sensor, ESP32 microcontroller, CAN bus, AK10-9 motor
//Note for ARL lab students reading this: in the event of any confusion, please see "Team01_Final_Publication" and the relevant sensor data sheets

//Include standard libraries for electronics 
#include <Arduino.h>
#include <CAN.h>
#include <CAN_config.h>
#include <ESP32CAN.h>
#include <can_regdef.h>
#include <stdint.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Declare MPU6050 sensors:
Adafruit_MPU6050 mpu1;//Left leg sensor
Adafruit_MPU6050 mpu2;//Right leg sensor

//**State Variables**
int strike_state;//variable to help keep track of state transitions, upon trigger of sensor thresholds

//Gyroscope Variables
//Gyroscope variables for sensor state machine on left leg:
int gyrox_left_prev=0;//Saved from previous loop
int gyrox_left_current=0;//Saved from current loop
int gyrox_left_diff=0;//difference between previous and current loop

//Gyroscope variables for sensor state machine on right leg:
int gyrox_right_prev=0;//Saved from previous loop
int gyrox_right_current=0;//Saved from current loop
int gyrox_right_diff=0;//difference between previous and current loop


//**Defining CAN messages, for control with CAN bus:**
CAN_frame_t msg;               // Message sent to motor to rotate
static CAN_frame_t off_msg;    // Message sent to motor to stop rotating
CAN_frame_t rx_msg; 
CAN_frame_t mmode;           // Message received from the motor
CAN_frame_t pos_set;
CAN_device_t CAN_cfg;           // CAN Config

// Function prototypes:
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr);
int hextodec(uint16_t num);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
void readmsg();

//**Velocity, position or torque/current control initialization:**
//Velocity controls (this system does not use velocity control, initialize to zero):
float vel = 0;  
int  v_int =   float_to_uint(vel, -46.48, 46.48, 12);
//Torque controls (this system does not use torque control unless for force testing/diagnostics, initialize to zero):
float torque_f = 0;
int t_int =  float_to_uint(torque_f, -30, 30, 12);
//Note: integers for "torque" variables is read by the motor as an amperage. See data sheet.
//Position controls (system DOES use position control. Initialize position to zero):
float pos_f = 0;
int p_int = float_to_uint(pos_f, -720, 720, 16);
//Define PD variables for position control (tune as needed):
int kd_int = 5; // Generally controls the derivative response, resistance during change
int kp_int = 20;//Generally controls the proportional response, responsiveness
// Can check the range for Kp and Kd values in AK10-9 data sheet


//Setup loop:
void setup() {

  Serial.begin(115200);//initialize baud rate
  delay(10);//standard delay
  
  // Initialize CAN for ESP32:
  CAN_cfg.speed = CAN_SPEED_1000KBPS;//set the CAN speed
  CAN_cfg.tx_pin_id = GPIO_NUM_5;  // Set the Tx pin (adjust if necessary)
  CAN_cfg.rx_pin_id = GPIO_NUM_4;  // Set the Rx pin (adjust if necessary)
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));  // Set the RX queue size
  ESP32Can.CANInit();      // Initialize CAN with the configuration
  *((volatile uint32_t*)(0x3ff6b010)) = (*((volatile uint32_t*)(0x3ff6b010)) & ~0x10);

  Serial.println("Enter 'R' to run the system:");//Request input from the user in serial monitor, before running the system
  
  // Enter M_Mode which is motor control mode (i.e, enter MIT Mode -- see AK10-9 data sheet)
  mmode.FIR.B.FF = CAN_frame_std;
  mmode.MsgID = 0x001; //Motor iD

  //Set address/ID for MIT mode:
  mmode.FIR.B.DLC = 8;
  mmode.data.u8[0] = 0xff;
  mmode.data.u8[1] = 0xff;
  mmode.data.u8[2] = 0xff;
  mmode.data.u8[3] = 0xff;
  mmode.data.u8[4] = 0xff;
  mmode.data.u8[5] = 0xff;
  mmode.data.u8[6] = 0xff;
  mmode.data.u8[7] = 0xfc;
  ESP32Can.CANWriteFrame(&mmode);//Send ID to ESP32

// Set the position to 0 when it starts. Can chnage if you do not want to start at 0 position for the new run.
  pos_set.FIR.B.FF = CAN_frame_std;
  pos_set.MsgID = 0x001;
  pos_set.FIR.B.DLC = 8;
  pos_set.data.u8[0] = 0xff;
  pos_set.data.u8[1] = 0xff;
  pos_set.data.u8[2] = 0xff;
  pos_set.data.u8[3] = 0xff;
  pos_set.data.u8[4] = 0xff;
  pos_set.data.u8[5] = 0xff;
  pos_set.data.u8[6] = 0xff;
  pos_set.data.u8[7] = 0xfe;
  ESP32Can.CANWriteFrame(&pos_set);//Send position command to ESP32

  //**Set up CAN messages and other configurations, based on desired position motor control scheme:**
  msg.FIR.B.FF = CAN_frame_std;
  msg.MsgID = 0x001;  // Motor ID
  msg.FIR.B.DLC = 8;
  msg.data.u8[0] = p_int >> 8;                      // Position high bit
  msg.data.u8[1] = p_int & 0xFF;                    // Position low bit
  msg.data.u8[2] = v_int >> 4;                      // Speed high bit
  msg.data.u8[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);  // Speed low bit, KP 4 high bit
  msg.data.u8[4] = kp_int & 0xFF;                   // KP low bit
  msg.data.u8[5] = kd_int >> 4;                     // Kd high bit
  msg.data.u8[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);  // Torque high bit, Kd low bit
  msg.data.u8[7] = t_int & 0xFF;                    // Torque low bit

  //**Message for turning the motor off**
  off_msg.FIR.B.FF = CAN_frame_std;
  off_msg.MsgID = 0x001;
  off_msg.FIR.B.DLC = 8;
  off_msg.data.u8[0] = 0x7f;
  off_msg.data.u8[1] = 0xff;
  off_msg.data.u8[2] = 0x7f;
  off_msg.data.u8[3] = 0xf0;
  off_msg.data.u8[4] = 0x00;
  off_msg.data.u8[5] = 0x00;
  off_msg.data.u8[6] = 0x07;
  off_msg.data.u8[7] = 0xff;


   //**Try to initialize MPU sensors on right and left leg (send failure message if failure occurs):**
  if (!mpu1.begin()) {
    Serial.println("Failed to find MPU6050 chip1");//Left leg sensor
    while (1) {
      delay(10);//stay in this loop if fail
    }
  } 
    if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip2");//Right leg sensor
    while (1) {
      delay(10);//stay in this loop if fail
    }
  }
  Serial.println("MPU6050 Found!");//Send if each sensor is successful

//Set accelerometer range for left leg MPU sensor and output selection to user (default: 8G):
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu1.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  //Set gyroscope range for left leg MPU sensor and output selection to user (default: 500 deg/s):
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu1.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  //Set left leg mpu1 to 184hz, for better performance, and output selection to user:
  mpu1.setFilterBandwidth(MPU6050_BAND_184_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu1.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

//Set accelerometer range for right leg MPU sensor and output selection to user (default: 8G):
mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu2.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

    //Set gyroscope range for right leg MPU sensor and output selection to user (default: 500 deg/s):
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu2.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

//Set right leg mpu2 to 184hz, for better performance, and output selection to user:
  mpu2.setFilterBandwidth(MPU6050_BAND_184_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu2.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

//Standard setup delay:
  Serial.println("");
  delay(100);
}


//**Primary control loop**
void loop() {

//Note: for first loop, all sensor values are always initialized as zero

  /* Get new sensor events with the readings (gyro left "g1" and gyro right "g2" on the gyro x-axis are the critical readings) */
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);

  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);
  delay(10);

//Update gyro state machine variables, for the control state machine:
    gyrox_left_diff = gyrox_left_current - gyrox_left_prev;//take the trend developed from previous loop
    gyrox_left_prev = gyrox_left_current;//update previous gyro value, for current looping
    gyrox_left_current = g1.gyro.x;//update current gyro value, for current looping

    gyrox_right_diff = gyrox_right_current - gyrox_right_prev;//take the trend developed from previous loop
    gyrox_right_prev = gyrox_right_current;//update previous gyro value, for current looping
    gyrox_right_current = g2.gyro.x;//update current gyro value, for current looping

    //Condition for left leg step state (looks for negative difference trend, and the trigger of a threshold):
    if (strike_state != 1 && gyrox_left_diff<=0 && gyrox_left_current<=-0.9) 
    {
      delay(50);//standard delay

      // Code to run the motor in the CCW direction
      //**Note: due to PD controller, position tuning may be required for ideal performance

        pos_f = 60;  // Set position for CCW direction (tune as needed)
        p_int = float_to_uint(pos_f, -720, 720, 16);
        msg.data.u8[0] = p_int >> 8;  // Update position in the CAN message
        msg.data.u8[1] = p_int & 0xFF;
        ESP32Can.CANWriteFrame(&msg);  // Send CAN message for position control
        
        strike_state = -1;//declare strike state, to prevent unwanted switching
        //Introduce a delay, to remain in step state
        delay(200);
        //Serial print gyro data and gyro state to user for left actuation:
        Serial.print(g1.gyro.x);
        Serial.print(g2.gyro.x);
        Serial.println("Left_act");
    }
    //Condition for right leg step state (looks for negative difference trend, and the trigger of a threshold):
    else if(strike_state != -1 && gyrox_right_diff<=0 && gyrox_right_current<=-0.9)
    {
      delay(50);//standard delay

      // Code to run the motor in the CW direction
      //**Note: due to PD controller, position tuning may be required for ideal performance

        pos_f = -100;  // Set position for CW direction (tune as needed)
        p_int = float_to_uint(pos_f, -720, 720, 16);
        msg.data.u8[0] = p_int >> 8;  // Update position in the CAN message
        msg.data.u8[1] = p_int & 0xFF;
        ESP32Can.CANWriteFrame(&msg);  // Send CAN message for position control
        
        strike_state = 1;//declare strike state, to prevent unwanted switching
        //Introduce a delay, to remain in step state
        delay(200);
        //Serial print gyro data and gyro state to user for right actuation:
        Serial.print(g1.gyro.x);
        Serial.print(g2.gyro.x);
        Serial.println("Right_act");
    }
      
    //If strike occurs, return to neutral position
    if(strike_state == 1 || strike_state== -1)
    {
        delay(10);//Standard delay

      // Code to run the motor in the CW direction
      //**Note: due to PD controller, position tuning may be required for ideal performance

        pos_f = -35;  // Set position for neutral (effectively a 0 position, tune as needed)
        p_int = float_to_uint(pos_f, -720, 720, 16);
        msg.data.u8[0] = p_int >> 8;  // Update position in the CAN message
        msg.data.u8[1] = p_int & 0xFF;
        ESP32Can.CANWriteFrame(&msg);  // Send CAN message*/

        //Introduce a delay, to remain in step state
        delay(100);
        //Serial print gyro data and gyro state to user for neutral actuation:
        Serial.print(g1.gyro.x);
        Serial.print(g2.gyro.x);
        Serial.println("Neutral");
        strike_state=0;
    }
    else
    {
      delay(50);// Delay for standard frequency of data acquisition
    }

  // Reading real-time values of position, velocity, and torque of the motor
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_msg, 3 * portTICK_PERIOD_MS) == pdTRUE) {
    readmsg();
  }

  delay(50);  // Delay for standard frequency of data acquisition
}



//**General hexadecimal handling for motor**
//-------------------------------------------------------
static uint8_t hex[17] = "0123456789abcdef";

static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}
//-------------------------------------------------------
//// Function for converting from hexadecimal to decimal system

int hextodec(uint16_t num){
  int out;
  out = (num & 0x000f) + 1.6*(num & 0x00f0) + pow(1.6,2)*(num & 0x0f00) + pow(1.6,3)*(num & 0xf000);
  return out;
}
// -------------------------------------------------------------
/// Converting the ints to float
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
//-----------------------------------------------------------------

/// To convert from float to int
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
/// Converts a float to an unsigned int, given range and number of bits ///
float span = x_max - x_min;
if(x < x_min) x = x_min;
else if(x > x_max) x = x_max;
return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

// Function to read return data from the motor (do not confuse the i as current, it is torque)
void readmsg() {

  // Unpack ints from CAN frame
  int p_i = (rx_msg.data.u8[1] << 8) | rx_msg.data.u8[2];         // Motor position data
  int v_i = (rx_msg.data.u8[3] <<  4) | (rx_msg.data.u8[4] >> 4); // Motor speed data
  int i_i = ((rx_msg.data.u8[4] & 0xF) << 8) | rx_msg.data.u8[5]; // Motor torque data
  
  /// convert ints to floats ///
  // data used in the function is obtained from the data sheets for the T- Motor AK10-9 V 1.1, where the velcity and torque limits are mentioned.
  // Limits decided after trial and error
  
  float p = uint_to_float(p_i, -720, 720, 16);
  float v = uint_to_float(v_i, -46.48, 46.48, 12);
  float i = uint_to_float(i_i, -30, 30, 12);
 
  //print essential data needed for the purpose
  
//Serial.print("ID:");
//Serial.print(id);       // printing motor ID
//Serial.write("\t\t\t");
  Serial.print("P:");
  Serial.print(p);       // position in float
  Serial.write("\t\t\t");
  Serial.print("V:");
  Serial.print(v);         //velocity in float
  Serial.write("\t\t\t");
  Serial.print("T:");
  Serial.print(i);         // torque in float
  Serial.print('\t');
  Serial.write('\n');
}