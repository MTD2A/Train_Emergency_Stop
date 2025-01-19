/*
IR, Ultrasonic and Laser Sensor DCC Train Emergency STOP - DGMF. https://danskgmodelforening.dk/
The program is coded with a focus on simplicity, ease of understanding and self-explanatory.
Therefore minimal use of classes (C++ OOP), structures, pointers and similar more advanced code.

The parallelization and time control is based on fixed time-synchronized cadence control 
(Loop_Delay_Time) rather than the use of current time consumption. 
This is to ensure that functions are carried out synchronously and in phase.

Please read the accompanying detailed documentation of 40 pages.

Jørgen Bo Madsen - V6 - Januar 2025 - dgmf@jorgen-madsen.dk
-------------------------------------------------------------------------------------------------
*/

// Include libraries

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
// Visual Studio code debug
//#include "avr8-stub.h"
//#include "app_api.h"

#define SerialPort true  // true: test and debug monitor information - false: production

// IR binary Sensor configurations --------------------------------------------------------------

// IR long breakbeam 1-100 cm TTL sensor DFrobot SEN0523
// IR medium breakbeam 1-50 cm TTL sensor DFrobot SEN0503
// IR medium reflection beam 3-50 cm TTL sensor E18-D50NK
// IR medium reflection beam 3-80 cm TTL sensor E18-D80NK
// IR short range 1-10 cm TTL sensor FC-51 reflection og breakbeam.
// Reflect:  Object reflects the sensor beam = train on track. FC-51 normal
// Break: Object breaks the sensor beam = train on track. FC-51 sender LED on opposite side
const bool FC_51_Reflect = LOW, FC_51_Break = HIGH, SEN0503 = LOW, SEN0523 = LOW, E18_D50NK = LOW, E18_D80NK = LOW;
const bool YES = true, NO = false; // Easy understanding
// IR 1
const bool IR_Sensor_ON_1   = NO;  // IR sensor connected to Nano
const byte IR_Sensor_Pins_1 = 14;  // Digital D14/A0
const bool IR_Sensor_Mode_1 = FC_51_Break;
// IR 2
const bool IR_Sensor_ON_2   = NO;  // IR sensor connected to Nano
const byte IR_Sensor_Pins_2 = 15;  // Digital D15/A1
const bool IR_Sensor_Mode_2 = FC_51_Break;
// IR 3
const bool IR_Sensor_ON_3   = NO;  // IR sensor connected to Nano
const byte IR_Sensor_Pins_3 = 16;  // Digital D16/A2
const bool IR_Sensor_Mode_3 = FC_51_Break;
// IR 4
const bool IR_Sensor_ON_4   = NO;  // IR sensor connected to Nano
const byte IR_Sensor_Pins_4 = 17;  // Digital D17/A3
const bool IR_Sensor_Mode_4 = FC_51_Break;
//
const bool IR_Sensor_Debug  = YES; // Show which sensor is triggering

// IR analog range sensor GP2Y0A[21/41/51]
// Sensor MUST be powered directly from the Arduino Nano board 5V (27) and GND (29)
const byte  A21 = 0, A41 = 1, A51 = 2;  // Select GP2Y0A IR-sensor type.
const bool  IR_GP2Y0A_ON_1     = NO;    // IR sensor wired and connected to Nano
const byte  IR_GP2Y0A_Type_1   = A41;   // A21 = GP2Y0A21YK0F / A41 = GP2Y0A41SK0F / A51 = GP2Y0A51SK0F
const byte  IR_GP2Y0A_Min_CM_1 = 4;     // Min distance: GP2Y0A21 = 10 cm. GP2Y0A41 =  4 cm. GP2Y0A51 = 2 cm
const byte  IR_GP2Y0A_Max_CM_1 = 30;    // Max distance: GP2Y0A21 = 80 cm. GP2Y0A41 = 30 cm. GP2Y0A51 = 15 cm
const byte  IR_GP2Y0A_Pin_1    = A6;    // REMEMBER to ground PIN A6 when ON and disconnected
const bool  IR_GP2Y0A_Debug_1  = YES;   // Show constant measurement data
//
const bool  IR_GP2Y0A_ON_2     = NO;    // IR sensor wired and connected to Nano
const byte  IR_GP2Y0A_Type_2   = A51;   // A21 = GP2Y0A21YK0F / A41 = GP2Y0A41SK0F / A51 = GP2Y0A51SK0F
const byte  IR_GP2Y0A_Min_CM_2 = 2;     // Min distance: GP2Y0A21 = 10 cm. GP2Y0A41 =  4 cm. GP2Y0A51 = 2 cm
const byte  IR_GP2Y0A_Max_CM_2 = 15;    // Max distance: GP2Y0A21 = 80 cm. GP2Y0A41 = 30 cm. GP2Y0A51 = 15 cm
const byte  IR_GP2Y0A_Pin_2    = A7;    // REMEMBER to ground PIN A7 when ON and disconnected
const bool  IR_GP2Y0A_Debug_2  = YES;   // Show constant measurement data

// HC-SR04 long range ultrasonic indor accurate digital distance measurement
// RCWL-1620 outdor and A02YYUW waterproof long range ultrasonic sensors
const bool  USonic_Sensor_ON_1 = NO;    // Wired and connected to Nano
const byte  USonic_Min_CM_1    = 3;     // Minimal distance 3 cm.
const byte  USonic_Max_CM_1    = 50;    // Max distance 100 cm.
const byte  USonic_Pin_Echo_1  = 2;     // D2 ISR Echo input
const byte  USonic_Pin_Trig_1  = 4;     // D4 Trigger output
const bool  USonic_Debug_1     = YES;   // Show constant measurement data
//
const bool  USonic_Sensor_ON_2 = NO;   // Wired and connected to Nano
const byte  USonic_Min_CM_2    = 3;     // Minimal distance 3 cm.
const byte  USonic_Max_CM_2    = 50;    // Max distance 100 cm.
const byte  USonic_Pin_Echo_2  = 3;     // D3 ISR Echo input
const byte  USonic_Pin_Trig_2  = 5;     // D5 Trigger output
const bool  USonic_Debug_2     = YES;   // Show constant measurement data
//
// Closeup tranducer blocking detecetion. Measured error distance CM
const word  HC_SR04 = 820, RCWL_1670 = 693, A02YYMW = 693; // indoor, outdoor and waterproof
const word  USonic_Blocking_CM_1   = HC_SR04;   // 0 = OFF!
const word  USonic_Blocking_CM_2   = RCWL_1670; // 0 = OFF!
const word  USonic_Max_Block_Count = 600;  // Number of loops to calculate average

// Sensor configuration --------------------------------------------------------------------

// VL53L4CD I2C Time of Flight Micro Distance Sensor and close-up sensor blocking detection
// Fast ranging 10 ms loop robust and error-correcting functionality with detailed error descriptions
// REMEMBER! disconnet power briefly to all sensors and Arduino MCU for correct reset!
const bool  VL53L4CD_Laser_I2C_ON[8]    = // Wired and connected to Nano
// sensor   0      1      2      3      4      5      6      7
        {  NO,    NO,    NO,    NO,    NO,    NO,    NO,    NO }; 
const word  VL53L4CD_Min_Distance_MM[8] = // Min distance = 25 mm
// sensor   0      1      2      3      4      5      6      7
        {  25,    25,    25,    25,    25,    25,    25,    25 };
const word  VL53L4CD_Max_Distance_MM[8] = // Max distance = 800 mm.
// sensor   0      1      2      3      4      5      6      7
       {  800,   800,   800,   800,   800,   800,   800,   800 };
//
// Show range, status, warning and error messages ------------------------------------------
// 
const bool  I2C_wire_Debug_Enabled       = NO;   // List I2C Wire connected devices and other I2C debug information
const bool  VL53L4CD_Print_Object_Detect = NO;   // Print distance and min-max zone, when object is detected: 
const bool  VL53L4CD_Print_Range_Status  = NO;   // Print status information. Se www.ST.com UM2931 manual.
//
const bool  VL53L4CD_Debug_Enabled[8]   = // Print ranging and error messages
// sensor   0      1      2      3      4      5      6      7
        { YES,   YES,   YES,   YES,   YES,   YES,   YES,   YES }; 
//
// Close-up Blocking detection distance configuration --------------------------------------
//
const bool  VL53L4CD_Blocking_Detect[8] =
// sensor   0      1      2      3      4      5      6      7
       {  YES,   YES,   YES,   YES,   YES,   YES,   YES,   YES };
// Optimized to 1,0 mm air gap and 0,35 mm 9H photo lens cover glass (1,35 mm distance in total)
// Close-up total sensor blocking: Matte black = 10 mm, Matte gray = 4 mm, Light yellow = 16 mm, Shiny white = 13 mm
// To lower blocing distance to 15 mm, increase air gap to 2 mm (2,35 mm distance in total)
const word  VL53L4CD_Max_BLock_Dist_MM = 19;   // Starting from top of cover glass (1,35 mm from senesor)
const word  VL53L4CD_Max_Block_Count   = 600;  // Number of loops to calculate average
//
// Sensor calibration ----------------------------------------------------------------------
//
// Advanced configuration. Read first www.ST.com UM2931 manual.
const bool  VL53L4CD_Offset_Calibrate   = NO;   // (YES / NO) Activate calibration mode
const short VL53L4CD_Offset_Dist_MM[8]  =       // Subtract from / add to range result
// sensor   0      1      2      3      4      5      6      7
       {   -8,    -8,    -8,    -8,    -8,    -8,    -8,    -8 };

const short VL53L4CD_Xtalk_Distance_MM  = 1000; // Maximum measured raging distance without under-ranging
const bool  VL53L4CD_Xtalk_Calibrate    = NO;   // (YES / NO) Activate calibration mode
const word  VL53L4CD_Xtalk_KCPS[8]      =       // Depends on Cover Glass type or no cover glass
// sensor   0      1      2      3      4      5      6      7
       {    0,     0,     0,     0,     0,     0,     0,     0 };

// Adress and ID ---------------------------------------------------------------------------

const byte  VL53L4CD_I2C_Address = 0x29;    // Adafruit and others 0x29 HEX
const word  VL53L4CD_Sensor_ID   = 0xEBAA;  // Sensor ID. Do not change!

// Error correction thresholds -------------------------------------------------------------

// Optimized to 1,0 mm air gap and 0,35 mm 9H photo lens cover glass (1,35 mm distance in total)
// Read www.ST.com AN4907 Application note document DM00326504 
const word  VL53L4CD_Max_Sigma_MM      = 6;   // Default 15
const word  VL53L4CD_Min_Signal_KPCS   = 2048;
//
// Ambien light. Too much ambient light may will entail incorrect mauserments.
const word  VL53L4CD_Max_Ambient_KCPS  = 8000; // Maximum ambient light 
const word  VL53L4CD_Max_Ambient_Count = 600;  // Number of loops to calculate average

// MUX configuration -----------------------------------------------------------------------

// YES: I2C MUX connecte to Nano. NO: first VL53L4CD directly connected to Nano.
const bool  TCA_PCA_I2C_MUX_ON  = NO;   // If MUX is off, Only sensor 0 is useable
const byte  TCA_PCA_I2C_Address = 0x70;  // Defalt I2C address 0x70 HEX
// TCA9548A 8-Channel I2C Multiplexer / PCA9548A - sensor channel 0 - 7 [8]
// TCA9546A 4-Channel I2C Multiplexer / PCA9546A - sensor channel 0 - 3 [4]
const bool  TCA_PCA_9548A = true, TCA_PCA_9546A = false;
const bool  TCA_PCA_I2C_Model   = TCA_PCA_9548A;

// Global configuration variabels ----------------------------------------------------------------------

// Input / Output units and timers. 
// Timer configuration: Always use a multiple of Loop_Delay_Time for accurate timing

// Bridge
const byte  Button_Pin         = 11;   // D11  Green reset button
const short Bridge_Open        = LOW;  // Switch activate to GND and internal pullup resister
// Button
const byte  Bridge_Pin         = 12;   // D12  Brige open/close micro switch
const short Button_Presset     = LOW;  // Switch activate to GND and internal pullup resisterBridge_Open
// Stop and start relays
const byte  Relay_Stop_Pin     = 6;    // D6  Activate emergency stop (DiMAX feedback port 4a)
const byte  Relay_Start_Pin    = 7;    // D7  DE-activate emergency stop (DiMAX feedback port 4b)
const word  Relay_ON_Time      = 50;   // Default 50 Milliseconds (Relay_ON)
const word  Relay_OFF_Time     = 50;   // Default 50 Milliseconds (Relay_OFF)
const byte  Relay_ON           = LOW;
const byte  Relay_OFF          = HIGH;
// Buzzer sound
const byte  Buzzer_Pin         = 8;    // D4  Alarm sound
const word  Buzzer_Low_Time    = 250;  // Low  Pitch time. Defalt 250 Milliseconds
const word  Buzzer_High_Time   = 250;  // High pitch time. Defalt 250 Milliseconds
// Buzzer frequency
const word  Buzzer_Low_Pitch   = 2000; // Default 2.000 Hz
const word  Buzzer_High_Pitch  = 3000; // Default 3.000 Hz
const byte  Buzzer_Silent_Mode = LOW;  // Buzzer YL-44 is Silent on HIGH! KY-006 on LOW!
// Buzzer alarm ON-OFF cycle
const word  Alarm_ON_Time      = 5500;  // Defalt  5.000 Milliseconds Buzzer alarm cycle
const word  Alarm_OFF_Time     = 15000; // Defalt 15.000 Milliseconds Buzzer silent cycle
// Red LED blink
const byte  LED_Red_Pin        = 9;    // D9  Status multicolor LED - common cathode
const word  LED_Red_ON_Time    = 250;  // Defalt 250 Milliseconds
const word  LED_Red_OFF_Time   = 250;  // Defalt 250 Milliseconds
// Red LED PWM
const short LED_Red_ON         = 255;  // PWM LED brightness 10..255. Default 255
const short LED_Red_OFF        = 0;    // PWM 0. Zero light
// Green LED
const byte  LED_Green_Pin      = 10;   // D10 Status multicolor LED - common cathode
const short LED_Green_ON       = 127;  // PWM LED brightness 10..255. Defaul 127
const short LED_Green_OFF      = 0;    // PWM 0. Zero light
// More external sound and light systems
const byte  Alarm_Status_Pin   = 13;   // D13 - LED_BUILLTIN
const byte  Alarm_Status_ON    = HIGH;
const byte  Alarm_Status_OFF   = LOW;
// Rest button buzzer error sound
// 1.000 - 5.000 Hertz. 0 = no sound. Duration in milliseconds.
const byte  Error_Sound_Max_Step = 6;
const word  Error_Sound_Pitch_Hz[Error_Sound_Max_Step]    = { 3500,  0 , 4250,   0, 3500,    0 };
const word  Error_Sound_Duration_MS[Error_Sound_Max_Step] = {  500, 500, 2000, 500,  500, 3000 };
// General error buzzer sound
// 1.000 - 5.000 Hertz. 0 = no sound. Duration in milliseconds.
const byte  Reset_Sound_Max_Step = 8;
const word  Reset_Sound_Pitch_Hz[Reset_Sound_Max_Step]    = { 1000, 1100, 1200, 1300, 1400, 1500, 1600,   0 };
const word  Reset_Sound_Duration_MS[Reset_Sound_Max_Step] = {  100,  100,  100,  100,  100,  100,  100, 100 };
// Main loop delay 10 - 100.
const word  Loop_Delay_Time    = 10; // Main loop for parallel processing. 10 Millisec. DO NOT CHANGE!

// Global processing variables - do not change ----------------------------------------------------------

const bool IR_Sensor_ON[4]   = { IR_Sensor_ON_1,   IR_Sensor_ON_2,   IR_Sensor_ON_3,   IR_Sensor_ON_4   };
const byte IR_Sensor_Pins[4] = { IR_Sensor_Pins_1, IR_Sensor_Pins_2, IR_Sensor_Pins_3, IR_Sensor_Pins_4 };
const bool IR_Sensor_Mode[4] = { IR_Sensor_Mode_1, IR_Sensor_Mode_2, IR_Sensor_Mode_3, IR_Sensor_Mode_4 };
// Alarm
bool  Alarm_Loop_Switch    = false;
bool  Alarm_Loop_Stop      = true;
word  Alarm_Loop_Time      = 0; 
// Relays
bool  Relay_Loop_Switch    = false;
word  Relay_Loop_Time      = 0; 
// Buzzer
bool  Buzzer_Loop_Switch   = false;
word  Buzzer_Loop_Time     = 0; 
// LED
bool  LED_Red_Loop_Switch  = false;
word  LED_Red_Loop_Time    = 0; 
// Counters and logic
word  Offset_Delay_Time    = 0;
// Detector flags
bool  Bridge_Open_Detected = NO;
bool  Obstacle_Detected    = NO;
// Sensor status
bool  Last_Sensor_Satus    = false;
bool  Current_Sensor_Satus = false;
// Buzzer warning and error sound
word  Buzzer_Pitch_Hz[10]; 
word  Buzzer_Duration_MS[10];
byte  Buzzer_Max_Step      = 0;
byte  Buzzer_Step_Count    = 0;
word  Buzzer_Step_Time     = 0;
bool  Buzzer_Sound_ON      = NO;
// Button reset warning
bool  Reset_Sound_ON       = NO;
bool  Reset_Error_Detected = NO;
// Error sound
bool  Error_Sound_ON       = NO;
byte  Error_Sound_Count    = 0; // Number of repeatede sound loops
// For better function understanding
const bool Obstacle_Present = true; 
// Uultrasonic interrupt variables
volatile unsigned long USonic_Last_Pulse_Time_1 = 0;
volatile unsigned long USonic_Last_Pulse_Time_2 = 0;
word  USonic_Loop_Time = 0;
// Ultrasonic blocking detection
long  USonic_Average_Block_MM[2];        // Average blocking distance 
word  USonic_Average_Block_Count[2];     // Average blocking count
//
// I2C variables and structures ---------------------------------------------------------
//
const bool OK = true, Error = false;     // Easy understanding
bool  TCA_PCA_I2C_MUX_Status = OK;       // MUX start status OK
bool  VL53L4CD_Error_Status[8];          // I2C & range error handling
// Ambient light check
long  VL53L4CD_Average_Ambient_KCPS[8];  // Average ambient light level KCPS
word  VL53L4CD_Average_Ambient_Count[8]; // Average ambient count
// Blocking detection
long  VL53L4CD_Average_Block_MM[8];      // Average blocking distance 
word  VL53L4CD_Average_Block_Count[8];   // Average blocking count
// Sensor processing
byte  VL53L4CD_Status        = 0;  // General function return result
byte  VL53L4CD_Max_Count     = 0;  // Max number of lasers to process
byte  VL53L4CD_Active_Sensor = 0;  // Currently active laser [0-7]
byte  VL53L4CD_Max_ON_Index  = 0;  // Max number of activated (ON) lasers
byte  VL53L4CD_Loop_ON_Index = 0;  // Round-Robin laser ON reference Index 
byte  VL53L4CD_Laser_ON_List[8];   // Activated (ON) laser reference list
VL53L4CD_Result_t VL53L4CD_Result; // Measured data
// Sensor structures
VL53L4CD VL53l4CD_0(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_1(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_2(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_3(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_4(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_5(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_6(&Wire, 0);     // xhut = 0 => no pin used
VL53L4CD VL53l4CD_7(&Wire, 0);     // xhut = 0 => no pin used
// Laser pointer array
VL53L4CD* VL53L4CD_Lasers_List[8] = { &VL53l4CD_0, &VL53l4CD_1, &VL53l4CD_2, &VL53l4CD_3, \
                                      &VL53l4CD_4, &VL53l4CD_5, &VL53l4CD_6, &VL53l4CD_7 };
//
// Serial print -------------------------------------------------------------------------
//
#if SerialPort == true
  // Serial print slow down execution significantly and affects all functions that use time control.
  #define PortPrint(x)   Serial.print(x)
  #define PortPrintln(x) Serial.println(x)
#else
  // Production
  #define PortPrint(x)
  #define PortPrintln(x)
#endif


// Setup ---------------------------------------------------------------------------


void setup() {
  // Start serial connection to serial monitor
  Serial.begin(250000); // Monitor BAUD speed
  // Define output Pins
  Setup_PinMode();
  // Set ouput state
  Reset_Pin_Output();
  // Ultrasonic setup
  Setup_Ultrasonic();
  // I2C setup
  Setup_I2C_Lasers();
  // Error control
  Check_Configuration_Parameters();
  // Ensure sensors are ready and stable
  delay(100);
}  // end setup


void Setup_PinMode() {
  pinMode(Relay_Stop_Pin, OUTPUT);
  pinMode(Relay_Start_Pin, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);
  pinMode(LED_Red_Pin, OUTPUT);
  pinMode(LED_Green_Pin, OUTPUT);
  // Define input pins
  pinMode(Button_Pin, INPUT_PULLUP);
  pinMode(Bridge_Pin, INPUT_PULLUP);
  // Define IR sensor inputs (FC-51, SEN0503 & SEN052)
  for (byte Index = 0; Index < 4; Index++) {
    if (IR_Sensor_ON[Index] == YES) {
      if (IR_Sensor_Mode[Index] == LOW) {
        pinMode(IR_Sensor_Pins[Index], INPUT_PULLUP);
      }
      else {
        pinMode(IR_Sensor_Pins[Index], INPUT);
      }
    }
  }
  // Define IR GP2Y0A sensor inputs
  pinMode(IR_GP2Y0A_Pin_1, INPUT);
  pinMode(IR_GP2Y0A_Pin_2, INPUT);
} // Setup_PinMode


void Setup_Ultrasonic() {
  for (byte USonic_No = 0; USonic_No < 2; USonic_No++) {
    USonic_Average_Block_MM[USonic_No]    = 0;
    USonic_Average_Block_Count[USonic_No] = 0;
  }
  // Define Ultrasonic sensor input and output
  pinMode(USonic_Pin_Trig_1, OUTPUT);
  pinMode(USonic_Pin_Echo_1, INPUT);
  pinMode(USonic_Pin_Trig_2, OUTPUT);
  pinMode(USonic_Pin_Echo_2, INPUT);
  // Ultra Sonic interupts. Nano user interupt support only on pin 2 and 3
  // https://gammon.com.au/interrupts
  EIFR = bit (INTF0);  // clear flag for interrupt 0 (D2)
  EIFR = bit (INTF1);  // clear flag for interrupt 1 (D3)
  attachInterrupt(digitalPinToInterrupt(USonic_Pin_Echo_1), USonic_Distance_ISR_1, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(USonic_Pin_Echo_2), USonic_Distance_ISR_2, CHANGE);  
} // Setup_Ultrasonic


// Main loop ----------------------------------------------------------------------------------------


void loop() {
  // test Bridge_Open
  // if (digitalRead(Bridge_Pin) == !Bridge_Open) {  // Debug
  if (digitalRead(Bridge_Pin) == Bridge_Open) {
    // PortPrintln(F("The swing bridge is open. Sensor surveillance is active."));
    Bridge_Open_Green_LED_ON();
     // check out all sensors and save last status
    Last_Sensor_Satus = Current_Sensor_Satus;
    Current_Sensor_Satus = Read_All_Sensor_Status();
    if (Current_Sensor_Satus == Obstacle_Present || Obstacle_Detected == YES) {
      // PortPrintln(F("Train detected on rails. STOP all trains!"));
      Obstacle_Detected_Green_LED_OFF();
      Main_Alarm_Loop();
    } // Sensor obstacle detected
  } 
  else {
    Bridge_Closed_Green_LED_OFF();
    delay((unsigned long)Loop_Delay_Time);
  }  // End Bridge
  // Chek for errors
  Reset_Error_Sound();
  Error_Alarm_Sound();
} // End main loop


// Functions ---------------------------------------------------------------------------


void Main_Alarm_Loop() {
  STOP_All_Trains();
  LED_Red_Blink();
  //
  if (digitalRead(Button_Pin) == Button_Presset) {
    // Check for illigal reset. E.g train is still deteceted by sensor
//    if (Last_Sensor_Satus == Obstacle_Present  &&  Current_Sensor_Satus == Obstacle_Present) {
    if (Current_Sensor_Satus == Obstacle_Present) {
      Reset_Error_Detected = YES;
    } 
    else {
      // Start all trains (cancel / reset Emergency STOP) and reset all
      Start_All_Trains();
      Reset_All();
    }
  } 
  else {
    // Asynchronous alarm cycle. Buzzer on and off
    Buzzer_Alarm_Loop();
  }  // Button pressed
} // Main_Alarm_Loop


void Reset_Error_Sound() {
  if (Reset_Error_Detected == YES) {
    if (Reset_Sound_ON == NO) {
      Buzzer_Load_Data(Reset_Sound_Pitch_Hz, Reset_Sound_Duration_MS, Reset_Sound_Max_Step);
      Reset_Sound_ON = YES;
    }
    if (Buzzer_Sound_Process() == true) {  // Sound process ended
      Reset_Sound_ON = NO;
      Reset_Error_Detected = NO;
      if (Error_Sound_ON == YES) {  // Restart error sound process
        Error_Sound_ON    = NO;
        Buzzer_Step_Count = 0;
        Buzzer_Step_Time  = 0;
      }
    } 
  } // Reset_Error_Detected
}  // Reset_Sound_Sound


void Bridge_Open_Green_LED_ON() {
  if (Bridge_Open_Detected == NO) {
    analogWrite(LED_Green_Pin, LED_Green_ON);
    analogWrite(LED_Red_Pin, LED_Red_OFF);
    Bridge_Open_Detected = YES;
  }
} // Bridge_Open_Green_LED_ON


void Obstacle_Detected_Green_LED_OFF() {
  if (Obstacle_Detected == NO) {
    analogWrite(LED_Green_Pin, LED_Green_OFF);
    digitalWrite(Alarm_Status_Pin, Alarm_Status_ON);
    Obstacle_Detected = YES;
  }
} // Obstacle_Detected_Green_LED_OFF


// Read all sensors -------------------------------------------------------------------


bool Read_All_Sensor_Status() {
  bool Return_Status = false;
  unsigned long Start_Time_MS;
  Start_Time_MS = millis();
  //
  // IR sensor FC-51, SEN0503 & SEN0523
  if (IR_Sensor_Status_1_2_3_4() == Obstacle_Present) {
    Return_Status = true;
  }
  // IR sensor GP2Y0A
  if (IR_GP2Y0A_Status_1_Or_2() == Obstacle_Present) {
    Return_Status = true;
  }
  // Ultrasonic Sensors: HC-SR04, RCWL-1620 & A02YYMW
  if (USonic_Status_1_Or_2() == Obstacle_Present) {
    Return_Status = true;
  }
  if (VL53L4CD_Read_Sensors_Sequentially() == Obstacle_Present) {
    Return_Status = true;
  }
  // millis(): The number will overflow (go back to zero), after approximately 50 days.
  // https://www.gammon.com.au/millis
  Offset_Delay_Time = (word)(millis() - Start_Time_MS);
  if (Offset_Delay_Time >= Loop_Delay_Time) { 
    Offset_Delay_Time = Loop_Delay_Time;  // Zero delay time
  }
  delay((unsigned long)(Loop_Delay_Time - Offset_Delay_Time));
  return Return_Status;
} // Read_All_Sensor_Status


// IR binary sensers ---------------------------------------------------------------------


bool IR_Sensor_Status_1_2_3_4() {
  int Sub_Loop_Delay_MS;  // Milliseconds
  byte Sub_Max_Loop_Index;
  if (VL53L4CD_Max_ON_Index == 0) {  // Don't optimize. Laser is on and can use op to 8 ms pr. loop.
    Sub_Max_Loop_Index = (Loop_Delay_Time / 2) - 1;
    Sub_Loop_Delay_MS = 2;
  }
  else {  
    Sub_Max_Loop_Index = 1;
    Sub_Loop_Delay_MS = 0;
  }
  // Simple IR sensors are fast switching. Ensure more accurate readings.
  for (byte Sub_Loop_Index = 0; Sub_Loop_Index < Sub_Max_Loop_Index; Sub_Loop_Index++) {
    for (byte Index = 0; Index < 4; Index++) {
      if (IR_Sensor_ON[Index] == YES) {
        if (digitalRead(IR_Sensor_Pins[Index]) == IR_Sensor_Mode[Index]) {
          if (IR_Sensor_Debug == YES) {
            PortPrint(millis()); PortPrint(F(": IR Sensor [")); 
            PortPrint(Index + 1); PortPrintln(F("] triggered"));
          }
          return true;
        }
      }
    }
    delay(Sub_Loop_Delay_MS);
  }
  return false;
} // IR_Sensor_Status_1_2_3_4


// GP2Y0A infrared -----------------------------------------------------------------------


bool IR_GP2Y0A_Status_1_Or_2() {
  short IR_Pin_Value, IR_Value;
  // Test GP2Y0A (1) for value inside specified range
  if (IR_GP2Y0A_ON_1 == YES) {
    IR_Pin_Value = analogRead(IR_GP2Y0A_Pin_1);
    IR_Value = IR_GP2Y0A_Calculate_Dist(IR_Pin_Value, IR_GP2Y0A_Type_1, IR_GP2Y0A_Debug_1, 1);
    if (IR_Value >= (short)IR_GP2Y0A_Min_CM_1  &&  IR_Value <= (short)IR_GP2Y0A_Max_CM_1) {
      return true;
    }
  }
  //Test GP2Y0A (2) for value inside specified range
  if (IR_GP2Y0A_ON_2 == YES) {
    IR_Pin_Value = analogRead(IR_GP2Y0A_Pin_2);
    IR_Value = IR_GP2Y0A_Calculate_Dist(IR_Pin_Value, IR_GP2Y0A_Type_2, IR_GP2Y0A_Debug_2, 2);
    if (IR_Value >= (short)IR_GP2Y0A_Min_CM_2  &&  IR_Value <= (short)IR_GP2Y0A_Max_CM_2) {
      return true;
    }
  }
  return false;
} // IR_GP2Y0A_Status_1_Or_2


short IR_GP2Y0A_Calculate_Dist(const short &IR_Value_Calc, const byte &IR_GP2Y0A_Type,
                               const bool &IR_GP2Y0A_Debug, const byte &IR_GP2Y0A_No) {
    short IR_Value_Calc_CM = 0;
    switch (IR_GP2Y0A_Type) {
      case A21: { IR_Value_Calc_CM = (short)round((7700.0F / (IR_Value_Calc + 10)) - 6); } break;
    	case A41: { IR_Value_Calc_CM = (short)round((3300.0F / (IR_Value_Calc + 10)) - 1); } break;
    	case A51: { IR_Value_Calc_CM = (short)round((2000.0F / (IR_Value_Calc + 30)) - 3); } break;
    }
    if (IR_GP2Y0A_Debug == YES) {
      PortPrint(F("IR_GP2Y0A [")); PortPrint(IR_GP2Y0A_No); PortPrint(F("] (cm): ")); PortPrintln(IR_Value_Calc_CM);
    }
    return IR_Value_Calc_CM;
} // IR_GP2Y0A_Calculate_Dist


// Ultrasonic -----------------------------------------------------------------------


bool USonic_Status_1_Or_2() {
  // Minimum total measurement cycle Milliseconds
  const word USonic_Trig_Cycle = 70; // 70 Milliseconds. Do not change!
  short Read_Dist_CM = 0;
  //
  if (USonic_Sensor_ON_1 == YES || USonic_Sensor_ON_2 == YES) {
    if (USonic_Loop_Time >= USonic_Trig_Cycle) { // Minimum measurement cycle
      USonic_Loop_Time = Loop_Delay_Time;
      //Send trigger impuls - 10 MicroSeconds
      digitalWrite(USonic_Pin_Trig_1, HIGH);
      digitalWrite(USonic_Pin_Trig_2, HIGH);
      delayMicroseconds(10);
      digitalWrite(USonic_Pin_Trig_1, LOW);
      digitalWrite(USonic_Pin_Trig_2, LOW);
    }
    else {
      USonic_Loop_Time = USonic_Loop_Time + Loop_Delay_Time;
    }
    // Test Ultrasonic sensor (1) for value inside specified range
    if (USonic_Sensor_ON_1 == YES) {
      Read_Dist_CM = (short)round(USonic_Last_Pulse_Time_1 / 57.5);
      if (USonic_Debug_1 == YES) {
        PortPrint(F("Ultra Sonic [1] (cm): ")); PortPrintln(Read_Dist_CM);
      }
      // Blocking detection
      USonic_Blocking_Calculate_And_Check(1, USonic_Blocking_CM_1, Read_Dist_CM);
      if ((Read_Dist_CM > (short)USonic_Min_CM_1)  &&  (Read_Dist_CM < (short)USonic_Max_CM_1)) {
        return true;
      }
    }
    // Test Test Ultrasonic sensor (2) for value inside specified range
    if (USonic_Sensor_ON_2 == YES) {
      Read_Dist_CM = (short)round(USonic_Last_Pulse_Time_2 / 57.5);
      if (USonic_Debug_2 == YES) {
        PortPrint(F("Ultra Sonic [2] (cm): ")); PortPrintln(Read_Dist_CM);
      }
      // Blocking detection
      USonic_Blocking_Calculate_And_Check(2, USonic_Blocking_CM_2, Read_Dist_CM);
      if ((Read_Dist_CM > (short)USonic_Min_CM_2)  &&  (Read_Dist_CM < (short)USonic_Max_CM_2)) {
        return true;
      }
    }
  }
  return false;
} // USonic_Status_1_Or_2


void USonic_Blocking_Calculate_And_Check(byte USonic_No, const word &Blocking_CM, const short &Distance_CM) {
  word Average_Distance_CM; 
  if (Blocking_CM != 0) {
    USonic_No--;
    USonic_Average_Block_Count[USonic_No]++;
    USonic_Average_Block_MM[USonic_No] = USonic_Average_Block_MM[USonic_No] + (long)Distance_CM;
    if (USonic_Average_Block_Count[USonic_No] == USonic_Max_Block_Count) {
      Average_Distance_CM = (word)(USonic_Average_Block_MM[USonic_No] / USonic_Average_Block_Count[USonic_No]);
      USonic_Average_Block_MM[USonic_No]    = 0;
      USonic_Average_Block_Count[USonic_No] = 0;
      if ((Average_Distance_CM > (Blocking_CM - 10))  ||  Average_Distance_CM <= 2) {
        Set_Error_Sound_Count(1);
        PortPrint  (F("ERROR! Ultrasonic beam blocked (cm): ")); PortPrint(Average_Distance_CM);
        PortPrintln(F("  Blocking zone (cm): 0 - 2.")); 
      }
    }
  }
} // USonic_Blocking_Calculate_And_Check


// ----------------------------------------------------------------
// Ultrasonic Interrupt Service Routines
// ----------------------------------------------------------------

// Non-blocking interrupt control of Ultra Sonic Sensor  (1)
void USonic_Distance_ISR_1() { // Interrupt Service Routine
  if (USonic_Sensor_ON_1 == YES) {
    static unsigned long USonic_Start_Time_1;
    if (digitalRead(USonic_Pin_Echo_1) == HIGH) { // Pin is HIGH
      USonic_Start_Time_1 = micros();
    } 
    else {  // Pin is LOW
      USonic_Last_Pulse_Time_1 = micros() - USonic_Start_Time_1;
    }
  }
} //End ISR (1)


// Non-blocking interrupt control of Ultra Sonic Sensor (2)
void USonic_Distance_ISR_2() { 
  if (USonic_Sensor_ON_2 == YES) {
    static unsigned long USonic_Start_Time_2;
    if (digitalRead(USonic_Pin_Echo_2) == HIGH) { // Pin is HIGH
      USonic_Start_Time_2 = micros();
    }
    else {  // Pin is LOW
      USonic_Last_Pulse_Time_2 = micros() - USonic_Start_Time_2;
    }
  }
} //End ISR (2)


// Buzzer and LED -----------------------------------------------------------------------


void Start_All_Trains() {  // Monostable
  // Relays are mechanical and slow. Ensure sufficient time for relay to activate.
  digitalWrite(Relay_Stop_Pin, Relay_OFF);
  digitalWrite(Relay_Start_Pin, Relay_ON);
  delay((unsigned long)Relay_ON_Time);
  digitalWrite(Relay_Start_Pin, Relay_OFF);
} // Start_All_Trains


void STOP_All_Trains() {  
  // Relays are mechanical and slow.
  // Ensure sufficient time for relay to activate and de-activate.
  // Make relay pulsating several times each second to enforce continues DCC E-Stop.
  if (Relay_Loop_Time == 0) {
    digitalWrite(Relay_Stop_Pin, Relay_ON);
  }
  // Relay ON
  if (Relay_Loop_Switch == false) {
    if (Relay_Loop_Time >= Relay_ON_Time) {
      digitalWrite(Relay_Stop_Pin, Relay_OFF);
      Relay_Loop_Time = Loop_Delay_Time;
      Relay_Loop_Switch = true;
    }
    else {
      Relay_Loop_Time = Relay_Loop_Time + Loop_Delay_Time;
    }
  }
  //Relay OFF
  else {
    if (Relay_Loop_Time >= Relay_OFF_Time) {
      digitalWrite(Relay_Stop_Pin, Relay_ON);
      Relay_Loop_Time = Loop_Delay_Time;
      Relay_Loop_Switch = false;
    }
    else {
      Relay_Loop_Time = Relay_Loop_Time + Loop_Delay_Time;
    }
  }
} // STOP_All_Trains


void Buzzer_Alarm_Loop() {  
  if (Error_Sound_ON == NO  &&  Reset_Sound_ON == NO) {
    // switch between alarm cycle on and off (silent) asynchronously
    // Alarm ON
    if (Alarm_Loop_Switch == false) {
      if (Alarm_Loop_Time <= Alarm_ON_Time) {
        Alarm_Loop_Time = Alarm_Loop_Time + Loop_Delay_Time;
        Buzzer_Alarm_Pitch_Shift();
      }
      else {
        Alarm_Loop_Time = Loop_Delay_Time;
        Alarm_Loop_Switch = true;
        Alarm_Loop_Stop = true;
      }
    }
    // Alarm OFF
    else {
      if (Alarm_Loop_Time <= Alarm_OFF_Time) {
        Alarm_Loop_Time = Alarm_Loop_Time + Loop_Delay_Time;
        if (Alarm_Loop_Stop == true) {
          Alarm_Loop_Stop = false;
          Buzzer_Off();
        }
      }
      else {
        Alarm_Loop_Time = Loop_Delay_Time;
        Alarm_Loop_Switch = false;
      }
    }
  } //Error_Sound_Count
} // Buzzer_Alarm_Loop


void Buzzer_Alarm_Pitch_Shift() {
  if (Error_Sound_ON == NO  &&  Reset_Sound_ON == NO) {
    // switch between high and low pitch asynchronously
    if (Buzzer_Loop_Time == 0) {
      tone(Buzzer_Pin, Buzzer_High_Pitch);
    }
    // Buzzer high pitch
    if (Buzzer_Loop_Switch == false) {
      if (Buzzer_Loop_Time >= Buzzer_High_Time) {
        tone(Buzzer_Pin, Buzzer_Low_Pitch);
        Buzzer_Loop_Time = Loop_Delay_Time;
        Buzzer_Loop_Switch = true;
      }
      else {
        Buzzer_Loop_Time = Buzzer_Loop_Time + Loop_Delay_Time;
      }
    }
    // Buzzer low pitch
    else {
      if (Buzzer_Loop_Time >= Buzzer_Low_Time) {
        tone(Buzzer_Pin, Buzzer_High_Pitch);
        Buzzer_Loop_Time = Loop_Delay_Time;
        Buzzer_Loop_Switch = false;
      }
      else {
        Buzzer_Loop_Time = Buzzer_Loop_Time + Loop_Delay_Time;
      }    
    }
  }
}  // Buzzer_Alarm_Pitch_Shift


void Buzzer_Off() {
  noTone(Buzzer_Pin);
  // Buzzer YL-44 is active on LOW! KY-006 on HIGH!
  digitalWrite(Buzzer_Pin, Buzzer_Silent_Mode);
} // Buzzer_Off


void LED_Red_Blink() {  // Continuous slow blink.
  if (Error_Sound_ON == NO) {
    // Switch red LED between ON and OFF asynchronously
    if (LED_Red_Loop_Time == 0) {
      analogWrite(LED_Red_Pin, LED_Red_ON);
    }
    // LED on
    if (LED_Red_Loop_Switch == false) {
      if (LED_Red_Loop_Time >= LED_Red_ON_Time) {
        analogWrite(LED_Red_Pin, LED_Red_OFF);
        LED_Red_Loop_Time = Loop_Delay_Time;
        LED_Red_Loop_Switch = true;
      }
      else {
        LED_Red_Loop_Time = LED_Red_Loop_Time + Loop_Delay_Time;
      }
    }
    // LED OFF
    else {
      if (LED_Red_Loop_Time >= LED_Red_OFF_Time) {
        analogWrite(LED_Red_Pin, LED_Red_ON);
        LED_Red_Loop_Time = Loop_Delay_Time;
        LED_Red_Loop_Switch = false;
      }
      else {
        LED_Red_Loop_Time = LED_Red_Loop_Time + Loop_Delay_Time;
      }
    } // LED_Red_Loop_Switch
  } // Error_Sound_Count
} // LED_Red_Blink


void Bridge_Closed_Green_LED_OFF() {
  if (Bridge_Open_Detected == YES) {
    if (Obstacle_Detected == YES) {
      // Be carefull! It will overwriete Emergency Stop activated by other Sensor systems.
      Start_All_Trains();
    }
    Reset_All();
  }
} // Bridge_Closed_Green_LED_OFF


// Reset -----------------------------------------------------------------------


void Reset_All() {
  Reset_Pin_Output();
  Reset_Loop_Control();
  Reset_Buzzer_Sound();
  //
  Bridge_Open_Detected = NO;
  Obstacle_Detected    = NO;
  Offset_Delay_Time    = 0;
  Last_Sensor_Satus    = false;
  Current_Sensor_Satus = false;
 }  // Reset_All


void Reset_Pin_Output() {
  Buzzer_Off();
  digitalWrite(Relay_Start_Pin, Relay_OFF);
  digitalWrite(Relay_Stop_Pin, Relay_OFF);
  analogWrite(LED_Green_Pin, LED_Green_OFF);
  analogWrite(LED_Red_Pin, LED_Red_OFF);
  digitalWrite(USonic_Pin_Trig_1, LOW);
  digitalWrite(USonic_Pin_Trig_2, LOW);
  digitalWrite(Alarm_Status_Pin, Alarm_Status_OFF);
} // Reset_Pin_Output


void Reset_Loop_Control() {
  Alarm_Loop_Switch    = false;
  Alarm_Loop_Stop      = true;
  Alarm_Loop_Time      = 0; 
  Relay_Loop_Switch    = false;
  Relay_Loop_Time      = 0; 
  Buzzer_Loop_Switch   = false;
  Buzzer_Loop_Time     = 0; 
  LED_Red_Loop_Switch  = false;
  LED_Red_Loop_Time    = 0; 
} // Reset_Loop_Control


void Reset_Buzzer_Sound() {
  Buzzer_Max_Step      = 0;
  Buzzer_Step_Count    = 0;      
  Buzzer_Step_Time     = 0;
  Buzzer_Sound_ON      = NO;
  Reset_Sound_ON       = NO;
  Reset_Error_Detected = NO;
  Error_Sound_ON       = NO;
  Error_Sound_Count    = 0;  
} // Reset_Buzzer_Sound


// Check configuration -----------------------------------------------------------------------


void Check_Configuration_Parameters() {
  Check_GP2Y0A_Parameters();
  Check_Usonic_Parameters();
  Check_Relay_And_LED_Parameters();
  Check_Buzzer_Parameters();
  Check_Reset_Parameters();
  Check_Error_Parameters();
  Check_Process_Parameters();
} // Check_Configuration_Parameters


void Check_GP2Y0A_Parameters() {
  byte Test_Min_CM[3] = { 10,  4,  2 };
  byte Test_Max_CM[3] = { 80, 30, 15 };
  //
  if (IR_GP2Y0A_ON_1 == YES) {
    if (IR_GP2Y0A_Min_CM_1 < Test_Min_CM[IR_GP2Y0A_Type_1]) {
      PortPrint(F("Warning! IR_GP2Y0A_Min_CM_1 must be greater than or equal to (cm): ")); PortPrintln(Test_Min_CM[IR_GP2Y0A_Type_1]);
      Set_Error_Sound_Count(1);
    }
    if (IR_GP2Y0A_Max_CM_1 > Test_Max_CM[IR_GP2Y0A_Type_1]) {
      PortPrint(F("Warning! IR_GP2Y0A_Max_CM_1 must be less than or equal to (cm): ")); PortPrintln(Test_Max_CM[IR_GP2Y0A_Type_1]);
      Set_Error_Sound_Count(1);
    }
    if (Test_Min_CM[IR_GP2Y0A_Type_1] >= Test_Max_CM[IR_GP2Y0A_Type_1]) {
      PortPrint(F("ERROR! IR_GP2Y0A_Max_CM_1 must be grater than IR_GP2Y0A_Min_CM_1"));
      Set_Error_Sound_Count(1);
    }
  }
  //
  if (IR_GP2Y0A_ON_2 == YES) {
    if (IR_GP2Y0A_Min_CM_2 < Test_Min_CM[IR_GP2Y0A_Type_2]) {
      PortPrint(F("Warning! IR_GP2Y0A_Min_CM_2 should be greater than or equal to (cm): ")); PortPrintln(Test_Min_CM[IR_GP2Y0A_Type_2]);
      Set_Error_Sound_Count(1);
    }
    if (IR_GP2Y0A_Max_CM_2 > Test_Max_CM[IR_GP2Y0A_Type_2]) {
      PortPrint(F("Warning! IR_GP2Y0A_Max_CM_2 should be less than or equal to (cm): ")); PortPrintln(Test_Max_CM[IR_GP2Y0A_Type_2]);
      Set_Error_Sound_Count(1);
    }
    if (Test_Min_CM[IR_GP2Y0A_Type_2] >= Test_Max_CM[IR_GP2Y0A_Type_2]) {
      PortPrint(F("ERROR! IR_GP2Y0A_Max_CM_2 must be grater than IR_GP2Y0A_Min_CM_2"));
      Set_Error_Sound_Count(1);
    }
  }
} // Check_GP2Y0A_Parameters


void Check_Usonic_Parameters() {
  if (USonic_Sensor_ON_1 == YES) {
    if (USonic_Min_CM_1 < 3) {
      PortPrintln(F("Warning! USonic_Min_CM_1 must be greater than or equal to 3 cm."));
      Set_Error_Sound_Count(1);
    }
    if (USonic_Max_CM_1 > 100) {
      PortPrintln(F("Warning! USonic_Max_CM_1 must be less than or equal to 100 cm."));
      Set_Error_Sound_Count(1);
    }
    if (USonic_Min_CM_1 >= USonic_Max_CM_1) {
      PortPrintln(F("Warning! USonic_Max_CM_1 must be greater than USonic_Min_CM_1"));
      Set_Error_Sound_Count(1);
    }
  }
  //
  if (USonic_Sensor_ON_2 == YES) {
    if (USonic_Min_CM_2 < 3) {
      PortPrintln(F("Warning! USonic_Min_CM_2 must be greater than or equal to 3 cm."));
      Set_Error_Sound_Count(1);
    }
    if (USonic_Max_CM_2 > 100) {
      PortPrintln(F("Warning! USonic_Max_CM_2 must be less than or equal to 100 cm."));
      Set_Error_Sound_Count(1);
    }
    if (USonic_Min_CM_2 >= USonic_Max_CM_2) {
      PortPrintln(F("Warning! USonic_Max_CM_1 must be greater than USonic_Min_CM_1"));
      Set_Error_Sound_Count(1);
    }
  }
} // Check_Usonic_Parameters


void Check_Relay_And_LED_Parameters() {
  if (Relay_ON_Time < Loop_Delay_Time || Relay_OFF_Time < Loop_Delay_Time) {
    Set_Error_Sound_Count(1);
    PortPrint  (F("Waring! Relay_ON_Time and Relay_OFF_Time must be greater than Loop_Delay_Time: "));
    PortPrintln(Loop_Delay_Time);
    Set_Error_Sound_Count(1);
  }
  if (LED_Red_ON_Time < Loop_Delay_Time || LED_Red_OFF_Time < Loop_Delay_Time) {
    PortPrint  (F("Waring! LED_Red_ON_Time and LED_Red_OFF_Time must be greater than Loop_Delay_Time: "));
    PortPrintln(Loop_Delay_Time);
    Set_Error_Sound_Count(1);
  }
} // Check_Relay_And_LED_Parameters


void Check_Buzzer_Parameters() {
  if (Buzzer_Low_Time < Loop_Delay_Time || Buzzer_High_Time < Loop_Delay_Time) {
    PortPrint  (F("Waring! Buzzer_Low_Time and Buzzer_High_Time must be greater than Loop_Delay_Time: "));
    PortPrintln(Loop_Delay_Time);
    Set_Error_Sound_Count(1);
  }
  if (Buzzer_Low_Pitch < 1000) {
    PortPrintln(F("Waring! Buzzer_Low_Pitch must be greater than or equal to 1.000 Hz!"));
    Set_Error_Sound_Count(1);
  }
  if (Buzzer_High_Pitch > 5000) {
    PortPrintln(F("Waring! Buzzer_High_Pitch must be less than or equal to 5.000 Hz!"));
    Set_Error_Sound_Count(1);
  }
} // Check_Buzzer_Parameters


void Check_Reset_Parameters() {
  for (byte Index = 0; Index < Reset_Sound_Max_Step; Index++) {
    if (Reset_Sound_Duration_MS[Index] < Loop_Delay_Time) {
      PortPrint  (F("Waring! Reset_Sound_Duration_MS[")); PortPrint(Index); 
      PortPrint  (F("] must be greater than Loop_Delay_Time: ")); PortPrintln(Loop_Delay_Time);
      Set_Error_Sound_Count(1);
    }
  }
  for (byte Index = 0; Index < Reset_Sound_Max_Step; Index++) {
    if (Reset_Sound_Pitch_Hz[Index] < 1000  &&  Reset_Sound_Pitch_Hz[Index] != 0) {
      PortPrint  (F("Waring! Reset_Sound_Pitch_Hz[")); PortPrint(Index); 
      PortPrintln(F("] must be zero or greater than or equal to: 1.000 Hz!"));
      Set_Error_Sound_Count(1);
    }
  }
  for (byte Index = 0; Index < Reset_Sound_Max_Step; Index++) {
    if (Reset_Sound_Pitch_Hz[Index] > 5000) {
      PortPrint  (F("Waring! Reset_Sound_Pitch_Hz[")); PortPrint(Index); 
      PortPrintln(F("] must be less than or equal to 5.000 Hz!"));
      Set_Error_Sound_Count(1);
    }
  }
} // Check_Reset_Parameters


void Check_Error_Parameters() {
  for (byte Index = 0; Index < Error_Sound_Max_Step; Index++) {
    if (Error_Sound_Duration_MS[Index] < Loop_Delay_Time) {
      PortPrint  (F("Waring! Error_Sound_Duration_MS[")); PortPrint(Index); 
      PortPrint  (F("] must be greater than Loop_Delay_Time: ")); PortPrintln(Loop_Delay_Time);
      Set_Error_Sound_Count(1);
    }
  }
  for (byte Index = 0; Index < Error_Sound_Max_Step; Index++) {
    if (Error_Sound_Pitch_Hz[Index] < 1000  && Error_Sound_Pitch_Hz[Index] != 0) {
      PortPrint  (F("Waring! Error_Sound_Pitch_Hz[")); PortPrint(Index); 
      PortPrintln(F("] must be zoro or greater than or equal to: 1.000 Hz!"));
      Set_Error_Sound_Count(1);
    }
  }
  for (byte Index = 0; Index < Error_Sound_Max_Step; Index++) {
    if (Error_Sound_Pitch_Hz[Index] > 5000) {
      PortPrint  (F("Waring! Error_Sound_Pitch_Hz[")); PortPrint(Index); 
      PortPrintln(F("] must be less than or equal to 5.000 Hz!"));
      Set_Error_Sound_Count(1);
    }
  }
} // Check_Error_Parameters


void Check_Process_Parameters() {
  if (Loop_Delay_Time > 100) {
    PortPrint  (F("Warning! Loop_Delay_Time:")); PortPrint (Loop_Delay_Time);
    PortPrintln(F(" should not exceed 100 milliseconds! Proces execution will be slow."));
    Set_Error_Sound_Count(2);
  }
  if (Loop_Delay_Time < 10) {
    PortPrint  (F("FATAL error! Loop_Delay_Time:")); PortPrint (Loop_Delay_Time);
    PortPrintln(F(" must be 10 milliseconds or higer! Change and recompile!"));
    Set_Error_Sound_Count(3);
  }
} // Check_Process_Parameters


void Error_Alarm_Sound() {
  if (Error_Sound_Count > 0  &&  Reset_Sound_ON == NO) {
    if (Error_Sound_ON == NO) {
      analogWrite(LED_Green_Pin, LED_Green_OFF);
      analogWrite(LED_Red_Pin, LED_Red_ON);
      Buzzer_Load_Data(Error_Sound_Pitch_Hz, Error_Sound_Duration_MS, Error_Sound_Max_Step);
      Error_Sound_ON = YES;
    }
    //
    if (Buzzer_Sound_Process() == true) {  // Sound process ended
      Error_Sound_Count--;  // Next sound process (if any)
      if (Error_Sound_Count == 0) {
        Error_Sound_ON = NO;
      }
    }
  } 
}  // Error_Alarm_Sound


void Buzzer_Load_Data(const word Buzzer_Load_Pitch_Hz[], const word Buzzer_Load_Duration_MS[], const byte &Buzzer_Load_Max_Step) {
  for (byte Index = 0; Index < Buzzer_Load_Max_Step; Index++) {
    Buzzer_Pitch_Hz[Index]    =  Buzzer_Load_Pitch_Hz[Index];
    Buzzer_Duration_MS[Index] =  Buzzer_Load_Duration_MS[Index];
  }
  Buzzer_Max_Step = Buzzer_Load_Max_Step;
} // Buzzer_Load_Data 


bool Buzzer_Sound_Process() {
  if (Buzzer_Step_Time == 0) {
    Buzzer_Sound_ON = YES;
    if (Buzzer_Pitch_Hz[Buzzer_Step_Count] == 0) {
      Buzzer_Off();
    } 
    else {
      tone(Buzzer_Pin, Buzzer_Pitch_Hz[Buzzer_Step_Count]);
    }
    Buzzer_Step_Time = Buzzer_Step_Time + Loop_Delay_Time;
  }
  else {
    if (Buzzer_Step_Time >= Buzzer_Duration_MS[Buzzer_Step_Count]) {
      Buzzer_Step_Time = 0;
      Buzzer_Step_Count++;
      if (Buzzer_Step_Count >= Buzzer_Max_Step) {
        Buzzer_Step_Count = 0;
        Buzzer_Sound_ON = NO;
        Buzzer_Off();
        return true; // End of sound process
      }
    }
    else{
      Buzzer_Step_Time = Buzzer_Step_Time + Loop_Delay_Time;
    }
  }
  return false; // Continue sound process
} // Buzzer_Sound_Process


void Set_Error_Sound_Count(byte Set_Error_Count) {
if (Set_Error_Count > Error_Sound_Count) {
  Error_Sound_Count = Set_Error_Count;
  }
} // Set_Error_Sound_Count


// I2C laser setup -----------------------------------------------------------------------

void Setup_I2C_Lasers() {
  Initialize_I2C_bus();
  VL53L4CD_Calculate_Max_Lasers();
  VL53L4CD_Initialize_Error_check();
  VL53L4CD_Initialize_Read_Data();
  VL53L4CD_Check_I2C_Connected();
  VL53L4CD_Initialize_Lasers();
  VL53L4CD_Check_Parameters();
}


void Initialize_I2C_bus() {
  char Hex_To_Char[5] = "";
  Wire.begin();
  if (I2C_wire_Debug_Enabled == YES) {
    for (byte index = 0; index < 83; index++) { PortPrint(F("-")); } PortPrintln();
    PortPrintln(F("REMEMBER! disconnet power briefly to all sensors and Arduino MCU for correct reset!"));
    for (byte index = 0; index < 83; index++) { PortPrint(F("-")); } PortPrintln();
    snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%02X", VL53L4CD_I2C_Address);
    PortPrint(F("VL53L4CD_I2C_Address [HEX]: ")); PortPrintln(Hex_To_Char);
    snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%02X", TCA_PCA_I2C_Address);
    PortPrint(F("TCA_PCA_I2C_Address  [HEX]: ")); PortPrintln(Hex_To_Char);
    I2C_Detect();
  }
  Check_TCA_PCA_I2C_MUX();
  Wire.end();
  Wire.begin();
} // Initialize_I2C_bus


void Check_TCA_PCA_I2C_MUX() {
  if (TCA_PCA_I2C_MUX_ON == YES) {
    Wire.beginTransmission(TCA_PCA_I2C_Address);
    Wire.write(0);  // Close all TCA_PCA channels
    if (Wire.endTransmission() == 0) {
      TCA_PCA_I2C_MUX_Status = OK;
    }
    else {
      TCA_PCA_I2C_MUX_Status = Error;
      Set_Error_Sound_Count(1);
      Error_Sound_Count = 1;
      PortPrintln(F("ERROR: TCA_PCA_I2C_MUX_ON = YES - but no MUX is I2C connected!"));
    }
  }
} // Check_TCA_PCA_I2C_MUX(


void VL53L4CD_Calculate_Max_Lasers() {
  if (TCA_PCA_I2C_MUX_ON == YES  && TCA_PCA_I2C_MUX_Status == OK) {
    if (TCA_PCA_I2C_Model == TCA_PCA_9548A) {
      VL53L4CD_Max_Count = 8;      
    }
    else { // TCA_PCA_9546A
      VL53L4CD_Max_Count = 4;
    }
  }
  else { // First sensor
      VL53L4CD_Max_Count = 1;
  }
  //
  for (byte Index = 0; Index < VL53L4CD_Max_Count; Index ++) {
    if (VL53L4CD_Laser_I2C_ON[Index] == YES) {
      VL53L4CD_Laser_ON_List[VL53L4CD_Max_ON_Index] = Index;
      VL53L4CD_Max_ON_Index++;
    }
    else {
      VL53L4CD_Laser_ON_List[VL53L4CD_Max_ON_Index] = 0;
    }
  }
} // Calculate_Max_Lasers


void VL53L4CD_Initialize_Error_check() {
  // Clear data!
  for (byte Init_Laser_No = 0; Init_Laser_No < VL53L4CD_Max_Count; Init_Laser_No++) {
    VL53L4CD_Error_Status[Init_Laser_No]         = OK;
    VL53L4CD_Average_Block_MM[Init_Laser_No]     = 0;
    VL53L4CD_Average_Block_Count[Init_Laser_No]  = 0;
    VL53L4CD_Average_Ambient_KCPS[Init_Laser_No] = 0;
    VL53L4CD_Average_Ambient_Count[Init_Laser_No]= 0;
  }
} // Initialize_Error_check


void VL53L4CD_Initialize_Read_Data() {
    // Global sensor reading data (vl53l4cd_api.h)
    VL53L4CD_Result.range_status          = 0;
    VL53L4CD_Result.distance_mm           = 0;
    VL53L4CD_Result.ambient_rate_kcps     = 0;
    VL53L4CD_Result.ambient_per_spad_kcps = 0;
    VL53L4CD_Result.signal_rate_kcps      = 0;
    VL53L4CD_Result.signal_per_spad_kcps  = 0;
    VL53L4CD_Result.number_of_spad        = 0;
    VL53L4CD_Result.sigma_mm              = 0;
} // VL53L4CD_Initialize_Read_Data


void I2C_Detect() {
  byte I2C_status, I2C_address;
  char Hex_To_Char[5] = "";
  short nDevices;
  PortPrintln(F("Scanning I2C address range HEX: 0x01 - 0x77"));
  nDevices = 0;
  for(I2C_address = 1; I2C_address < 127; I2C_address++ ) {
    // Did the device acknowledge to the address?
    Wire.beginTransmission(I2C_address);
    I2C_status = Wire.endTransmission();
    snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%02X", I2C_address);
    if (I2C_status == 0)
    {
      PortPrint(F("  I2C device found at address [HEX]: ")); 
      PortPrintln(Hex_To_Char);
      nDevices++;
    }
    else if (I2C_status == 4) {
      PortPrint(F("Unknown error at address 0x / not connected / Pull up resistor missing"));
      PortPrintln(Hex_To_Char);
    }
  }
  if (nDevices == 0) {
    PortPrintln(F("  No I2C devices found!"));
  }
  PortPrintln(F("Done I2C scanning"));
} // I2C_Detect


void VL53L4CD_Check_I2C_Connected() {
  byte I2C_Check_Status = 0;
  byte Check_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Check_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (TCA_PCA_I2C_MUX_ON == YES  && TCA_PCA_I2C_MUX_Status == OK) {
      TCA_PCA_Select_Channel(Check_Laser_No);
    }
    if (VL53L4CD_Error_Status[Check_Laser_No] == OK) {
      Wire.beginTransmission(VL53L4CD_I2C_Address);
      I2C_Check_Status = Wire.endTransmission();
      VL53L4CD_Print_Error(I2C_Check_Status, Check_Laser_No, 1);
      if (I2C_Check_Status == 0  && VL53L4CD_Debug_Enabled[Check_Laser_No] == YES) {
        VL53L4CD_Print_Header(Check_Laser_No); PortPrintln(F("I2C Connected OK")); 
      }
    }
  }  // for
}  // VL53L4CD_Check_I2C_Connected


void VL53L4CD_Initialize_Lasers() {
  byte Init_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Init_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Error_Status[Init_Laser_No] == OK) {
      if (TCA_PCA_I2C_MUX_ON == YES  && TCA_PCA_I2C_MUX_Status == OK) {
        TCA_PCA_Select_Channel(Init_Laser_No);
      }
      VL53L4CD_Set_I2C_Adresse(Init_Laser_No);
      VL53L4CD_Check_Sensor_ID(Init_Laser_No);
      VL53L4CD_Setup_And_Initialize_Sensor(Init_Laser_No);
      VL53L4CD_Calibrate_Sensor();  // Alle enabled lasers
      VL53L4CD_Setup_And_Start_Continuous_Ranging(Init_Laser_No);
      delay (10);  // Stability
    }
  }
} // VL53L4CD_Initialize_Lasers


void VL53L4CD_Set_I2C_Adresse(const byte &Set_Laser_No) {
  // Set the samer address for all sensors (Adafruit = 0x29 / TS = 0x52)
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SetI2CAddress(VL53L4CD_I2C_Address);
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 2);
  }
} // VL53L4CD_Set_I2C_Adresse


void VL53L4CD_Check_Sensor_ID(const byte &ID_Laser_No) {
  char Hex_To_Char[7] = "";
  uint16_t VL53L4CD_ID = 0;
  if (VL53L4CD_Error_Status[ID_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[ID_Laser_No]->VL53L4CD_GetSensorId(&VL53L4CD_ID);
    }
    if (VL53L4CD_Debug_Enabled[ID_Laser_No] == YES) {
       snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%04X", VL53L4CD_ID);
      VL53L4CD_Print_Header(ID_Laser_No); PortPrint(F("Sensor ID: ")); PortPrintln(Hex_To_Char);
    }
    if (VL53L4CD_ID != VL53L4CD_Sensor_ID) {
      VL53L4CD_Print_Error(250, ID_Laser_No, 3);
  }
} // VL53L4CD_Check_Sensor_ID


void VL53L4CD_Setup_And_Initialize_Sensor(const byte &Set_Laser_No) {
  // Initialize and setup sensor
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) { 
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SensorInit();  // NOT InitSensor!
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 4);
  } 
  // Set offset distance mm.
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) { 
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SetOffset(VL53L4CD_Offset_Dist_MM[Set_Laser_No]);
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 5);
  } 
  // Set crosstalk KCPS
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) { 
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SetXtalk(VL53L4CD_Xtalk_KCPS[Set_Laser_No]);
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 6);
  }

  
} // VL53L4CD_Setup_And_Initialize_Sensor


// I2C laser calibration ------------------------------------------------------------------------------


void VL53L4CD_Calibrate_Sensor() {
  // Calibrate Distance first (1)
  if(VL53L4CD_Offset_Calibrate == YES) {
    VL53L4DC_Calibrate_Offset_Distance();  // Loop all active Lasers
  }
  // Calibrate Xtalk next (2)
  if(VL53L4CD_Xtalk_Calibrate == YES) {
    VL53L4DC_Calibrate_Xtalk_Distance();  // Loop all active Lasers
  }
} // VL53L4CD_Calibrate_Sensor


void VL53L4DC_Calibrate_Offset_Distance() {
  const int16_t Target_Distance_MM  = 100; // ST recommandation
  const int16_t Number_Of_Samples   = 20;  // ST recommandation
  int16_t Target_Measured_Offset_MM = 0;   // +/- mm
  byte Offset_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Offset_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Error_Status[Offset_Laser_No] == OK) {
      PortPrintln();
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      VL53L4CD_Print_Header(Offset_Laser_No); PortPrintln(F("Calibrating sensor distance procedure."));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      PortPrintln(F("Use GRAY cardboard / paper as reflektion object and indoor low light environment."));
      PortPrint  (F("Target distance must be [mm]: ")); PortPrintln(Target_Distance_MM);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Offset_Laser_No]->VL53L4CD_GetOffset(&Target_Measured_Offset_MM);
      PortPrint  (F("Current offset distance [mm]: ")); PortPrintln(Target_Measured_Offset_MM);
      VL53L4CD_Print_Error(VL53L4CD_Status, Offset_Laser_No, 8);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Offset_Laser_No]->VL53L4CD_CalibrateOffset( \
                        Target_Distance_MM, &Target_Measured_Offset_MM, Number_Of_Samples);
      PortPrint  (F("NEW calibration offset [mm]: ")); PortPrintln(Target_Measured_Offset_MM);
      VL53L4CD_Print_Error(VL53L4CD_Status, Offset_Laser_No, 9);
      PortPrint  (F("Remember to type in new value: VL53L4CD_Offset_Dist_MM[")); PortPrint(Offset_Laser_No); PortPrintln(F("]"));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      delay(10000);
    }
  } 
} // VL53L4DC_Calibrate_Offset_Distance


void VL53L4DC_Calibrate_Xtalk_Distance() {
  uint16_t Measured_Xtalk_KCPS = 0;
  int16_t  Number_Of_Samples   = 20;  // ST recommandation
  byte Xtalk_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Xtalk_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Laser_I2C_ON[Xtalk_Laser_No] == YES  &&  VL53L4CD_Error_Status[Xtalk_Laser_No] == OK) {
      PortPrintln();
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      VL53L4CD_Print_Header(Xtalk_Laser_No); PortPrintln(F("Calibrating sensor Crosstalk procedure."));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      PortPrintln(F("Use GRAY cardboard / paper as reflektion object and indoor low light environment."));
      PortPrint  (F("Maximum measured ranging distance without under-ranging [mm]: ")); PortPrintln(VL53L4CD_Xtalk_Distance_MM);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Xtalk_Laser_No]->VL53L4CD_GetXtalk(&Measured_Xtalk_KCPS);
      PortPrint  (F("Current crosstalk compensation [KPCS]: ")); PortPrintln(Measured_Xtalk_KCPS);
      VL53L4CD_Print_Error(VL53L4CD_Status, Xtalk_Laser_No, 10);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Xtalk_Laser_No]->VL53L4CD_CalibrateXtalk( \
                        VL53L4CD_Xtalk_Distance_MM, &Measured_Xtalk_KCPS, Number_Of_Samples);
      PortPrint  (F("NEW crosstalk compensation [KCPS]: ")); PortPrintln(Measured_Xtalk_KCPS);
      VL53L4CD_Print_Error(VL53L4CD_Status, Xtalk_Laser_No, 11);
      PortPrint  (F("Remember to type in new value: VL53L4CD_Xtalk_KCPS[")); PortPrint(Xtalk_Laser_No); PortPrintln(F("]"));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      delay(10000);
    }
  } 
} // VL53L4DC_Calibrate_Xtalk_Distance


//  I2C laser processing ---------------------------------------------------------------------


void TCA_PCA_Select_Channel(const byte &TCA_PCA_Channel_No) {
  // 4 or 8 channel I2C Multiplexer
  if (VL53L4CD_Error_Status[TCA_PCA_Channel_No] == OK) {
    byte End_Status = 0;
    Wire.beginTransmission(TCA_PCA_I2C_Address);
    Wire.write(1 << TCA_PCA_Channel_No);  // Shit left one bit
    End_Status = Wire.endTransmission();
    // 
    if (End_Status > 0) {
      Set_Error_Sound_Count(1);
      VL53L4CD_Error_Status[TCA_PCA_Channel_No] = Error;
      PortPrint(F("TCA_PCA channel [")); PortPrint(TCA_PCA_Channel_No); PortPrint(F("] ERROR! "));
      VL53L4CD_Print_Error_Text (End_Status);
    }
  }
} // TCA_PCA_Select_Channel


void VL53L4CD_Setup_And_Start_Continuous_Ranging(const byte &Start_Laser_No) {
  // Program the fastest speed
  const word Time_Budget_MS   = 10;  // 10 Milliseconds
  const word Inter_Measure_MS = 0;   //  0 Milliseconds
  if (VL53L4CD_Error_Status[Start_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[Start_Laser_No]->VL53L4CD_SetRangeTiming(Time_Budget_MS, Inter_Measure_MS);
    VL53L4CD_Print_Error(VL53L4CD_Status, Start_Laser_No, 12);
  }
  // Start Measurements
  if (VL53L4CD_Error_Status[Start_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[Start_Laser_No]->VL53L4CD_StartRanging();
    VL53L4CD_Print_Error(VL53L4CD_Status, Start_Laser_No, 13);
  }
} // VL53L4CD_Setup_And_Start_Continuous_Ranging


bool VL53L4CD_Read_Sensors_Sequentially() {
  if (VL53L4CD_Loop_ON_Index >= VL53L4CD_Max_ON_Index) {
    VL53L4CD_Loop_ON_Index = 0;
  }
  VL53L4CD_Active_Sensor = VL53L4CD_Laser_ON_List[VL53L4CD_Loop_ON_Index];
  VL53L4CD_Loop_ON_Index++;
  return VL53L4CD_Read_One_Sensor(VL53L4CD_Active_Sensor);
} // VL53L4CD_Read_Sensors_Sequentially


bool VL53L4CD_Read_One_Sensor (const byte &Read_One_Laser) {
  if (VL53L4CD_Laser_I2C_ON[Read_One_Laser] == YES  &&  VL53L4CD_Error_Status[Read_One_Laser] == OK) {
    VL53L4CD_Active_Sensor = Read_One_Laser;
    if (TCA_PCA_I2C_MUX_ON == YES  &&  TCA_PCA_I2C_MUX_Status == OK) {
      TCA_PCA_Select_Channel(Read_One_Laser);
    }
    if (VL53L4CD_Error_Status[Read_One_Laser] == OK) {
      return VL53L4CD_Read_And_Check(Read_One_Laser);
    }
  }
  return false;
} // VL53L4CD_Read_One_Sensor


bool VL53L4CD_Read_And_Check(const byte &Read_Laser_No) {
  bool Return_Status = false;
  byte New_Data_Ready = 0;
  //
  VL53L4CD_Status = VL53L4CD_Lasers_List[Read_Laser_No]->VL53L4CD_CheckForDataReady(&New_Data_Ready); // 1,2 milliseconds
  if ((!VL53L4CD_Status == OK) && (New_Data_Ready == YES)) {
     // (Mandatory) Clear HW interrupt to restart measurements
    VL53L4CD_Lasers_List[Read_Laser_No]->VL53L4CD_ClearInterrupt(); // 0,5 milliseconds
    VL53L4CD_Lasers_List[Read_Laser_No]->VL53L4CD_GetResult(&VL53L4CD_Result);  // 4,0 Milliseconds
    VL53L4CD_Print_Debug_Info(Read_Laser_No);
    VL53L4CD_check_Ambient_Light(Read_Laser_No);
    //
    if (VL53L4CD_Result.range_status == VL53L4CD_ERROR_NONE  &&  
        VL53L4CD_Result.distance_mm > 0  && 
        VL53L4CD_Result.sigma_mm <= VL53L4CD_Max_Sigma_MM  &&
        VL53L4CD_Result.signal_per_spad_kcps >=  VL53L4CD_Min_Signal_KPCS) { 
      VL53L4CD_Blocking_Calculate_And_Check(Read_Laser_No);
      Return_Status = VL53L4CD_Check_Min_And_Max_Distance(Read_Laser_No);
    }
  }
  return Return_Status;
} //VL53L4CD_Read_And_Check


void VL53L4CD_check_Ambient_Light(const byte &Ambient_Laser_No) {
  word Average_Ambient_KCPS = 0;
  VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] = VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] + (long)VL53L4CD_Result.ambient_rate_kcps;
  if (VL53L4CD_Average_Ambient_Count[Ambient_Laser_No] == VL53L4CD_Max_Ambient_Count) {
    Average_Ambient_KCPS = (word)(VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] / VL53L4CD_Average_Ambient_Count[Ambient_Laser_No]);     
    if (Average_Ambient_KCPS > VL53L4CD_Max_Ambient_KCPS) {
      Set_Error_Sound_Count(1);
      VL53L4CD_Print_Header(Ambient_Laser_No); 
      PortPrint(F("ERROR! Ambient light is to high (KCPS): ")); PortPrintln(Average_Ambient_KCPS);
    }
    VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] = 0;
    VL53L4CD_Average_Ambient_Count[Ambient_Laser_No] = 0;
  }
  else {
    VL53L4CD_Average_Ambient_Count[Ambient_Laser_No]++;
  }
}  // VL53L4CD_check_Ambient_Light


void VL53L4CD_Print_Debug_Info(const byte &Info_Laser_No) {
  char Print_Result[120] = "";
  if (VL53L4CD_Debug_Enabled[Info_Laser_No] == YES) {
    snprintf(Print_Result, sizeof(Print_Result), 
      "Status = %2u, Distance = %4u mm, Signal rate= %5u kcps, Sigma = %3u, Ambient light = %5u",
      VL53L4CD_Result.range_status, VL53L4CD_Result.distance_mm, VL53L4CD_Result.signal_rate_kcps, 
      VL53L4CD_Result.sigma_mm, VL53L4CD_Result.ambient_rate_kcps);
    VL53L4CD_Print_Header(Info_Laser_No); PortPrintln(Print_Result);
    if (VL53L4CD_Print_Range_Status == YES) {
      VL53L4CD_Print_Ranging_Error(Info_Laser_No, VL53L4CD_Result.range_status);
    }
  }
} // VL53L4CD_Print_Debug_Info


bool VL53L4CD_Check_Min_And_Max_Distance(const byte &Dist_Laser_No) {
  if (VL53L4CD_Result.distance_mm >= VL53L4CD_Min_Distance_MM[Dist_Laser_No]  &&  \
      VL53L4CD_Result.distance_mm <= VL53L4CD_Max_Distance_MM[Dist_Laser_No]) {
    if (VL53L4CD_Print_Object_Detect == YES) {
      VL53L4CD_Print_Header(Dist_Laser_No); PortPrint(F("Object detected (mm): ")); 
      PortPrint(VL53L4CD_Result.distance_mm); PortPrint(F(" Detection zone: "));
      PortPrint(VL53L4CD_Min_Distance_MM[Dist_Laser_No]); PortPrint(F(" - "));
      PortPrintln(VL53L4CD_Max_Distance_MM[Dist_Laser_No]);
    }
    return true;
  }
  return false;
} // Check_Min_And_Max_Distance


void VL53L4CD_Blocking_Calculate_And_Check(const byte &Average_Laser_No) {
  word Average_Distance_MM; 
  if (VL53L4CD_Blocking_Detect[Average_Laser_No] == YES) {
    VL53L4CD_Average_Block_MM[Average_Laser_No] += (long)VL53L4CD_Result.distance_mm;
    if (VL53L4CD_Average_Block_Count[Average_Laser_No] == VL53L4CD_Max_Block_Count) {
      Average_Distance_MM = (word)(VL53L4CD_Average_Block_MM[Average_Laser_No] / VL53L4CD_Average_Block_Count[Average_Laser_No]);
      VL53L4CD_Average_Block_MM[Average_Laser_No]     = 0;
      VL53L4CD_Average_Block_Count[Average_Laser_No]  = 0;
      //
      VL53L4CD_Blocking_Check(Average_Laser_No, Average_Distance_MM);
    }
    else {
      VL53L4CD_Average_Block_Count[Average_Laser_No]++;
    }
  }
} // VL53L4CD_Blocking_Calculate_And_Check


void VL53L4CD_Blocking_Check(const byte &Check_Laser_No, const word &Check_Average_Distance_MM) {
  if (Check_Average_Distance_MM <= VL53L4CD_Max_BLock_Dist_MM) {
    Set_Error_Sound_Count(1);
    VL53L4CD_Print_Header(Check_Laser_No); 
    PortPrint(F("ERROR! Sensor beam blocked (mm): ")); PortPrint(Check_Average_Distance_MM);
    PortPrint(F(" Blocking zone: 0 - ")); PortPrintln(VL53L4CD_Max_BLock_Dist_MM);
  }
} // L53L4CD_Blocking_Check


// I2C lasor error handling ------------------------------------------------------------------------


void VL53L4CD_Check_Parameters() {
  byte Chek_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Chek_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Min_Distance_MM[Chek_Laser_No] < 20) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint(F("Warning! VL53L4CD_Min_Distance_MM[")); PortPrint(Chek_Laser_No);
      PortPrint(F("] = ")); PortPrint(VL53L4CD_Min_Distance_MM[Chek_Laser_No]); 
      PortPrintln(F(" Should be greater than or equal to 25 mm."));
      Set_Error_Sound_Count(1);
    }
    if (VL53L4CD_Min_Distance_MM[Chek_Laser_No] <= VL53L4CD_Max_BLock_Dist_MM) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint(F("Warning! VL53L4CD_Min_Distance_MM[")); PortPrint(Chek_Laser_No);
      PortPrint(F("] = ")); PortPrint(VL53L4CD_Min_Distance_MM[Chek_Laser_No]); 
      PortPrint(F(" Should be greater than or equal to VL53L4CD_Max_BLock_Dist_MM = "));
      PortPrintln(VL53L4CD_Max_BLock_Dist_MM);
      Set_Error_Sound_Count(1);
    }
    if (VL53L4CD_Max_Distance_MM[Chek_Laser_No] > 800) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint(F("Warning! VL53L4CD_Max_Distance_MM[")); PortPrint(Chek_Laser_No); 
      PortPrint(F("] = ")); PortPrint(VL53L4CD_Max_Distance_MM[Chek_Laser_No]);
      PortPrintln(F(" Should be less than or equal to 800 mm."));
      Set_Error_Sound_Count(1);
    }
    if (VL53L4CD_Min_Distance_MM[Chek_Laser_No] >=VL53L4CD_Max_Distance_MM[Chek_Laser_No]) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint (F("ERROR! VL53L4CD_Max_Distance_MM must be greater than VL53L4CD_Min_Distance_MM"));
      Set_Error_Sound_Count(1);
    }
  }
  //
  if (TCA_PCA_I2C_MUX_ON == YES  &&  TCA_PCA_I2C_MUX_Status == OK  &&  VL53L4CD_Max_ON_Index == 0) {
    PortPrintln(F("Warning! TCA_PCA_I2C_MUX_ON = YES - but no lasers [0-7] are enabled!"));
    Set_Error_Sound_Count(1);
  }
} // Check_VL53L4CD_Parameters


void VL53L4CD_Print_Error(const byte &Print_Error_Status, const byte &Print_Laser_No, const byte &Function_Numer) {
  if (Print_Error_Status != 0) {
    Set_Error_Sound_Count(1);
    VL53L4CD_Error_Status[Print_Laser_No] = Error;
    VL53L4CD_Print_Header(Print_Laser_No);
    switch (Function_Numer) {
      case  1: PortPrint(F("Check_I2C_Connected"));    break;
      case  2: PortPrint(F("_Set_I2C_Adresse"));       break;
      case  3: PortPrint(F("GetSensorID"));            break;
      case  4: PortPrint(F("SensorInit"));             break;
      case  5: PortPrint(F("SetOffset"));              break;
      case  6: PortPrint(F("SetXtalk"));               break;
      case  7: PortPrint(F("SetDetectionThresholds")); break;
      case  8: PortPrint(F("GetOffset"));              break;
      case  9: PortPrint(F("CalibrateOffset"));        break;
      case 10: PortPrint(F("GetXtalk"));               break;
      case 11: PortPrint(F("CalibrateXtalk"));         break;
      case 12: PortPrint(F("SetRangeTiming"));         break;
      case 13: PortPrint(F("StartRanging"));           break;
      default:
        PortPrintln(F("Unknown function!"));
    }
    PortPrint(F(" - ERROR! ")); 
    VL53L4CD_Print_Error_Text (Print_Error_Status);
  }
}


void VL53L4CD_Print_Error_Text(const byte &Print_Error_Status) {
  switch (Print_Error_Status) {
    case   0: break; // VL53L4CD_ERROR_NONE
    case   1: PortPrintln(F("Data too long to fit in transmit buffer.")); break;
    case   2: PortPrintln(F("Received NACK on transmit of address."));    break;
    case   3: PortPrintln(F("Received NACK on transmit of data."));       break;
    case   4: PortPrintln(F("Other error."));                             break;
    case   5: PortPrintln(F("Timeout"));                                  break;
    case 250: PortPrintln(F("Wrong sensor ID."));                         break;
    case 253: PortPrintln(F("VL53L4CD_ERROR_XTALK_FAILED."));             break;
    case 254: PortPrintln(F("VL53L4CD_ERROR_INVALID_ARGUMENT."));         break;
    case 255: PortPrintln(F("VL53L4CD_ERROR_TIMEOUT."));                  break;
    default:
      PortPrintln(F("Unknown error!"));
  }
} // L53L4CD_Print_Error_Text


void VL53L4CD_Print_Ranging_Error(const byte &Print_Ranging_Error, const byte &Range_Laser_No) {
  // Some error occurred, print it out!
  if (Print_Ranging_Error != 0) {
    VL53L4CD_Print_Header(Range_Laser_No);
  }
  switch (Print_Ranging_Error) {
    case   0: break; // Returned distance is valid
    case   1: PortPrintln(F("WARNING! Sigma is above the defined threshold."));            break;
    case   2: PortPrintln(F("WARNING! Signal is below the defined threshold."));           break;
    case   3: PortPrintln(F("ERROR! Measured distance is below detection threshold."));    break;
    case   4: PortPrintln(F("ERROR! Phase out of valid limit (max 1.200 mm)."));           break;
    case   5: PortPrintln(F("ERROR! Hardware fail."));                                     break;
    case   6: PortPrintln(F("WARNING! Phase valid but no wrap around check performed."));  break;
    case   7: PortPrintln(F("ERROR! Wrapped target, phase does not match."));              break;
    case   8: PortPrintln(F("ERROR! Processing fail."));                                   break;
    case   9: PortPrintln(F("ERROR! Crosstalk signal fail."));                             break;
    case  10: PortPrintln(F("ERROR! Interrupt error."));                                   break;
    case  11: PortPrintln(F("ERROR! Merged target."));                                     break;
    case  12: PortPrintln(F("ERROR! Signal is too low."));                                 break;
    case 255: PortPrintln(F("ERROR! Other error (e.g boot)."));                            break;
    default:
      PortPrintln(F("UNKNOWN error!"));
  }
} // VL53L4CD_Print_Ranging_Error


void VL53L4CD_Print_Header(const byte &Header_Laser_No) {
  // General header info
  PortPrint(F("VL53L4CD [")); PortPrint(Header_Laser_No); PortPrint(F("] "));
}


// end of program
