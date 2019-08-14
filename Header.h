#include <Encoder.h>
#include <TimerOne.h>
#include <TimerThree.h>

// Sensor pins
#define sensorpinL A2
#define sensorpinM A3
#define sensorpinR A4

//LED Pin
#define LED_PIN     10
#define NUM_LEDS    2

//LED array to control which LED we manipulate
CRGB leds[NUM_LEDS];

// PWM and DIR pins
#define PWMR 4
#define PWML 9
#define DIRR 5
#define DIRL 6

// Encoder pins
#define SIGR1 3
#define SIGR2 2
#define SIGL1 19
#define SIGL2 18

// Set up encoder readings
Encoder myEnc_1(SIGR2, SIGR1);//M1-Right (2,3) = Robot_forward
Encoder myEnc_2(SIGL1, SIGL2);//M2-Left (19,18) = Robot_forward

// Hardware parameters
const float r =0.016; // radious of the wheels
const float C = 8344; // 298x28
const float b = 0.094; //from tip to tip

// Inturrupt sampling time 100Hz
const float dt = 0.01; // [s]

// Main loop sampling time
const int sampling_time = 100;  // [ms]

// Params for model controller: p1 = p2 = 120 rad/s [scale by 3 to make it faster]
const float scale = 3;
const float a_ = 3.29873105632614*scale;
const float b_ = -0.993558700820458*scale;
const float c_ = 5.55328540582497*scale;
const float d_ = -3.24811305031929*scale;

// Robo position struct
typedef struct {
  float x;
  float y;
  float theta;
}robot_pos;

// Initialize global structs
robot_pos current_pos ={0,0,0};

// Time keeping varibales
unsigned long current_time = 0;
unsigned long previous_time = 0;

// Variables for encoder readings
long oldPosition_R  = 0;
long oldPosition_L  = 0;

// Variables for encoder values per time sample
float NL,NR; 

// Variables for distance measurements
float left_distance = 0;
float middle_distance = 0;
float right_distance = 0;

// Varibales for desired and real velocities
float Vr, Wr;
float WLd, WRd; 
float WLr, WRr; 
float Vd, Wd;

// Flag for initialization of the model based controller
int ctr_Flag = 0;