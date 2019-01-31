/* Dorothy il fregno che si muove da solo
 ---------------------------------------------- 
 by Pierpaolo Granello e Alessandro Dei Giudici

  Laboratorio Multidisciplinare di Elettronica
  Magistrale Ingegneria Elettronica - Sapienza
 ----------------------------------------------
*/

#include <SoftwareSerial.h>
#include "PWMServo.h"
#include "TFMini.h"
#include <SparkFun_TB6612.h>  //Contains the class Motor
#include <avr/power.h>

#define Min_Tilt  50    //Tilt -12.5 degree
#define Mid_Tilt  62.5  //Servo Mid Position
#define Max_Tilt  75    //Tilt +12.5 degree

#define Min_Pan   52.5  //Pan -22.5 degree
#define Mid_Pan   75    //Servo Mid Position
#define Max_Pan   97.5  //Pan +22.5 degree

//(Writes distance and Strenght for each location)
#define Iter_Tilt   6     //Points per Row
#define Iter_Pan    19    //Points per Column

// Pins for all inputs, PWM defines must be on PWM pins
#define AIN1 5  //2 Verde
#define BIN1 7  //7 Arancione
#define AIN2 4  //4 Blu
#define BIN2 8  //8 Rosso
#define PWMA 3  //5 Viola
#define PWMB 11 //6 Marrone
#define STBY 6  //  Giallo

//Servo Setup
PWMServo TILT;           //Create servo object to control a servo
PWMServo PAN;            //Create servo object to control a servo
//Serial port Setup
//Uno RX (TFMINI TX verde),Uno TX (TFMINI RX bianco)
SoftwareSerial mySerial(12 , 13); 
TFMini tfmini;

//Motors configuration line up with function like forward: 1,-1
const int offsetA = 1;
const int offsetB = 1;
Motor LEFT = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
Motor RIGHT = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
//Allocating Memory for LIDAR results
int *LiDAR_Radius = malloc(Iter_Pan*sizeof(uint16_t*));

boolean dir = 0;
int search = 0;   // index for ambient scan
int xPan = 0;     // for storing radius distance
int yTilt = 0;    // for storing radius distance
float xpos = Min_Pan;   // Index for servo PAN position
float ypos = Min_Tilt;  // Index for servo TILT position
char  xscanDirection = 0;
char  yscanDirection = 0;
float pi = 3.14159265;
float deg2rad = pi / 180.0;

//Functions
void Pan_Flow();
void Tilt_Flow();
void lidar_acquisition();
//void printMatrix();
int ChooseDirection();
void ChangeDirection();
void STOP();


void setup() {
  attachInterrupt(digitalPinToInterrupt(2), STOP, LOW);
  PAN.attach(SERVO_PIN_A);       // Giallo
  TILT.attach(SERVO_PIN_B);      // Viola 
  PAN.write(Min_Pan);
  TILT.write(Min_Tilt);

  Serial.begin(115200); //Initialize HW serial port (debug port)
  while (!Serial);      //Wait for serial port.
  Serial.println ("Initializing...");
  mySerial.begin(TFMINI_BAUDRATE);  //data rate for the SWSerial
  tfmini.begin(&mySerial);  //Initialize the TF Mini sensor
}

void loop() {
  while (search < 8) {
    for (int i=0; i<Iter_Pan; ++i) {
      LiDAR_Radius[i] = 100;
    } 
    Pan_Flow();       //Servo and Lidar actions

    if(ChooseDirection()==1){   // Go Straight
      search = 0;
      forward(LEFT,RIGHT,100);
      delay(1500);
      brake(LEFT, RIGHT);
    }
    else {
      ChangeDirection(); 
    }
  }
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
}
