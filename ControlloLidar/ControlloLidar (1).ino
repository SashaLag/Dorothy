#include <SoftwareSerial.h>
#include "TFMini.h"

//Uno RX (TFMINI TX), Uno TX (TFMINI RX)
SoftwareSerial mySerial(12,13);
TFMini tfmini;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println ("Initializing...");
  mySerial.begin(TFMINI_BAUDRATE);
  tfmini.begin(&mySerial);    
}


void loop() {
  uint16_t dist = tfmini.getDistance();

  Serial.print(dist);
  Serial.print(" cm      ");

  //Wait some time before next measurement
  delay(25);  
}


