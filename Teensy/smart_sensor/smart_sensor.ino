// #include "uart_sender.h"
#include "callbacks.h"
#include <Wire.h>


void setup() {
  Serial.begin(115200);
  setup_imu();
  setup_lidar(l1);
  setup_lidar(l2);
  UART.begin(115200);

}

void loop() {
    if(ICM.dataReady()){
        ICM.getAGMT();
        bump_sens(&ICM);
    }

}


void receiveEvent(int numBytes){
  int message = Wire.read();
  if(message > 0){
    Serial.println("boop a beep bop");
  }
}
