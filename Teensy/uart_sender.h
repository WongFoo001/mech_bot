#include "Arduino.h"
#include <ArduinoEigen.h>

using namespace Eigen;

#define UART Serial2

// send bytes
void send_er(uint8_t buf[]){
  //int bytesSent = UART.write(buf, 2);
  UART.write(buf, 2);
//  Serial.print("buffer ");
  Serial.print("Packet");
  Serial.print(": ");
  Serial.print((char)buf[0]);
  Serial.print(" ");
  Serial.println(buf[1]);
}

void eigen_print(VectorXd vec, String nam){
  int len = vec.norm();
  Serial.print(nam);
  Serial.print(": ");
  for(int i = 0; i < len; i++){
    Serial.print(vec[i]);
    Serial.print(" ");
  }
  Serial.println("");
}
