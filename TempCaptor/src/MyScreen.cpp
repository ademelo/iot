#include <Arduino.h>
#include "MyScreen.h"

MyScreen::MyScreen(String message){
  //Serial.begin(115200);
  Serial.println("Create MyScreen");
}

void MyScreen::print(){
  Serial.println(_message);
}
