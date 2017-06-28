#include <Arduino.h>
#include <U8g2lib.h>
#ifndef MyScreen_h
#define MyScreen_h

class MyScreen{
  public:
    MyScreen(String message);
    void print();
  private:
    String _message;
};
#endif
