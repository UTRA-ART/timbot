/*
  Library for logging into serial monitor for Fault Monitor
  Written by James Huynh - Mar 2025
*/
#ifndef Logger_h
#define Logger_h

#include "Arduino.h"

class Logger {
public:
  Logger(bool enabled);
  void logVoltage(double voltage);
  void logCurrent(double current);
  void logTemperature(double temperature);
  void printLog();
private:
  bool enable_;
  void printTimestamp();
  double voltage_;
  double current_;
  double temperature_;
};


#endif
#include <Arduino.h>
#include <time.h>

bool enable_ = false;
double voltage_ = 0.0;
double current_ = 0.0;
double temperature_ = 0.0;

Logger::Logger(bool enable) {
  enable_ = enable;
}
void Logger::logVoltage(double voltage) {
  voltage_ = voltage;
}
void Logger::logCurrent(double current) {
  current_ = current;
}
void Logger::logTemperature(double temperature) {
  temperature_ = temperature;
}
void Logger::printLog() {
  printTimestamp();
  char buf[100];
  char volt[10];
  char curr[10];
  char temp[10];
  dtostrf(voltage_, 1, 2, volt);
  dtostrf(current_, 1, 2, curr);
  dtostrf(temperature_, 1, 2, temp);
  
  sprintf(buf, " Voltage: %s Current: %s Temperature: %s", volt, curr, temp);
  Serial.print(buf);
  Serial.println();
}

void Logger::printTimestamp() {
  if (enable_) {
    long time = millis() / 1000;
    int hrs = (time - (time % 3600)) / 3600;
    time -= hrs * 3600;
    int mins = (time - (time % 60)) / 60;
    time -= mins * 60;

    char buf[100];
    sprintf(buf, "[%02d:%02d:%02d] ", hrs, mins, time);
    Serial.print(buf);
  }
}
