/*
Edited Dual-core FAULT MONITOR
March 7, 2026
Reagan Hu, Zakiul and Halley Martynov

This code handles sensing all the possible issues within the rover, and shuts off power if anything happens.
- Voltage
- Current
- Alarm states
- Temperature

As well, it controls fans through PWM.
*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Logger.h"

//CONSTANTS
#define VOLTAGE_REF 3.3  // ESP32 uses 3.3V reference
#define ADC_MAX 4095  // ESP32 ADC resolution (12-bit)
//Sensors
const int tempsensor = 12; //analog
const int currentsensor = 14; //analog
const int volt_sensor = 4; //analog

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(tempsensor);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

const int numberOfDevices = 2;  // Number of temperature devices found

DeviceAddress tempDeviceAddress;  // We'll use this variable to store a found device address




//Motor alarm state pins
const int alarm_left = 13;
const int alarm_right = 2;

//Output LEDs
const int current_Error_LED = 26;      // red
const int voltage_Error_LED = 25;      // blue
const int temperature_Error_LED = 33;  // green
const int alarm_Error_LED = 32;        // yellow

//Control for outside parts
const int power_control = 15;          //This pin is connected to the relay to shut down power when an error occurs, write HIGH to shut off.
const int fan = 19;  //fan PWM output pin
const int stop_all = 34;  // set low to stop stuff
const int soft_start = 16;

//HYPERPARAMETERS
//CHANGE TO DESIRED VALUES
const float MAX_SAFE_CUR = 10;
// This will shut down the rover if it reaches this temp
const int MAX_SAFE_TEMP = 100;
//the suggested temperatures change the linear curve of the fans
const int MAX_SUGGESTED_TEMP = 70;
const int MIN_SUGGESTED_TEMP = 20;

// variables for storage

float tempc = 0.0;                                                   //temperature in Celsius
float vout = 0.0;                                                    //temporary variable to hold sensor reading
float AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0, AcsValueF = 0.0;  //Holds values for calculating current
float tempC;

//error states
int temperature_error = 0;
int current_error = 0;
int alarm_error = 0;
int voltage_error = 0;

// restart settings
int robot_status = 0;

// Default messages
char ok_msg[2] = "O";
char current_msg[2] = "C";
char undervoltage_msg[2] = "V";
char temperature_msg[2] = "T";
char alarm_msg[2] = "A";
char user_msg[2] = "U";


//used for temperature averaging
const int avgMax = 5;        // Change the amount of previous temperatures to average out
const int defaultTemp = 10;  // Change the default temperature values in the temperature array, affects initial fan response

int countArray[avgMax];  // Array to store temperature values
int avgCount = 0;        // Index to store temperature values at


Logger logger(true);

SemaphoreHandle_t mutex;

// Setup function, initializes pins + starts rover
void setup() {
  {
    pinMode(tempsensor, INPUT);
    pinMode(currentsensor, INPUT);
    pinMode(alarm_left, INPUT);
    pinMode(alarm_right, INPUT);
    pinMode(volt_sensor, INPUT);
    pinMode(fan, OUTPUT);
    pinMode(current_Error_LED, OUTPUT);
    pinMode(voltage_Error_LED, OUTPUT);
    pinMode(alarm_Error_LED, OUTPUT);
    pinMode(temperature_Error_LED, OUTPUT);
    pinMode(power_control, OUTPUT);
    pinMode(stop_all, OUTPUT);
    pinMode(soft_start, OUTPUT);




  }


  // Initializes values for the temperature averaging array
  for (int i = 0; i < avgMax; i++) {
    countArray[i] = defaultTemp;
  }

  Serial.begin(115200);

  //Initializing Mutex  
  mutex=xSemaphoreCreateMutex();

  //Initializing tasks for the two cores
  xTaskCreatePinnedToCore(
    coreZeroTasks,
    "coreZero",
    4096,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    coreOneTasks,
    "coreOne",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  

  //These functions make sure the robot is powered off initially, then powers it on
  shutoff();
  restart();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));

}



void coreZeroTasks(void * pv) {
  for(;;) {
    senseCurrent();
    senseVolt();
    vTaskDelay(pdMS_TO_TICKS(100));

  }
  
}

void coreOneTasks(void * pv) {
for (;;) {
  senseAlarm();
  senseProbe();

  //Using mutex to safely check error states
  xSemaphoreTake(mutex,portMAX_DELAY);

    bool hasError = (current_error || temperature_error || alarm_error || voltage_error);
    bool currentStatus = robot_status;
    bool localCurr = current_error;
    bool localTemp = temperature_error;
    bool localAlarm = alarm_error;
    bool localVolt = voltage_error;

  xSemaphoreGive(mutex);

  logger.printLog();
  // The rover is running normally, and should now restart
  if (!currentStatus && !hasError) {
    // everything ok + allowed to restart
    Serial.println("ON");
    restart();
  }

  if (hasError) {
    shutoff();
  }

//Warning LEDs
  if (localCurr) {
    digitalWrite(current_Error_LED, HIGH);
    Serial.println(current_msg);
  } else {
    digitalWrite(current_Error_LED, LOW);
  }
  if (localTemp) {
    digitalWrite(temperature_Error_LED, HIGH);
    Serial.println(temperature_msg);
  } else digitalWrite(temperature_Error_LED, LOW);

  if (localAlarm) {
    digitalWrite(alarm_Error_LED, HIGH);
    Serial.println(alarm_msg);
  } else digitalWrite(alarm_Error_LED, LOW);

  if (localVolt) {
    digitalWrite(voltage_Error_LED, HIGH);
    Serial.println(undervoltage_msg);
  } else digitalWrite(voltage_Error_LED, LOW);

  // The rover is running normally, and is not set to restart
  if (currentStatus && !hasError) {
  // everything ok + motors not enabled
    Serial.println(ok_msg);
  }

  vTaskDelay(pdMS_TO_TICKS(500));

}

}


//Shuts down the robot
void shutoff() {
  digitalWrite(power_control, HIGH);
  digitalWrite(stop_all, LOW);
  digitalWrite(soft_start, LOW);
  Serial.println("OFF");
 //delay(3000);
  xSemaphoreTake(mutex,portMAX_DELAY);
  robot_status = 0;
  xSemaphoreGive(mutex);
}

// After shutting down, this function turns the bot back on
void restart() {
  softStart();
  digitalWrite(stop_all, HIGH);
  Serial.println(ok_msg);
}


//This function checks the current, ensures it doesn't rise too high
void senseCurrent() {
  for (int x = 0; x < 150; x++) {          //Get 150 samples
    AcsValue = analogRead(currentsensor);  //Read current sensor values
    Samples = Samples + AcsValue;          //Add samples together
    // delay(3);                              // let ADC settle before next sample 3ms
  }
  AvgAcs = Samples / 150.0;
  Samples = 0;
  AcsValueF = -1 * (1.65 - (AvgAcs * (3.3 / 4095.0))); //UPDATED to work with 3.3V instead of 5.
  //Taking Average of Samples
  //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than
  //you must change the offset according to the input voltage)
  //0.100v(100mV) is rise in output voltage when 1A current flows at input

  //publish and activate alarm signal if necessary
  AcsValueF = AcsValueF * 10;
  logger.logCurrent(AcsValueF);

  xSemaphoreTake(mutex,portMAX_DELAY);
  if (AcsValueF >= MAX_SAFE_CUR) {
    current_error = 1;
    //shutoff();
    Serial.println(current_msg);
  } else {
    current_error = 0;
  }
  xSemaphoreGive(mutex);
  return;
}

void senseProbe() {
  sensors.requestTemperatures();  // Send the command to get temperatures

  double sensAvg = 0;
  /*
  Serial.print("numdevices: ");
  Serial.println(numberOfDevices);
  */
  // Loop through each device, print out temperature data
  for (int i = 0; i < numberOfDevices; i++) {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i)) {
      float tempC = sensors.getTempC(tempDeviceAddress);
      sensAvg += tempC;
    }
  }

  sensAvg = sensAvg / (double)numberOfDevices;
  countArray[avgCount] = sensAvg;



  if (avgCount < avgMax) { avgCount++; }  //increment array index, or set to 0 if at end of erray
  else {
    avgCount = 0;
  }


  double avgTemp = getAvg(countArray, avgMax);  //average temperature of the last [avgMax]
  //Serial.println(avgTemp);
  xSemaphoreTake(mutex,portMAX_DELAY);
  if (avgTemp >= MAX_SAFE_TEMP) {
    if (!temperature_error) {
      temperature_error = 1;
    }
  } else {
    temperature_error = 0;
  }
  xSemaphoreGive(mutex);
  logger.logVoltage(avgTemp);

}

// This function checks the alarm state of the motors, in case the motors are having any issues
void senseAlarm() {
  int lRead = digitalRead(alarm_left);
  int rRead = digitalRead(alarm_right);
  Serial.print(lRead);
  Serial.print(" ");
  Serial.println(rRead);

  xSemaphoreTake(mutex,portMAX_DELAY);
  if (!lRead || !rRead) {
    Serial.println("ERROR");
    if (alarm_error == 0) {
      alarm_error = 1;
      //shutoff();
      Serial.println(alarm_msg);
    }
  } else {
    alarm_error = 0;
  }
  xSemaphoreGive(mutex);
}

// This function checks the voltage, ensures that it isn't undervolted
void senseVolt() {
  int raw_value = analogRead(volt_sensor);
  float voltage = (raw_value / (float)ADC_MAX) * VOLTAGE_REF;  // Convert to actual voltage

  logger.logVoltage(voltage);

  xSemaphoreTake(mutex,portMAX_DELAY);
  if (voltage < 0.5) {  // No voltage = rover off
    Serial.println("NoVolt");
    voltage_error = 1;
  } else if (voltage < 1.5) {  // Undervoltage (adjust based on your battery)
    Serial.println("UVolt");
    voltage_error = 1;
  } else {  // Voltage OK
    voltage_error = 0;
  }
  xSemaphoreGive(mutex);
}

// This function starts the entire system, it will wait for 1s before switching the relay to prevent any delays with capacitance
void softStart() {
  //enables power to the rover
  digitalWrite(power_control, LOW);
  delay(1000);
  Serial.println("ON");
  //reset the state of the leds
  digitalWrite(current_Error_LED, LOW);
  digitalWrite(voltage_Error_LED, LOW);
  digitalWrite(temperature_Error_LED, LOW);
  digitalWrite(alarm_Error_LED, LOW);

  digitalWrite(soft_start, HIGH);

  xSemaphoreTake(mutex,portMAX_DELAY);
  robot_status = 1;
  xSemaphoreGive(mutex);
}

// Takes average of an array, takes in array and the highest the array will be read to (generally, max == array length)
double getAvg(int array[], int max) {  
  double sum = 0;
  for (int i = 0; i < max; i++) {
    sum += (double)array[i];
  }
  return (sum / (double)max);
}


