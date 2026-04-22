const int HALL_PIN_L = 2;
const int HALL_PIN_R = 3;

// Simplified variable types
unsigned long pulse_count_l = 0;
unsigned long pulse_count_r = 0;

unsigned long last_print_time = 0;

bool prev_state_l = false;
bool prev_state_r = false;

void setup(){
  // Using INPUT_PULLUP forces the pins to a stable HIGH state 
  // when the sensor isn't triggering, preventing noise.
  pinMode(HALL_PIN_L, INPUT_PULLUP);
  pinMode(HALL_PIN_R, INPUT_PULLUP);
  
  Serial.begin(115200);
}

void loop(){
  bool curr_state_l = digitalRead(HALL_PIN_L);
  if(curr_state_l && !prev_state_l) pulse_count_l++;
  prev_state_l = curr_state_l;

  bool curr_state_r = digitalRead(HALL_PIN_R);
  if(curr_state_r && !prev_state_r) pulse_count_r++;
  prev_state_r = curr_state_r;

  if(millis() - last_print_time > 30){
    Serial.print("<");
    Serial.print(pulse_count_l);
    Serial.print(",");
    Serial.print(pulse_count_r);
    Serial.println(">");
    
    pulse_count_l = 0;
    pulse_count_r = 0;
    last_print_time = millis();
  }
}