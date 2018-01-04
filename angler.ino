#define VERBOSE
#ifdef VERBOSE
  #define V_PRINT(str)   (Serial.print(str))
  #define V_PRINTLN(str) (Serial.println(str))
#endif

#define ANGLE_CHECK
#ifdef ANGLE_CHECK
  uint8_t angle_state = 0;
  const uint8_t _angle_query = B10000000;
  const uint8_t _angle_input = B00000001;
  const uint8_t _angle_len   = B00000010;

  #define ANG_IP_PRINT(str)   {if(angle_state&_angle_input){Serial.print(str);}}
  #define ANG_IP_PRINTLN(str) {if(angle_state&_angle_input){Serial.println(str);}}
  #define ANG_LEN_PRINT(str)   {if(angle_state&_angle_len){Serial.print(str);}}
  #define ANG_LEN_PRINTLN(str) {if(angle_state&_angle_len){Serial.println(str);}}

  
#endif

const int cos_pin = A2;
const int sin_pin = A3;

uint8_t state = 0;
const uint8_t _calibrating = B10000000;
const uint8_t _baud = B01000000;
const uint8_t _silent = B00100000;
const uint8_t _number_record = B00000001;

#include "ee_save.h"

//todo: should I store this too?
uint32_t msecs_calibrate = 100; // 16 gives only a minute at most
unsigned long prev_msec_check = 0; // At progrom start, this is 0
uint16_t cos_max = 0;
uint16_t cos_offset = 1023;
uint16_t cos_size = 0;
uint16_t sin_max = 0;
uint16_t sin_offset = 1023;
uint16_t sin_size = 0;
uint32_t baud_rate = 9600;

double get_angle(uint16_t input_cos,  uint16_t input_sin)
{
  int16_t len_cos = (((int16_t)input_cos - (int16_t)cos_offset) - (int16_t)cos_size / 2);
  ANG_LEN_PRINT("len_cos is ");ANG_LEN_PRINT(len_cos);ANG_LEN_PRINT("!");
  int16_t len_sin = (((int16_t)input_sin - (int16_t)sin_offset) - (int16_t)sin_size / 2);
  ANG_LEN_PRINT("\tlen_sin is ");ANG_LEN_PRINT(len_sin);ANG_LEN_PRINTLN("!");

  return atan2(len_sin, len_cos);
}

double radians_to_deg(double rad)
{
  return rad*180.0 / PI;
}

void calibration_step(uint16_t cos_input,   uint16_t sin_input)
{
  V_PRINTLN("Calibration step!");
  if (cos_input > cos_max){ //increase cos max
    cos_max = cos_input; //cos_max must be used, otherwise size might be set before offset is correct
  }
  if (cos_input < cos_offset){ //decrease cos_min
    cos_offset = cos_input;
  }
  if (sin_input > sin_max){ 
    sin_max = sin_input;
  }
  if (sin_input < sin_offset){ 
    sin_offset = sin_input;
  }
  if(millis() - prev_msec_check > msecs_calibrate){
    cos_size = cos_max - cos_offset;
    sin_size = sin_max - sin_offset;
    V_PRINTLN("Updating calibration data!");
    ee_write(msecs_calibrate,
             cos_max, cos_offset, cos_size,
             sin_max, sin_offset, sin_size,
             baud_rate);
    prev_msec_check = millis();
  }
  
}

//todo: optional degrees or radians
void setup() {
  //load stuff
  analogReference(EXTERNAL);
  startup_read(msecs_calibrate,
               cos_max, cos_offset, cos_size,
               sin_max, sin_offset, sin_size,
               baud_rate);
               
  Serial.begin(baud_rate);
  V_PRINTLN("Start!");  
}

void input_handler(char c, int& number_store){
  switch(c){
    #ifdef ANGLE_CHECK
    case 'a':
      angle_state |= _angle_query;
      state |= _number_record;
      break;
    #endif
    case 'c': //calibrate
      state |= _calibrating;
      V_PRINTLN("Calibrating!");
      break;
    case 'd': //done calibrating
      state &= ~_calibrating;
      V_PRINTLN("Done calibrating!");
      break;
    case 'b': //set baud
      state |= _baud;
      V_PRINTLN("Setting baud rate!");
      break;
    case 's':
      state ^= _silent;
      if (state&_silent){
        V_PRINTLN("Silencing degree output!");
      }
      else{
        V_PRINTLN("Resuming degree output!");
      }
      break;
    case 'r': //reset
      msecs_calibrate = 100; // 16 gives only a minute at most
      cos_max = 0;
      cos_offset = 255;
      cos_size = 0;
      sin_max = 0;
      sin_offset = 255;
      sin_size = 0;
      baud_rate = 9600;
      
      ee_write(msecs_calibrate,
             cos_max, cos_offset, cos_size,
             sin_max, sin_offset, sin_size,
             baud_rate);
      state |= _calibrating;
      V_PRINTLN("Erasing calibration data and calibrating!");
      break;
      
    case '0':case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9':
#ifdef ANGLE_CHECK
      if (angle_state&_angle_query){
        switch(c){
          case '0':
            angle_state ^= _angle_input;
            if (angle_state&_angle_input){
              V_PRINTLN("Displaying analog angle input!");
            }
            else{
              V_PRINTLN("Silencing analog angle input!");
            }
            break;
          case '1':
            angle_state ^=_angle_len;
            if (angle_state&_angle_len){
              V_PRINTLN("Displaying normalized sin/cos values!");
            }
            else{
              V_PRINTLN("Silencing normalized sin/cos values!");
            }
          default:
            break;
        }
        angle_state &= ~_angle_query;
        state &= ~_number_record;
      }
#endif
    
      if (state&_calibrating && !(state&_number_record)){
        msecs_calibrate = (c - '0');
        state |= _number_record;
      }
      else if (state&(_calibrating | _number_record)){
        msecs_calibrate = msecs_calibrate*10 + (c - '0');
      }
      if (state&_baud && !(state&_number_record)){
        number_store = (c-'0');
        state |= _number_record;
      }
      else if (state&(_baud | _number_record)){
        number_store = msecs_calibrate*10 + (c - '0');
      }
      break;
    case '\n':
      #ifdef VERBOSE
      if (state&(_calibrating | _number_record)){
        //V_PRINT("Milliseconds between calibration storage is now ");V_PRINT(msecs_calibrate);V_PRINTLN("!");
      }
      #endif
      state &= ~_number_record;
      if (state& _baud){
        state &= ~_baud;
        baud_rate = number_store;
        V_PRINT("Baud is now ");V_PRINT(baud_rate);V_PRINTLN("!");
        Serial.begin(baud_rate);
      }
      
      //V_PRINTLN("Ending command string!");
      
      break;
#ifdef VERBOSE
    case 'g':
      Serial.println("Stored Data:");
        Serial.print("\t cos max:");Serial.println(cos_max);
        Serial.print("\t cos offset:");Serial.println(cos_offset);
        Serial.print("\t cos size:");Serial.println(cos_size);
        Serial.print("\t sin max:");Serial.println(sin_max);
        Serial.print("\t sin offset:");Serial.println(sin_offset);
        Serial.print("\t sin size:");Serial.println(sin_size);
        Serial.print("\t calibration record interval:");Serial.println(msecs_calibrate);
      break;
#endif
      
    default:
      break;
    
  }
}

void loop() {
  uint16_t cos_input = analogRead(cos_pin);
  ANG_IP_PRINT("Cos analog in is ");ANG_IP_PRINT(cos_input);ANG_IP_PRINT("!");
  delay(50);
  uint16_t sin_input = analogRead(sin_pin);
  ANG_IP_PRINT("\tSin analog in is ");ANG_IP_PRINT(sin_input);ANG_IP_PRINTLN("!");

  char in_byte = -1;
  int number_store= 0;

  if (Serial.available()>0){
    in_byte = Serial.read();
    input_handler(in_byte, number_store);
  }

  if (state&_calibrating){
    calibration_step(cos_input, sin_input);
  }

  double angle = get_angle(cos_input, sin_input);
  //V_PRINT("State is ");V_PRINT(state);V_PRINTLN("!");
  if (!(state&_silent)){
    Serial.println(radians_to_deg(angle));
  }
  

  

 // Serial.println(get_angle)
  // put your main code here, to run repeatedly:

}
