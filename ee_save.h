#ifndef __EE_SAVE
#define __EE_SAVE

#include <avr/eeprom.h>
#include <util/crc16.h>
#include <Arduino.h>
#include <EEPROM.h>

const uint16_t n_bytes = 20;

// based on http://www.microchip.com/stellent/groups/SiteComm_sg/documents/Training_Tutorials/en532276.pdf 
//  giving p of failure in 10 years = [5/(1 million*(15/10))]= 0.00000333333,
//  binomial distributions such that the population of Germany having 0 eeprom failures is 99.95%
//  http://www.wolframalpha.com/input/?i=P%5BX+%3E%3D+2%5D+for+X~B(2,0.00000333333) giving the failure probabilty of 2 sections at once
//  the probability of two sections failing at once in 10 for anyone in the entire population of Germany is much lower than 0.05%
//  (.0000000004/(1.11111×10^-11) = 36. So 36 times safer than required.)

// starting up on invalid data is 1/256 though, so writing startup data in factory is recommended

const uint8_t n_sections = 2; 

inline uint16_t section_length(){
  return EEPROM.length()/n_sections;
}

int check_crc(const uint8_t* data, const uint8_t& data_len/*, uint8_t crc*/){
  uint8_t crc = 0;

  for(uint8_t i=0; i<data_len; ++i){
    crc = _crc_ibutton_update(crc, data[i]);
  }
  return crc;
}

//todo:check if state is needed
bool startup_read(uint32_t& msecs_calibrate,
                  uint16_t& cos_max, uint16_t& cos_offset, uint16_t& cos_size,
                  uint16_t& sin_max, uint16_t& sin_offset, uint16_t& sin_size,
                  uint32_t& baud_rate)
{
  uint8_t buf[n_bytes+1];
  for(uint8_t i=0; i<n_sections;++i){
    eeprom_busy_wait();
    eeprom_read_block(buf, (const void*)(section_length()*i), n_bytes+1);
    V_PRINT("EEPROM block ");V_PRINT((section_length()*i));V_PRINTLN(" read!");
    if(check_crc(buf, n_bytes+1)==0){//section has correct crc
      break;
    }
    else{
      V_PRINT("CRC check failed on section");V_PRINT(i);V_PRINTLN("!");
      if(i>=n_sections-1){//no correct sections
        V_PRINT("All CRC checks failed! Running with factory default settings!");
        state|=_calibrating;//start calibrating
        //default values are already set in program
        return false;
      }
    }
  }
  msecs_calibrate = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
  cos_max = buf[4]<<8 | buf[5];
  cos_offset = buf[6]<<8 | buf[7];
  cos_size = buf[8]<<8 | buf[9];
  sin_max = buf[10]<<8 | buf[11];
  sin_offset = buf[12]<<8 | buf[13];
  sin_size = buf[14]<<8 | buf[15];
  baud_rate = (buf[16]<<24) | (buf[17]<<16) | (buf[18]<<8) | buf[19]; 
  return true;
}

void ee_write(const uint32_t& msecs_calibrate,
              const uint16_t& cos_max, const uint16_t& cos_offset, const uint16_t& cos_size,
              const uint16_t& sin_max, const uint16_t& sin_offset, const uint16_t& sin_size,
              const uint32_t& baud_rate)
{
  uint8_t buf[n_bytes+1];
  buf[0] = msecs_calibrate>>24; buf[1] = (msecs_calibrate>>16) & 0xFF; buf[2] = (msecs_calibrate>>8) & 0xFF; buf[3] = (msecs_calibrate) & 0xFF;
  buf[4] = cos_max>>8; buf[5] = cos_max & 0xFF;
  buf[6] = cos_offset>>8; buf[7] = cos_offset & 0xFF;
  buf[8] = cos_size>>8; buf[9] = cos_size & 0xFF;
  buf[10] = sin_max>>8; buf[11] = sin_max & 0xFF;
  buf[12] = sin_offset>>8;   buf[13] = sin_offset & 0xFF;
  buf[14] = sin_size>>8; buf[15] = sin_size & 0xFF;
  buf[16] = baud_rate>>24; buf[17] = (baud_rate>>16) & 0xFF; buf[18] = (baud_rate>>8) & 0xFF; buf[19] = (baud_rate) & 0xFF;

  uint8_t crc_code = check_crc(buf, n_bytes);
  buf[n_bytes] = crc_code;
  V_PRINT("CRC code is ");V_PRINT(crc_code);V_PRINTLN("!");

  for(uint8_t i=0; i<n_sections; ++i){
    eeprom_busy_wait();
    eeprom_update_block(buf, (const void*)(section_length()*i), n_bytes+1);
    V_PRINT("Wrote settings to section ");V_PRINT(i);V_PRINT(" at block ");V_PRINT((section_length()*i));V_PRINTLN("!");
  }
  
}


#endif
