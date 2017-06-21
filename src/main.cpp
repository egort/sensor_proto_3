/*
       #####
      #    ##
         ##
           ##
      ##   ##
       #####
*/
//#include <stdlib.h>
#include <Wire.h>
#include <SPI.h>
#include "plainRFM69.h"      // RFM69 radio module 915MHz
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h" // temp & atm. pressure sensor

#define SLAVE_SELECT_PIN 10 // SS/NSS line on SPI bus (RFM69)
#define SENDER_DETECT_PIN A0 // tie this pin down on the receiver (RFM69)
#define RESET_PIN 8 // connected to the reset pin of the RFM69
#define DIO2_PIN 2 // Pin DIO 2 on the RFM69 is attached to this digital pin which should have interrupt capability (2 or 3)
#define INTERRUPT_NUMBER 0 // on Pro Mini INT0 at pin 2, INT1 at 3.

#define BMP_ADDR 0x76 /// BMP280 address is 0x76 for I2C bus mode
#define P0 1013.25 /// BMP280 constant (conversion pressure to altitude)
#define SERIAL_SPEED 19200

Adafruit_BMP280 bmp; // I2C bus mode
plainRFM69 rfm = plainRFM69(SLAVE_SELECT_PIN); // SPI bus mode

bool isBMP280present = false; // sensor availability
char send_buff[64]; // rfm sending data buffer

// Temp2|" +String(bmp.readTemperature()) +"|Pres2|" +bmp.readPressure()/133.3 +"|Alt2|" +bmp.readAltitude(1013.25);

void interrupt_RFM()
{
    rfm.poll(); // in the interrupt, call the poll function.
}

void setup() {
  Wire.begin(); // Initialise I2C communication as MASTER
  SPI.begin();
  Serial.begin(SERIAL_SPEED);

  bareRFM69::reset(RESET_PIN); // sent the RFM69 a hard-reset
  // !!! Order of calling rfm methods is important !!!
  rfm.setRecommended(); // set recommended paramters in RFM69
  // rfm.setAES(false); // should come before setPacketType
  rfm.setPacketType(true/*variable packet length*/, false/*no filtering on address*/);
  rfm.setBufferSize(2);   // set the internal buffer size (at least 2)
  rfm.setPacketLength(64); // set the packet length (64 is maximum) for fixed length packet mode
  // rfm.setNodeAddress(0x01);
  rfm.setFrequency((uint32_t) 915*1000*1000); // set the frequency
  rfm.baud4800(); // baudrate is default, 4800 bps
  rfm.receive(); // set it to receiving mode
  rfm.setDioMapping1(RFM69_PACKET_DIO_2_AUTOMODE); // tell the RFM to represent whether we are in automode on DIO 2.
  pinMode(DIO2_PIN, INPUT); // set pinmode to input.
  SPI.usingInterrupt(INTERRUPT_NUMBER); // Tell the SPI library we're going to use the SPI bus from an interrupt.
  attachInterrupt(INTERRUPT_NUMBER, interrupt_RFM, CHANGE); // hook our interrupt function to any edge.
  rfm.receive(); // start receiving
  pinMode(SENDER_DETECT_PIN, INPUT_PULLUP);
  delay(5);

  if (!bmp.begin(BMP_ADDR))
    isBMP280present = false; // BPM280 sensor was not found -- respond with empty data fields
  else
    isBMP280present = true;
}


// -----------------------------------------------------------------------------------------------------------
// parsing and wrapping sending data in global send_buff[64] at requested position
// return position of the first '\0' in send_buff[], or 0 when no enough space for requested data placement
// prefix like "XXXnn" & trail "|"
// data in -9999.9 to 9999.9
// pos in 0 to 63: position in send_buff[] to insert prefix and data
uint8_t wrap_data_to_send_buff (const char *prefix, float data, uint8_t pos)
{
  uint8_t delim_len = 1;                                  // delimiter string (i.e. "|") length
  uint8_t data_len;                                       // TODO: try to use negative for left alignment in dtostr
  uint8_t prefix_len = strlen(prefix);                    // prefix length (WITHOUT \0 and delimiter)
  char *ptr_prefix_in_buff = &send_buff[pos];             // sensor prefix (const char[]) position pointer
  char *ptr_data_in_buff = &send_buff[pos + prefix_len + delim_len];  // sensor data (float) position pointer
  uint8_t send_buff_size = sizeof(send_buff);             // send_buff[] full size

  Serial.print("sizeof(send_buff)="); Serial.println(send_buff_size);
  Serial.print("strlen(send_buff)="); Serial.println(strlen(send_buff));
  Serial.print("pos="); Serial.println(pos);
  Serial.print("prefix="); Serial.print(prefix); Serial.print(", prefix_len="); Serial.println(prefix_len);

  // buffer overflow checking
  if (pos + prefix_len + delim_len < send_buff_size )
  {
    strcpy(ptr_prefix_in_buff, prefix);             // add prefix
    strcat(ptr_prefix_in_buff, "|");                // append delimiter
  }
  else
  {
    Serial.println("ERR:no space for prefix, return 0.") ;
    return 0;  // not enough space for prefix in send_buff[]
  }

  // --- calculate length (float) data in chars
  if (data == 0) data_len = 1;                    // 0
  if (data > 0 && data < 10 ) data_len = 3;       // 0.1 ... 9.9
  if (data >= 10 && data < 100) data_len =4;      // 10.0 ... 99.9
  if (data >= 100 && data < 1000) data_len=5;     // 100.0 ... 999.9
  if (data >= 1000 && data < 10000) data_len=6;   // 1000 ... 9999.9
  if (data > -10 && data < 0) data_len=4;         // -9.9 ... -0.1
  if (data > -100 && data <= -10) data_len=5;     // -99.9 ... -10.0
  if (data > -1000 && data <= -100) data_len=6;   // -999.9 ... -100.0
  if (data > -10000 && data <= -1000) data_len=7; // -9999.9 ... -1000.0
  if (data >= 10000) { data = -9999.9; data_len=7; } // invert out of range values, to looks like owerflow
  if (data <= -10000) { data = 9999.9; data_len=6; } // --"--

  Serial.print("data="); Serial.print(data, 1); Serial.print(", data_len="); Serial.println(data_len);

  // TODO:implement delim_len also for prefix (to use "TMP01" instead of "TMP01|")

  if (pos + prefix_len + delim_len + data_len + delim_len < send_buff_size)                // send_buff[] range checking
  {
    ptr_data_in_buff = dtostrf(data, data_len, 1, ptr_data_in_buff);                 // put sensor data (float) data_len-1
    strcat(ptr_data_in_buff, "|");
    //strcpy(&send_buff[pos +prefix_len +data_len], "|");         // put trail delimiter
  }
  else // not enough space in send_buff[]
  {
    Serial.println("ERR:no space for data, return 0.") ;
    return 0;
  }
  // ---

  // --- show send_buff content [debug]
  char *ptr0 = &send_buff[0];
  for (int i = 0; i < 64; i++)
  {
      if (ptr0[i] < 32 || ptr0[i] > 126) // print code
      {
          Serial.print("\\");
          Serial.print((int8_t)ptr0[i]);
          Serial.print(",");
      }
      else
      {
          Serial.print(ptr0[i]); // print ascii char
          Serial.print(",");
      }
  } Serial.println("");
  Serial.print("return next pos="); Serial.print(pos + prefix_len + delim_len + data_len + delim_len); Serial.println(" (from 0)");
  Serial.println("==============================================");
  // ---

  return pos + prefix_len + delim_len + data_len + delim_len; // TODO: last char always EOL ('\0')
}


// -----------------------------------------------------------------------------------------------------------
void sender()
{
    uint32_t start_time = millis();

    while(true) // infinite sending loop
    {
        if (!rfm.canSend())
        {
            continue; // sending is not possible, already sending
        }

        if ((millis() - start_time) > 5000) // do this every 2000 ms
        {
            start_time = millis();
            uint8_t pos; // offset from the send_buff[] beginning

            pos = wrap_data_to_send_buff("TMP01", -1, 0); // first portion (temperature)
            pos = wrap_data_to_send_buff("TMP02", -10, pos); // first portion (temperature)
            pos = wrap_data_to_send_buff("TMP03", -100, pos); // first portion (temperature)
            pos = wrap_data_to_send_buff("TMP04", -1000, pos); // first portion (temperature)
            pos = wrap_data_to_send_buff("TMP44", -999, pos); // first portion (temperature)
            pos = wrap_data_to_send_buff("TMP92", 99.9, pos); // first portion (temperature)

            // len = wrap_data_to_send_buff("TMP03|", bmp.readTemperature(), 0); // first portion (temperature)
            // len = wrap_data_to_send_buff("ALT03|", bmp.readAltitude(1013.25), len); // next, altitude in meters
            // len = wrap_data_to_send_buff("PRS03|", bmp.readPressure()/133.3, len); // last, pressure in mm Hg
            rfm.sendVariable(send_buff, pos);
            Serial.print("sent buff=[");Serial.print(send_buff);Serial.println("]"); // show sent data
        }
    }
}



// -----------------------------------------------------------------------------------------------------------
void loop()
{
  sender(); // -----
}
