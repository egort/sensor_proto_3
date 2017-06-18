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
int ins_data (const char *prefix, float data, uint8_t pos)
{
  // char send_buff[64] is GLOBAL
  // TODO: implement BUFFER OVERFLOW checking
  uint8_t data_len; // TODO: try to use negative for left alignment in dtostr
  int8_t prefix_len = strlen(prefix);             // prefix length
  char *ptr_prefix = &send_buff[pos];             // sensor prefix (const char[]) position pointer
  char *ptr_data = &send_buff[pos + prefix_len];    // sensor data (float) position pointer

  strcpy(ptr_prefix, prefix);                     // put sensor prefix

  Serial.print("prefix=");
  Serial.print(prefix);
  Serial.print(", prefix_len=");
  Serial.print(prefix_len);
  Serial.print(", data=");
  Serial.println(data);

  // --- calculate length of sensor data part (float + delimiter), i.e. "0.1|" = 4
  if (data == 0) data_len = 2;
  if (data > 0 && data < 10 ) data_len = 4;
  if (data >= 10 && data < 100) data_len =5;
  if (data >= 100 && data < 1000) data_len=6;
  if (data >= 1000 && data < 10000) data_len=7;
  if (data > -10 && data < 0) data_len=5;
  if (data > -100 && data <= -10) data_len=6;
  if (data > -1000 && data <= -100) data_len=7;
  if (data > -10000 && data <= -1000) data_len=8;
  dtostrf(data, data_len-1, 1, ptr_data);                 // put sensor data
  strcpy(&send_buff[pos + prefix_len + data_len-1], "|"); // put trail delimiter
  // ---

  // --- show send_buff content [debug]
  Serial.print("pos="); Serial.println(pos + prefix_len + data_len);
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
  }
  Serial.println("");
  Serial.print("return len="); Serial.println(prefix_len + data_len);
  Serial.println("");
  // ---

  return pos + prefix_len + data_len; // TODO: last char always EOL ('\0')
}


// -----------------------------------------------------------------------------------------------------------
void sender()
{
    uint32_t start_time = millis();

    //uint8_t length;
    //String Buff;

    while(true) // infinite sending loop
    {
        if (!rfm.canSend())
        {
            continue; // sending is not possible, already sending
        }

        if ((millis() - start_time) > 5000) // do this every 2000 ms
        {
            start_time = millis();
            uint8_t len;

            len = ins_data("TMP01|", -1, 0); // first portion (temperature)
            len = ins_data("TMP02|", -10, 0); // first portion (temperature)
            len = ins_data("TMP03|", -100, 0); // first portion (temperature)
            len = ins_data("TMP04|", -1000, 0); // first portion (temperature)
            len = ins_data("TMP44|", -9999, 0); // first portion (temperature)
            len = ins_data("TMP92|", 99.9, 0); // first portion (temperature)

            // len = ins_data("TMP03|", bmp.readTemperature(), 0); // first portion (temperature)
            // len = ins_data("ALT03|", bmp.readAltitude(1013.25), len); // next, altitude in meters
            // len = ins_data("PRS03|", bmp.readPressure()/133.3, len); // last, pressure in mm Hg
            rfm.sendVariable(send_buff, len);
            Serial.print("sent buff=[");Serial.print(send_buff);Serial.println("]"); // show sent data
        }
    }
}



// -----------------------------------------------------------------------------------------------------------
void loop()
{
  sender(); // -----
}
