/*
       #####
      #    ##
         ##
           ##
      ##   ##
       #####
*/

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

  if (!bmp.begin(BMP_ADDR)) isBMP280present = false; // BPM280 sensor was not found -- respond with empty data fields
  else isBMP280present = true;
}



// -----------------------------------------------------------------------------------------------------------
String readBMP280()
{ // C|мм.р.ст.|м
 if (isBMP280present)
 {
  return "Temp2|" +String(bmp.readTemperature()) +"|Pres2|" +bmp.readPressure()/133.3 +"|Alt2|" +bmp.readAltitude(1013.25);
 }
 else return "Temp2|-|Pres2|-|Alt2|-";
}



// -----------------------------------------------------------------------------------------------------------
void sender()
{
    uint32_t start_time = millis();
    uint8_t length;
    String Buff;

    while(true) // infinite sending loop
    {
        if (!rfm.canSend())
        {
            continue; // sending is not possible, already sending
        }

        if ((millis() - start_time) > 2000) // do this every 2000 ms
        {
            start_time = millis();
            //Serial.print("Send Packet ("); Serial.print(length); Serial.print("): "); Serial.println(*counter);
            Buff = readBMP280();
            length = Buff.length() +1;
            char buffer[length];
            Buff.toCharArray(buffer,length);
            rfm.sendVariable(&buffer,length);
            Serial.println(Buff); // show sent data
        }
    }
}



// -----------------------------------------------------------------------------------------------------------
void loop()
{
  sender(); // -----
}
