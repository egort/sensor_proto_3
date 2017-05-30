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
#include "plainRFM69.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h"

//#define DHTPIN 3 // DHT11 sensor pin
#define SLAVE_SELECT_PIN 10 // SS/NSS line on SPI bus
#define SENDER_DETECT_PIN A0 // tie this pin down on the receiver.
#define RESET_PIN 8 // connected to the reset pin of the RFM69
#define DIO2_PIN 2 // Pin DIO 2 on the RFM69 is attached to this digital pin which should have interrupt capability (2 or 3)
#define INTERRUPT_NUMBER 0 // on Pro Mini INT0 at pin 2, INT1 at 3.

#define Addr 0x76 /// BMP280 I2C address is 0x76(108)
#define P0 1013.25

Adafruit_BMP280 bmp; // I2C
bool isBMP280present = false;
plainRFM69 rfm = plainRFM69(SLAVE_SELECT_PIN); // SPI

void interrupt_RFM()
{
    rfm.poll(); // in the interrupt, call the poll function.
}

void setup() {
  Wire.begin(); // Initialise I2C communication as MASTER
  SPI.begin();
  Serial.begin(19200);
  if (!bmp.begin(0x76))
    isBMP280present = false; // BPM280 sensor was not found -- respond with empty data fields
  else
    isBMP280present = true;
  // <!-- RFM69 INIT
  bareRFM69::reset(RESET_PIN); // sent the RFM69 a hard-reset.
  rfm.setRecommended(); // set recommended paramters in RFM69.
  rfm.setPacketType(false, false); // set the used packet type.
  rfm.setBufferSize(2);   // set the internal buffer size.
  rfm.setPacketLength(64); // set the packet length.
  rfm.setFrequency((uint32_t) 915*1000*1000); // set the frequency.
  // baudrate is default, 4800 bps now.
  rfm.receive(); // set it to receiving mode.
  rfm.setDioMapping1(RFM69_PACKET_DIO_2_AUTOMODE); // tell the RFM to represent whether we are in automode on DIO 2.
  pinMode(DIO2_PIN, INPUT); // set pinmode to input.
  SPI.usingInterrupt(INTERRUPT_NUMBER); // Tell the SPI library we're going to use the SPI bus from an interrupt.
  attachInterrupt(INTERRUPT_NUMBER, interrupt_RFM, CHANGE); // hook our interrupt function to any edge.
  rfm.receive(); // start receiving
  pinMode(SENDER_DETECT_PIN, INPUT_PULLUP);
  // RFM69 INIT -->
  delay(5);
}



// -----------------------------------------------------------------------------------------------------------
String readBMP280()
{ // C|мм.р.ст.|м
 if (isBMP280present)
 {
  return "Temp2|" +String(bmp.readTemperature()) +"|Pres2|" +bmp.readPressure()/133.3 +"|Alt2|" +bmp.readAltitude(1013.25);
 }
 else
  return "Temp2||Pres2||Alt2|";
}



// -----------------------------------------------------------------------------------------------------------
void sender()
{
    uint32_t start_time = millis();
    //String Buff;
    //uint8_t length = 1;

    while(true){
        if (!rfm.canSend()){
            continue; // sending is not possible, already sending.
        }

        if ((millis() - start_time) > 2000){ // every 500 ms.
            start_time = millis();

            // be a little bit verbose.
            //Serial.print("Send Packet ("); Serial.print(length); Serial.print("): "); Serial.println(*counter);

            // send the number of bytes equal to that set with setPacketLength.
            // read those bytes from memory where counter starts.
            String Buff = readBMP280();
            uint8_t length = Buff.length() +1;
            char buffer[length];
            Buff.toCharArray(buffer,length);
            rfm.send(&buffer);
            Serial.println(Buff);
            //rfm.sendVariable(&tx_buffer, length);

            // length = (length + 1) % length_overflow;
            // if (length == 0){
            //     length++;
            // }


            //(*counter)++; // increase the counter.
        }

    }
}



// -----------------------------------------------------------------------------------------------------------
void loop()
{
  sender(); // -----
}
