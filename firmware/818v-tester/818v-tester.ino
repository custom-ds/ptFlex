/* 

This is a similified version of the 818V code to test the transmit functionalityon the ptSolar PCB.

*/




#include "Modem.h"
#include "ptTracker.h"


#define PIN_PTT_OUT 2     //PD2
#define PIN_AUDIO_OUT 3   //PD3   - APRS Packet Audio
#define PIN_DRA_EN 4      //PD4
#define PIN_AUDIO 5       //PD5   - Audio Annunciation
#define PIN_GPS_EN 6      //PD6
#define PIN_GPS_TX 7      //PD7

#define PIN_GPS_RX 8      //PB0
#define PIN_DRA_TX 9      //PB1
#define PIN_DRA_RX 10     //PB2
//PB3 is MOSI
//PB4 is MISO
#define PIN_LED 13        //PB5

//Analog Pins
#define PIN_AUDIO_IN A0   //PC0
#define PIN_ANALOG_BATTERY A1   //PC1


#define DELAY_BETWEEN 400

ptTracker Tracker(PIN_LED, PIN_AUDIO, PIN_ANALOG_BATTERY, 2);    //Object that manages the board-specific functions
Modem Aprs(PIN_DRA_EN, PIN_PTT_OUT, PIN_AUDIO_OUT, PIN_DRA_TX, PIN_DRA_RX);;            //Object that assembles the packets for the TNC and transmits them


/**
 * @brief  The function that runs first before the main loop() function is called indefinitely.
 * @note   This function is called once at startup and is used to initialize the board and set up the hardware.
 */
void setup() {
  Serial.begin(19200);

  Aprs.setDebugLevel(2);

  //Configure the Misc Pins
  pinMode(PIN_LED, OUTPUT);
  //pinMode(PIN_AUDIO, OUTPUT);
  
  Serial.begin(19200);
  Serial.println("ptSolar Test Fixture for troubleshooting SA818V Transmit Module");
}


/**
 * @brief  Main loop for the program.  This is where the main logic of the program is executed.
 * @note   This function will run continuously until the board is powered off or reset.
 */
void loop() {

  char Callsign[7];    //6 digit callsign + Null
  strcpy(Callsign, "N0CALL");

  char CallsignSSID = '0';

  char Destination[7];
  strcpy(Destination, "APRS  ");

  char DestinationSSID = '0';

  char Path1[7];
  strcpy(Path1, "WIDE2 ");

  char Path1SSID = '1';

  char Path2[7];
  strcpy(Path2, "      ");

  char Path2SSID = '0';

  Serial.println("Transmitting an APRS packets");

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 10000 ");
  Aprs.packetAppend((long)10000, true);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 20000 ");
  Aprs.packetAppend((long)20000, false);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 30000 ");
  Aprs.packetAppend((long)30000, true);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 40000 ");
  Aprs.packetAppend((long)40000, false);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 60000 ");
  Aprs.packetAppend((long)60000, true);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 70000 ");
  Aprs.packetAppend((long)70000, false);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 90000 ");
  Aprs.packetAppend((long)90000, true);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 100000 ");
  Aprs.packetAppend((long)100000, false);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
  Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
  Aprs.packetAppend((char *)">Testing Long Xmit 140000 ");
  Aprs.packetAppend((long)140000, true);
  Aprs.packetSend();
  delay(DELAY_BETWEEN);   //wait X seconds and repeat


  for (int i=1;i<=10;i++) {

    Aprs.setCourtesyTone(true);    //Set the courtesy tone flag
    Aprs.packetHeader(Destination, DestinationSSID, Callsign, CallsignSSID, Path1, Path1SSID, Path2, Path2SSID, true);
    Aprs.packetAppend((char *)">Project Traveler ptSolar Flight Computer ");
    Aprs.packetAppend((long)i, false);

    Tracker.readBatteryVoltage(true);  //read the battery voltage before the transmission
    Aprs.packetSend();
    //Normally seeing about 280mV of drop during the transmission with a 0.5F supercap
    Tracker.readBatteryVoltage(true);  //read the battery voltage after the transmission

    delay(DELAY_BETWEEN);

  }
  Serial.println("Done with the packets.");
  delay(DELAY_BETWEEN);   //wait X seconds and repeat

  Aprs.sendTestDiagnotics();   //Send a test tone to the transmitter.  This function is used to test the transmitter and the audio path.
  delay(DELAY_BETWEEN);   //wait X seconds and repeat
}


//------------------------------------------ Functions and Timers  for the internal modulation ------------------------------------------
/**
 * @brief ISR - This is the interrupt service routine for the timer.  It is used to generate the audio tones for the VHF transmitter using the AX.25 protocol.
 */
ISR(TIMER1_COMPA_vect) {
  static uint8_t iStuffZero = 0;
  static bool bStuffBit = false;
  static uint16_t iRateGen = 1;
  static uint16_t iTonePhase = 0;      //two byte variable.  The highByte contains the element in arySine that should be output'ed
  static uint8_t toneHigh = 0;

  // digitalWrite(PIN_LED, HIGH);   //Uncomment for troubleshooting ISR Timing


  //increment the phase counter.  It will overflow automatically at > 65535
  switch (toneHigh) {
    case 0:
      iTonePhase += Modem::TONE_LOW_STEPS_PER_TICK; 
      break;
    case 1:
      iTonePhase += Modem::TONE_HIGH_STEPS_PER_TICK;
      break;
    case 2:
      iTonePhase += Modem::TONE_COURTESY_STEPS_PER_TICK;    //courtesy chirp
      break;
    default:
      iTonePhase = Modem::TONE_LOW_STEPS_PER_TICK; 
      break;
  }
  
  OCR2B = Aprs.getDACValue(highByte(iTonePhase));   //analogWrite to PIN PD3, directly using the OCR register

  iRateGen--;
  if (iRateGen == 0) {
    //it's time for the next bit

    if (bStuffBit) {
      //we hit the stuffing counter  - we don't need to get the next bit yet, just change the tone and send one bit
      toneHigh = !toneHigh;
      iStuffZero = 0;

      bStuffBit = false;    //reset this so we don't keep stuffing
      iRateGen = Modem::BAUD_GENERATOR_COUNT;  
    } else {
      //this is just a normal bit - grab the next bit from the szString

      switch(Aprs.getNextBit()) {
        case 0:
          //we only flip the output state if we have a zero
          toneHigh = !toneHigh;   //Flip Bit
          iStuffZero = 0;
          iRateGen = Modem::BAUD_GENERATOR_COUNT;  
          break;     
        case 1:
          //it's a 1, so send the same tone...
          iStuffZero++;      //increament the stuffing counter

          //if there's been 5 in a row, then we need to stuff an extra bit in, and change the tone for that one
          if (iStuffZero == 5 && !Aprs.noBitStuffing()) {
            bStuffBit = true;      //once we hit five, we let this fifth (sixth?) one go, then we set a flag to flip the tone and send a bogus extra bit next time
          }
          iRateGen = Modem::BAUD_GENERATOR_COUNT;  
          break;
        case 2:
          //this is a special case - we need to send a courtesy tone, so set the flag and break out of the loop
          toneHigh = 2;
          bStuffBit = false;
          iRateGen = Modem::COURTESY_TONE_COUNT;  
          break;
      } 
    }
    
  }
  //digitalWrite(PIN_LED, LOW);   //Uncomment for troubleshooting ISR Timing
}


