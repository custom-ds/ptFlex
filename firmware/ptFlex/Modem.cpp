/*
APRS Data Modem for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

Version History:
Version 2.0.2 - July 20, 2025 - Synchronized the ptFlex and ptSolar code bases to be parameterized by the TRACKER_PTFLEX and TRACKER_PTSOLAR defines.
Version 2.0.1 - July 12, 2025 - Fixed bug that was corrupting the first packet.
Version 2.0.0 - March 9, 2025 - Major refactoring to make use of the DRA/SA818V transmitter module. Based on the prior TNC module from the ptFlex-series of trackers.

*/

#include "Modem.h"
#include "Arduino.h"
#include <SoftwareSerial.h>

/**
 * @brief Initializes the APRS object with the pins for the transmitter and serial port.
 * @param pinEnable The pin to enable the transmitter.
 * @param pinPTT The pin to key up the transmitter.
 * @param pinTxAudio The pin to send audio to the transmitter.
 * @param pinSerialTx The pin to transmit serial data to the 818V module.
 * @param pinSerialRx The pin to receive serial data from the 818V module.
 */
Modem::Modem(uint8_t pinEnable, uint8_t pinPTT, uint8_t pinTxAudio, uint8_t pinSerialTx, uint8_t pinSerialRx) {
  this->_pinEnable = pinEnable;
  this->_pinPTT = pinPTT;
  this->_pinTxAudio = pinTxAudio;
  this->_pinSerialTx = pinSerialTx;
  this->_pinSerialRx = pinSerialRx;

  pinMode(this->_pinEnable, OUTPUT);   //Always configure the enable pin as an output
  pinMode(this->_pinPTT, OUTPUT);     //Always configure PTT as an output
  pinMode(this->_pinTxAudio, INPUT);    //Configure as an input until we need it.

  //Disable the transmitter until we need it.
  digitalWrite(this->_pinPTT, LOW);    //Stop the transmit - 
  digitalWrite(this->_pinEnable, LOW);    //disable the transmitter


  this->_txDelay = 30;    //default to 30 if not otherwise defined.
  this->_lastTransmitMillis = 0;    //initialize the last transmit time to zero

  _iSZLen = -1;      //reset back to a clean buffer
  this->PTT(false);   //make sure the transmitter is unkeyed
}


/**
 * @brief  Keys up the 818V transmitter. Passes through the Tx/Rx frequency during the process.
 * @param  tx: A boolean indicating whether or not to key up the transmitter or unkey into receive mode.
 * @notes  The configuration of the module should be done each time the chip is taken out of PowerDown mode.
 */
void Modem::PTT(bool tx) {

  char response;
  long start;

  wdt_reset();    //reset the watchdog timer

  if (tx) {
    this->configTimers();   //always make sure the ISR and PWM timers are configured prior to transmitting

    //Configure the analog output for the audio
    pinMode(this->_pinTxAudio, OUTPUT);
    analogWrite(this->_pinTxAudio, 128);   //Set the audio output to 128. Subsequent calls in the ISR will write directly to the OCR2B register.

    //Turn on the transmitter
    digitalWrite(this->_pinEnable, HIGH);
    
    //Configure the serial port
    SoftwareSerial* XMIT = nullptr;
	  XMIT = new SoftwareSerial(this->_pinSerialRx, this->_pinSerialTx, false);
	  XMIT->begin(9600);
    delay(100);
    wdt_reset();    //reset the watchdog timer

    //Connect to the radio chip
    if (this->_debugLevel >0) Serial.println(F("Conn"));
    XMIT->print(F("AT+DMOCONNECT\r\n"));
    
    //Get the response:
    start = millis();
    do {
      if (XMIT->available()) {
        response = XMIT->read();
        if (this->_debugLevel == 2) Serial.print(response);     //debug the output from the response
      }
    } while (response != '0' && (millis() - start) < MAX_WAIT_TIMEOUT);
    if (this->_debugLevel ==2) Serial.println("");
    if (this->_debugLevel >0) Serial.println(F("End Resp"));
    delay(100);

    //Configure the transceiver
    if (this->_debugLevel >0) {
      Serial.print(F("Set Freq: "));
      Serial.println(this->_szTxFreq);
    }

    XMIT->print(F("AT+DMOSETGROUP=0,"));
    XMIT->print(this->_szTxFreq);
    XMIT->print(",");
    XMIT->print(this->_szRxFreq);
    XMIT->print(F(",0000,4,0000\r\n"));   //No CTCSS Tx, Sql 4, No CTCSS Rx

    wdt_reset();    //reset the watchdog timer

    //Get the response:
    start = millis();
    do {
      if (XMIT->available()) {
        response = XMIT->read();
        if (this->_debugLevel == 2) Serial.print(response);     //debug the output from the response
      }
    } while (response != '0' && (millis() - start) < MAX_WAIT_TIMEOUT);  
    
    wdt_reset();    //reset the watchdog timer

    if (this->_debugLevel ==2) Serial.println("");
    if (this->_debugLevel >0) Serial.println(F("End Resp"));
    //delay(100);

    if (this->_debugLevel >0) Serial.println("Filter:");
    XMIT->print(F("AT+SETFILTER=1,1,1\r\n"));   //Set the tx/rx filters to 1,1,1

    //Get the response:
    start = millis();
    do {
      if (XMIT->available()) {
        response = XMIT->read();
        if (this->_debugLevel == 2) Serial.print(response);     //debug the output from the response
      }
    } while (response != '0' && (millis() - start) < MAX_WAIT_TIMEOUT);

    wdt_reset();    //reset the watchdog timer

    if (this->_debugLevel ==2) Serial.println("");
    if (this->_debugLevel >0) Serial.println(F("End Resp"));
    delay(100);

    // Clean up
    XMIT->end();	//close the serial port to the GPS so it doens't draw excess current
    delete XMIT;
    XMIT = nullptr;

    //Push the PTT
    digitalWrite(this->_pinPTT, HIGH);   //There's a delay of about 37mS from PTT going high to when RF is emitted.
  } else {
    //End of transmission. stop the PTT and shut down the transmitter
    digitalWrite(this->_pinPTT, LOW);
    digitalWrite(this->_pinEnable, LOW);

    pinMode(this->_pinTxAudio, INPUT);    //Configure as an input until we need it. This will save power.
  }
}


/**
 * @brief  Sets the debug level for the APRS object.
 * @param  level: The debug level to set. 0=none, 1=Normal Serial.printlns, 2=Verbose feedback from 818V module
 */
void Modem::setDebugLevel(uint8_t level) {
  this->_debugLevel = level;
}


/**
 * @brief Starts the process of sending an APRS packet. Header information is passed in to address and route the packet appropriately.
 * @param szDest The destination callsign of the packet
 * @param destSSID The destination SSID of the packet
 * @param szCall The source callsign of the packet
 * @param callSSID The source SSID of the packet
 * @param szPath1 The first path of the packet
 * @param path1SSID The first path's SSID
 * @param szPath2 The second path of the packet
 * @param path2SSID The second path's SSID
 * @param bUsePath A boolean indicating whether or not to use the path information
 * @note  This function is called to start the transmission of a packet.  It will start the timer and begin the process of transmitting the packet.  Calling this function indicates that the packet is ready to be transmitted.  The packet will be transmitted in the background, and the function will return immediately.
 */
void Modem::packetHeader(char *szDest, char destSSID, char *szCall, char callSSID, char *szPath1, char path1SSID, char *szPath2, char path2SSID, bool bUsePath) {

  uint8_t i;
  
  //Add the destination address
  for (i=0; i<6; i++) {
    this->packetAppend((char)(szDest[i] << 1));
  }
  this->packetAppend((char)(destSSID << 1));
  
  //Add the from callsign
  for (i=0; i<6; i++) {
    this->packetAppend((char)(szCall[i] << 1));
  }
  
  if ((szPath1[0] != ' ') && (bUsePath)) {
    //there's a path
    
    //don't end the callsign section
    this->packetAppend((char)((callSSID << 1) | 0x00));    //DON'T flag the last bit with a 1 to indicate end of string

    //dump the Path1
    for (int j=0; j<6; j++) {
      this->packetAppend((char)(szPath1[j] << 1));
    }
    if (szPath2[0] != ' ') {
      //there's a second path
      
      //don't end the Path1 section
      this->packetAppend((char)((path1SSID << 1) | 0x00));    //DON'T flag the last bit with a 1 to indicate end of string
      
      for (int k=0; k<6; k++) {
        this->packetAppend((char)(szPath2[k] << 1));
      }
      
      //Max of 2 paths, so always end here
      this->packetAppend((char)((path2SSID << 1) | 0x01));    //flag the last bit with a 1 to indicate end of string
      
    } else {
      //This was the only path - end it
      this->packetAppend((char)((path1SSID << 1) | 0x01));    //flag the last bit with a 1 to indicate end of string
    }
  } else {
    //this is the end of the callsign, there was no Paths
    this->packetAppend((char)((callSSID << 1) | 0x01));    //flag the last bit with a 1 to indicate end of string
  }
  
  this->packetAppend((char)0x03);    //Control Byte
  this->packetAppend((char)0xF0);    //PID    
}


/** 
 * @brief Appends a string to the packet buffer.  This function will append the string to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param sz The string to append to the packet buffer.
 */
void Modem::packetAppend(char *sz) {
  while (*sz != 0) {
    this->packetAppend(*sz);
    sz++;
  }	
}


/**
 * @brief Appends a character to the packet buffer.  This function will append the character to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param c The character to append to the packet buffer.
 */
void Modem::packetAppend(char c) {
  if (_iSZLen < (MAX_SZXMIT_SIZE - 2)) {
    //we still have room in the array
    _szXmit[++_iSZLen] = c;
    _szXmit[_iSZLen + 1] = '\0';    //null terminate
  }  
}


/**
 * @brief Appends a float to the packet buffer.  This function will append the float to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param f The float to append to the packet buffer.
 * @note  This is output a positive or negative float, with a single decimal point. Note that sprintf() does not support floats, so this function is a workaround.
 * @note  The float is FLOORed at a single decimal point.  For example, 12.31 will be output as 12.3, and 12.39 will also be output as 12.3.
 */
void Modem::packetAppend(float f) {
  if (f < 0) {
    this->packetAppend('-');    //negative sign
    f = f * -1;        //convert the negative number to positive for the rest of the processing
  }
  
  this->packetAppend((long)int(f), false);  //prints the int part
  this->packetAppend('.'); // print the decimal point
  int iDec = (f - int(f)) * 10;    //calculate the decimal point portion
  this->packetAppend((long)iDec, false) ; 
}


/**
 * @brief Appends a long to the packet buffer.  This function will append the long to the packet buffer, and the packet will be transmitted when the packetSend() function is called.
 * @param lNumToSend The long to append to the packet buffer.
 * @param bLeadingZero A boolean indicating whether or not to pad the number with leading zeros. If padded, the number will be 6 digits long.
 */
void Modem::packetAppend(long lNumToSend, bool bLeadingZero) {
  char szTemp[8];
  if (bLeadingZero)  sprintf(szTemp, "%06lu", lNumToSend);    //convert the number to a string
  else sprintf(szTemp, "%lu", lNumToSend);    //convert the number to a string

  this->packetAppend(szTemp);    //append the string to the packet buffer
}


/**
 * @brief This function is called to start the transmission of a packet.  It will start the timer and begin the process of transmitting the packet.
 * @note  Calling this function indicates that the packet is ready to be transmitted.  For internally modulated signals (not KISS), the signal will transmit through the timer ISR, but this function will wait until it's finished to return.
 */
void Modem::packetSend() {

  //Initialize the state machine
  this->_iTxState = 0;
  this->_iSZPos = 0;
  this->_iTxDelayRemaining = this->_txDelay;    //start off with a txDelay parameter
  this->_CRC = 0xFFFF;    //init the CRC variable


  //Keep track of the time we started transmitting
  this->setLastTransmitMillis();

  //Key the transmitter
  this->PTT(true);

  //Start the interrupt routine to modulate the signal
  this->timer1ISR(true);

  //wait for the state machine to get to a State 5, which is when it shuts down the transmitter.
  while (_iTxState != 6) {
    delay(100);    //just wait patiently until the packet is done
  }

  _iSZLen = -1;      //reset back to a clean buffer
}


/**
 * @brief  Sends a test tone to the transmitter.  This function is used to test the transmitter and the audio path.
 */
void Modem::sendTestDiagnotics() {

  wdt_reset();    //reset the watchdog timer

  //Keep track of the time we started transmitting
  this->_lastTransmitMillis = millis();

  this->PTT(true);
  delay(DIAGNOSTIC_DELAY);   //deadkey before starting the tones.

  wdt_reset();    //reset the watchdog timer

  this->_iTxState = 12;    //set the state to 12 to indicate a constant test tone. Use 12 because it resets if there was previously a courtesy tone.
  this->timer1ISR(true);
  delay(DIAGNOSTIC_DELAY);
  delay(DIAGNOSTIC_DELAY);

  wdt_reset();    //reset the watchdog timer

  //Swap tones
  this->_iTxState = 12;    //temporarily set the state to 12 to flip the tone to the opposite.
  delay(DIAGNOSTIC_DELAY);
  delay(DIAGNOSTIC_DELAY);

  wdt_reset();    //reset the watchdog timer

  //Alternate the tones
  this->_iTxState = 13;    //set the state to 11 to indicate a constant test tone
  delay(DIAGNOSTIC_DELAY);
  delay(DIAGNOSTIC_DELAY);

  wdt_reset();    //reset the watchdog timer
  
  this->timer1ISR(false);   //stop the tones
  this->_iTxState = 0;
  delay(DIAGNOSTIC_DELAY);   //dead key for a little bit

  this->PTT(false);   //shut down the transmitter
}


/**
 * @brief  Calculates the CRC Checksums for the AX.25 packet.
 * @param  iBit: The bit to calculate the CRC for.
 */
void Modem::calcCRC(bool bit) {
  unsigned int xor_int;
  
  xor_int = this->_CRC ^ bit;				// XOR lsb of CRC with the latest bit
  this->_CRC >>= 1;									// Shift 16-bit CRC one bit to the right

  if (xor_int & 0x0001) {					// If XOR result from above has lsb set
    this->_CRC ^= 0x8408;							// Shift 16-bit CRC one bit to the right
  }
  return;
}


/**
 * @brief  The main component of the transmitting State Machine. After a packetSend() command is sent, this function gets called repeatedly from the ISR timer routine to output the packet bit-by-bit..
 * @return The next bit in the packet to be transmitted.
 */
uint8_t Modem::getNextBit() {
  static int iRotatePos = 0;
  uint8_t bOut = 0;
  

  switch (this->_iTxState) {
  case 0:
    //Transmitting the preamble (0x7e) TXDelays flag
    this->_bNoStuffing = true;
    bOut = ((0x7e & (1 << iRotatePos)) != 0);    //get the bit
  
    
    if (iRotatePos != 7) iRotatePos++;
    else {
      //rotate around for the next bit

      iRotatePos = 0;
     
      if (this->_iTxDelayRemaining > 0) {
        //we have more txdelays
        this->_iTxDelayRemaining--;
      } else {
        //We're done with the preamble, so drop down to the next state
        this->_iTxState = 1;
      }
    }
    break;
  case 1:
    //normal packet payload
    this->_bNoStuffing = false;    //we're done with the preamble, so no more bit stuffing for the end-of-packet flag

    bOut = ((_szXmit[_iSZPos] & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      //we need to get the next byte if possible

      iRotatePos = 0;
      
      if (_iSZPos < _iSZLen) {
        //we have more bytes
        _iSZPos++;
      } else {
        this->_iTxState = 2;    //drop down and transmit the CRC bytes
      }
    }
    
    
    //calculate the CRC for the data portion of the packet
    this->calcCRC(bOut);
    
    break;  
  case 2:  
    //Transmit the CRC Check bits (first byte)
  
    bOut = (((lowByte(this->_CRC) ^ 0xff) & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      iRotatePos = 0;
      this->_iTxState = 3;    //we need to drop down to the next state
    }  
    break;
  
  case 3:
    //Transmit the CRC Check bits (second byte)
    bOut = (((highByte(this->_CRC) ^ 0xff) & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      iRotatePos = 0;
      this->_iTxState = 4;    //we need to drop down to the next state
    }  
    break;

  case 4:  
    //send end-of-packet flag

    this->_bNoStuffing = true;    //we're done with the CRC, so no more bit stuffing for the end-of-packet flag

    bOut = ((0x7e & (1 << iRotatePos)) != 0);    //get the bit
  
    if (iRotatePos != 7) iRotatePos++;
    else {
      this->_iTxState = 5;    //we need to drop down to the next state
    } 
    break;


  case 5:
    //done - shut the transmitter down unless we need a courtesy tone beep
    
    this->_iTxState = 6;    //set the state to 6 to indicate a courtesy tone
    iRotatePos = 0;    //reset to get ready for next byte

    if (this->_courtesyTone) {
      //send a courtesy tone
      bOut = 2;
    } else {
      this->timer1ISR(false);    //stop the timer
      
      //Unkey the transmitter
      this->PTT(false);
      bOut = 1;
    }
    
    
    
  
    break;
  case 6:
    //Shut down the transmitter following a courtesy tone

    this->timer1ISR(false);    //stop the timer
    
    //Unkey the transmitter
    this->PTT(false);
    bOut = 1;
    break;
  case 11:
    //Send a constant tone
    this->_bNoStuffing = true;
    bOut = 1;    //send high which means stay with whatever tone was used last
    break;
  case 12:
    //swap the tone
    bOut = 0;    //swap the tone
    this->_iTxState = 11;    //After it's been swapped, go back to the constant tone
    break;
  case 13:
    //alternate the tones
    bOut = 0;    //send a low, which means switch between high and low tones repeatedly
    break;

  default:
    //this should never happen, but if it does, set to state 6 to shut down the transmitter
    this->_iTxState = 6;
    bOut = true;
  }

  return bOut;  
}


/**
 * @brief  Returns the current state of the noBitStuffing flag.  This flag is used to indicate whether or not to stuff an extra bit into the packet because of an excessive number of 1's in a row.
 * @return A boolean indicating whether or not to stuff an extra bit into the packet.
 * @note  This function is called from the ISR routine.  It must be public for now.
 */
bool Modem::noBitStuffing() {
  return this->_bNoStuffing;  
}


/**
 * @brief  Configures the ISR times to create the AX.25 packet and the audio wave forms.
 */
void Modem::configTimers() {
  //Timer1 drives the IRQ for generating the audio frequency and baud rate.
  TCCR1A = 0x00;
  TCCR1B = 0x09;    //Set WGM12 high, and set CS=001 (which is clk/1, or no prescaler);
  
  //The Overflow Control Register is a count-down timer that triggers the ISR when it reaches zero.
  //The count down depends on the clock frequency and the prescaler.
  //  16MHz clock with no prescaler:  666
  //  8MHz clock with no prescaler:  333
  //
  // Equations:
  //  OSC = The clock oscilator frequency in Hz. 3.3V ATMega328P is limited to 8MHz. 5.0V can go up to 16MHz.
  //  BG =  The baud rate generator multiplier. This should be a value greater than 16, and less than about 48. Going too high could cause the 
  //         ISR to overflow. Too a value will result in poor quality audio being generated. BG effectively sets the number of steps within the NCO.
  //  OCR = The overflow counter value. This is the value that the timer will count down to. The lower the value, the higher the frequency. This is
  //         different than the prescaler, but is a way to fine-tune the prescaler.
  //
  //  Select an appropriate BG value, then calculate the OCR value. The formula is:
  //  OCR = OSC / (BG * 1200)     (for 1200 baud)
  //
  //  Then calculate the high and low tone step values for the NCO.
  //  TONE_HIGH_STEPS_PER_TICK = (65536 * OCR * 2200) / OSC     (for 2200Hz high tone)
  //  TONE_LOW_STEPS_PER_TICK = (65536 * OCR * 1200) / OSC      (for 1200Hz low tone)
  //
  //  Some values that I've used in the past:
  //
  //     Variable      |  16MHz  |   8MHz  |  Notes
  //     --------------------------------------------------------------------------------------------------------------------------
  //     OCR           |  666    |   333   |
  //     BG            |  20     |   20    |  20 works out to be evenly divisible by 1200 producing a low error rate. Audio quality is marginal at this rate.
  //     TONE_HIGH     |  6001   |   6001  |  These are the same value for either clock frequency as the OCR normalizes the value.
  //     TONE_LOW      |  3273   |   3273  |


  OCR1A = this->TIMER1_OCR;


  //Timer2 drives the PWM frequency.  We need it sufficiently higher than the 1200/2200hz tones
  //Set timer2 to fast PWM mode
  TCCR2A = (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << WGM22) | (1 << CS20);    //set prescaler to clk/1

  OCR2A = 0xc8;    //Overflow counter for Timer2. This needs to be kept in sync with the maximum value in the arySin table.
}


/**
 * @brief  Starts and stops the Timer1 ISR routine.
 * @param  run: A boolean indicating to either start or stop the ISR routine.
 */
void Modem::timer1ISR(bool run) {
  if (run) sbi(TIMSK1, OCIE1A);
  else cbi(TIMSK1, OCIE1A);
}


/**
 * @brief  Sets the delay between the end of the packet and the transmitter being unkeyed.
 * @param  txDelay: The number of bytes to cycle through before the packet is transmitted.
 */
void Modem::setTxDelay(unsigned int txDelay) {
  this->_txDelay = txDelay;
}


/**
 * @brief  Returns the pin number for the PTT line.
 * @return The pin number for the PTT line.
 */
uint8_t Modem::getPinTxAudio() {
  return this->_pinTxAudio;
}
