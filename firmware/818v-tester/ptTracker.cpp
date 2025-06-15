/*
Board-specific functions for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

Version History:
Version 1.0.0 - March 9, 2025 - Initial Release.

*/

#include "ptTracker.h"



ptTracker::ptTracker(uint8_t pinLED, uint8_t pinPiezo, uint8_t pinBattery, uint8_t annunciateMode) {
    this->_pinLED = pinLED;
    this->_pinPiezo = pinPiezo;
    this->_pinBattery = pinBattery;

    //Set everything to inputs - we will set the outputs when we need them
    pinMode(this->_pinLED, INPUT);  
    pinMode(this->_pinPiezo, INPUT);
    pinMode(this->_pinBattery, INPUT);

    this->_annunciateMode = annunciateMode;
  }


/**
 * @brief Annunciate a character via the LED and/or buzzer.
 * @param c The character to annunciate.
 */
void ptTracker::annunciate(char c) {
    //send an anunciator via LED and/or buzzer, depending on config

    switch (c) {
        case 'c':
        //Used when entering configuration mode
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        break;
    case 'e':
        //single chirp - Used during initialization of uBlox GPS
        this->audioTone(DELAY_DIT);
        break;
    case 'g':
        //Used during complete loss of GPS signal
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        break;
    case 'i':
        //double chirp - Used to confirm initialization of uBlox GPS
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        break;
    case 'k':
        //Initial "OK"
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        break;
    case 'l':
        //Used during loss of GPS lock
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        break;
    case 't':
        //Exercising and testing the Transmitter
        audioTone(DELAY_DAH);
        break;
    case 'w':
        //Indicates that configuration settings were written to EEPROM
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        break;
    case 'x':
        //Used when exercising the board to test for functionality
        this->audioTone(DELAY_DAH);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DIT);
        delay(DELAY_GAP);
        this->audioTone(DELAY_DAH);
        break;
    }
}


/**
 * @brief Reads the battery voltage and returns the value in volts.
 * @param bSerialOut If true, then the battery voltage will be printed to the serial port.
 * @return The battery voltage in volts.
 */
float ptTracker::readBatteryVoltage(bool bSerialOut) {
    int iBattery = analogRead(this->_pinBattery);
    float fVolts = (float)iBattery / 310.3;    //204.8 points per volt for 5.0V systems, 310.3 for 3.3V systems!!!!,
    fVolts = fVolts * 5.545;        //times (122/22) to adjust for the resistor divider (5.545)
    //  fVolts = fVolts + 0.19;      //account for the inline diode on the power supply  // not interested in diode drop for solar purposes??????????????????????????????????????????????????

    if (bSerialOut) {
        Serial.print(F("Batt: "));
        Serial.print(fVolts);
        Serial.println("V");    
    }

    return fVolts;
}


/**
 * @brief Annunciate a tone on the audio annunciator.
 * @param length The length of the tone in microseconds.
 * @note This function is called regardless of whether the audio annunciator is enabled or not, in order to provide consistent timing to the LED annunciator.
 */
void ptTracker::audioTone(int length) {

    //Set the pins to outputs
    pinMode(this->_pinLED, OUTPUT);  
    pinMode(this->_pinPiezo, OUTPUT);

    if (this->_annunciateMode & 0x01) {
        digitalWrite(this->_pinLED, HIGH);
    }

    for (int i = 0; i<length; i++) {
        if (this->_annunciateMode & 0x02) {
        digitalWrite(this->_pinPiezo, HIGH);
        }
        delayMicroseconds(200);

        if (this->_annunciateMode & 0x02) {
        digitalWrite(this->_pinPiezo, LOW);
        }
        delayMicroseconds(200);
    }

    if (this->_annunciateMode & 0x01) {
        digitalWrite(this->_pinLED, LOW);
    }

    //Set the pins back to inputs to save power
    pinMode(this->_pinLED, INPUT);  
    pinMode(this->_pinPiezo, INPUT);
}
