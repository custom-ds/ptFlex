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

    //set the analog reference to the internal 1.1V reference.  This is used for the battery voltage measurement.
    analogReference(INTERNAL);

    this->_annunciateMode = annunciateMode;
  }


/**
 * @brief Annunciate a character via the LED and/or buzzer.
 * @param c The character to annunciate.
 */
void ptTracker::annunciate(char c) {

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
    wdt_reset();    //reset the watchdog timer
}


/**
 * @brief Reads the battery voltage and returns the value in volts.
 * @param bSerialOut If true, then the battery voltage will be printed to the serial port.
 * @return The battery voltage in volts.
 */
float ptTracker::readBatteryVoltage(bool bSerialOut) {
    int iBattery = analogRead(this->_pinBattery);

    //Convert value to volts.
    // Max value: 1024
    // 5.0V systems: 1024/5.0 = 204.8 points per volt
    // 3.3V systems: 1024/3.3 = 310.3 points per volt
    // 1.1V Internal Reference: 1024/1.1 = 930.9 points per volt
    float fVolts = (float)iBattery / 930.9;

    //Compensate for the resistor divider on the battery voltage measurement circuit
    // TotalResistance / BottomResistor
    // Top: 100k, Bottom: 22k, Total: 122k
    // 122k / 22k = 5.545
    fVolts = fVolts * 5.545;        //times (122/22) to adjust for the resistor divider (5.545)

    //If you want to compensate for a diode drop, add it back here, in Volts.
    //  fVolts = fVolts + 0.19;

    if (bSerialOut) {
        Serial.print(F("Batt: "));
        Serial.print(fVolts);
        Serial.println("V");    
    }

    wdt_reset();    //reset the watchdog timer
    return fVolts;
}


/**
 * @brief Annunciate a tone on the audio annunciator.
 * @param length The length of the tone in microseconds.
 * @note This function is called regardless of whether the audio annunciator is enabled or not, in order to provide consistent timing to the LED annunciator.
 */
void ptTracker::audioTone(int length) {
    wdt_reset();    //reset the watchdog timer

    //Set the pins to outputs if they are needed for the annunciation mode selected
    if (this->_annunciateMode & 0x01) {
        pinMode(this->_pinLED, OUTPUT);
    }
    if (this->_annunciateMode & 0x02) {
        pinMode(this->_pinPiezo, OUTPUT);
    }

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
