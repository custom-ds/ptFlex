/*
Board-specific functions for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef ptTracker_h
#define ptTracker_h

#include "BoardDef.h"

#include <stdint.h>   //standard data types available, such as uint8_t
#include <arduino.h>
#include <avr/wdt.h>

//Anunciator Settings
#define DELAY_DAH 650
#define DELAY_DIT 200
#define DELAY_GAP 150


class ptTracker {
  public:
    // Constructor
    ptTracker(uint8_t pinLED, uint8_t pinPiezo, uint8_t pinBattery, uint8_t annunciateMode);

    // Public Functions
    void annunciate(char c);
    float readBatteryVoltage(bool bSerialOut);
    void setAnnunciateMode(uint8_t mode) { this->_annunciateMode = mode; }
    uint8_t getAnnunciateMode() { return this->_annunciateMode; }
    void (*reboot) (void) = 0;    //function pointer to the reboot the Tracker

  private:
    // Private Variables
    uint8_t _pinLED;
    uint8_t _pinPiezo;
    uint8_t _pinBattery;
    uint8_t _annunciateMode;        //0=No annunciator, 1=LED only, 2=LED and buzzer

    // Private Functions
    void audioTone(int length);
};
#endif
