/* ========================= .cpp ================================= */

/*
  RCSwitch - Arduino libary for remote control outlet switches
  Copyright (c) 2011 Suat Özgür.  All right reserved.

  Contributors:
  - Andre Koehler / info(at)tomate-online(dot)de
  - Gordeev Andrey Vladimirovich / gordeev(at)openpyro(dot)com
  - Skineffect / http://forum.ardumote.com/viewtopic.php?f=2&t=46
  - Dominik Fischer / dom_fischer(at)web(dot)de
  - Frank Oltmanns / <first name>.<last name>(at)gmail(dot)com
  - Andreas Steinel / A.<lastname>(at)gmail(dot)com

  Project home: http://code.google.com/p/rc-switch/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "RCSwitch.h"
#include "spark_wiring_usbserial.h"

#if not defined( RCSwitchDisableReceiving )
RCSwitch::InterruptData RCSwitch::receiverInterrupts[RCSWITCH_MAX_RX_PINS];
int RCSwitch::nInterruptSourcePin = -1;
int RCSwitch::nReceiveTolerance = 60;
#endif
unsigned int RCSwitch::timings[RCSWITCH_MAX_RX_PINS][RCSWITCH_MAX_CHANGES];

RCSwitch::RCSwitch() {
	this->nTransmitterPin = -1;
	this->setPulseLength(350);
	this->setRepeatTransmit(10);
	this->setProtocol(1);
	#if not defined( RCSwitchDisableReceiving )
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		this->receiverInterrupts[i] = RCSwitch::InterruptData();
	}
	this->nInterruptSourcePin = -1;
	this->setReceiveTolerance(60);
	#endif
}

/**
 * Sets the protocol to send.
 */
void RCSwitch::setProtocol(int nProtocol) {
	this->nProtocol = nProtocol;
	switch (nProtocol) {
		case 1 : this->setPulseLength(350); break;
		case 2 : this->setPulseLength(650); break;
		case 3 : this->setPulseLength(100); break;
	}
}

/**
 * Sets the protocol to send with pulse length in microseconds.
 */
void RCSwitch::setProtocol(int nProtocol, int nPulseLength) {
	this->nProtocol = nProtocol;
	this->setPulseLength(nPulseLength);
}


/**
 * Sets pulse length in microseconds
 */
void RCSwitch::setPulseLength(int nPulseLength)
	{ this->nPulseLength = nPulseLength; }

/**
 * Sets Repeat Transmits
 */
void RCSwitch::setRepeatTransmit(int nRepeatTransmit)
	{ this->nRepeatTransmit = nRepeatTransmit; }

/**
 * Set Receiving Tolerance
 */
#if not defined( RCSwitchDisableReceiving )
void RCSwitch::setReceiveTolerance(int nPercent)
	{ this->nReceiveTolerance = nPercent; }
#endif


/**
 * Enable transmissions
 *
 * @param nTransmitterPin    Arduino Pin to which the sender is connected to
 */
void RCSwitch::enableTransmit(int nTransmitterPin) {
	this->nTransmitterPin = nTransmitterPin;
	pinMode(this->nTransmitterPin, OUTPUT);
}

/**
 * Disable transmissions
 */
void RCSwitch::disableTransmit() {
	this->nTransmitterPin = -1;
}

/**
 * Switch a remote switch on (Type D REV)
 *
 * @param sGroup        Code of the switch group (A,B,C,D)
 * @param nDevice       Number of the switch itself (1..3)
 */
void RCSwitch::switchOn(char sGroup, int nDevice) {
	this->sendTriState( this->getCodeWordD(sGroup, nDevice, true) );
}

/**
 * Switch a remote switch off (Type D REV)
 *
 * @param sGroup        Code of the switch group (A,B,C,D)
 * @param nDevice       Number of the switch itself (1..3)
 */
void RCSwitch::switchOff(char sGroup, int nDevice) {
	this->sendTriState( this->getCodeWordD(sGroup, nDevice, false) );
}

/**
 * Switch a remote switch on (Type C Intertechno)
 *
 * @param sFamily  Familycode (a..f)
 * @param nGroup   Number of group (1..4)
 * @param nDevice  Number of device (1..4)
 */
void RCSwitch::switchOn(char sFamily, int nGroup, int nDevice) {
	this->sendTriState( this->getCodeWordC(sFamily, nGroup, nDevice, true) );
}

/**
 * Switch a remote switch off (Type C Intertechno)
 *
 * @param sFamily  Familycode (a..f)
 * @param nGroup   Number of group (1..4)
 * @param nDevice  Number of device (1..4)
 */
void RCSwitch::switchOff(char sFamily, int nGroup, int nDevice) {
	this->sendTriState( this->getCodeWordC(sFamily, nGroup, nDevice, false) );
}

/**
 * Switch a remote switch on (Type B with two rotary/sliding switches)
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 */
void RCSwitch::switchOn(int nAddressCode, int nChannelCode) {
	this->sendTriState( this->getCodeWordB(nAddressCode, nChannelCode, true) );
}

/**
 * Switch a remote switch off (Type B with two rotary/sliding switches)
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 */
void RCSwitch::switchOff(int nAddressCode, int nChannelCode) {
	this->sendTriState( this->getCodeWordB(nAddressCode, nChannelCode, false) );
}

/**
 * Deprecated, use switchOn(char* sGroup, char* sDevice) instead!
 * Switch a remote switch on (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param nChannelCode  Number of the switch itself (1..5)
 */
void RCSwitch::switchOn(char* sGroup, int nChannel) {
	char* code[6] = { "00000", "10000", "01000", "00100", "00010", "00001" };
	this->switchOn(sGroup, code[nChannel]);
}

/**
 * Deprecated, use switchOff(char* sGroup, char* sDevice) instead!
 * Switch a remote switch off (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param nChannelCode  Number of the switch itself (1..5)
 */
void RCSwitch::switchOff(char* sGroup, int nChannel) {
	char* code[6] = { "00000", "10000", "01000", "00100", "00010", "00001" };
	this->switchOff(sGroup, code[nChannel]);
}

/**
 * Switch a remote switch on (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param sDevice       Code of the switch device (refers to DIP switches 6..10 (A..E) where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 */
void RCSwitch::switchOn(char* sGroup, char* sDevice) {
	this->sendTriState( this->getCodeWordA(sGroup, sDevice, true) );
}

/**
 * Switch a remote switch off (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param sDevice       Code of the switch device (refers to DIP switches 6..10 (A..E) where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 */
void RCSwitch::switchOff(char* sGroup, char* sDevice) {
	this->sendTriState( this->getCodeWordA(sGroup, sDevice, false) );
}

/**
 * Returns a char[13], representing the Code Word to be sent.
 * A Code Word consists of 9 address bits, 3 data bits and one sync bit but in our case only the first 8 address bits and the last 2 data bits were used.
 * A Code Bit can have 4 different states: "F" (floating), "0" (low), "1" (high), "S" (synchronous bit)
 *
 * +-------------------------------+--------------------------------+-----------------------------------------+-----------------------------------------+----------------------+------------+
 * | 4 bits address (switch group) | 4 bits address (switch number) | 1 bit address (not used, so never mind) | 1 bit address (not used, so never mind) | 2 data bits (on|off) | 1 sync bit |
 * | 1=0FFF 2=F0FF 3=FF0F 4=FFF0   | 1=0FFF 2=F0FF 3=FF0F 4=FFF0    | F                                       | F                                       | on=FF off=F0         | S          |
 * +-------------------------------+--------------------------------+-----------------------------------------+-----------------------------------------+----------------------+------------+
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 * @param bStatus       Wether to switch on (true) or off (false)
 *
 * @return char[13]
 */
char* RCSwitch::getCodeWordB(int nAddressCode, int nChannelCode, boolean bStatus) {
	int nReturnPos = 0;
	static char sReturn[13];

	char* code[5] = { "FFFF", "0FFF", "F0FF", "FF0F", "FFF0" };
	if (nAddressCode < 1 || nAddressCode > 4 || nChannelCode < 1 || nChannelCode > 4) {
		return '\0';
	}
	for (int i = 0; i<4; i++) {
		sReturn[nReturnPos++] = code[nAddressCode][i];
	}

	for (int i = 0; i<4; i++) {
		sReturn[nReturnPos++] = code[nChannelCode][i];
	}

	sReturn[nReturnPos++] = 'F';
	sReturn[nReturnPos++] = 'F';
	sReturn[nReturnPos++] = 'F';

	if (bStatus) {
		sReturn[nReturnPos++] = 'F';
	} else {
		sReturn[nReturnPos++] = '0';
	}

	sReturn[nReturnPos] = '\0';

	return sReturn;
}

/**
 * Returns a char[13], representing the Code Word to be sent.
 *
 * getCodeWordA(char*, char*)
 *
 */
char* RCSwitch::getCodeWordA(char* sGroup, char* sDevice, boolean bOn) {
	static char sDipSwitches[13];
	int i = 0;
	int j = 0;

	for (i=0; i < 5; i++) {
		if (sGroup[i] == '0') {
			sDipSwitches[j++] = 'F';
		} else {
			sDipSwitches[j++] = '0';
		}
	}

	for (i=0; i < 5; i++) {
		if (sDevice[i] == '0') {
			sDipSwitches[j++] = 'F';
		} else {
			sDipSwitches[j++] = '0';
		}
	}

	if (bOn) {
		sDipSwitches[j++] = '0';
		sDipSwitches[j++] = 'F';
	} else {
		sDipSwitches[j++] = 'F';
		sDipSwitches[j++] = '0';
	}

	sDipSwitches[j] = '\0';

	return sDipSwitches;
}

/**
 * Like getCodeWord (Type C = Intertechno)
 */
char* RCSwitch::getCodeWordC(char sFamily, int nGroup, int nDevice, boolean bStatus) {
	static char sReturn[13];
	int nReturnPos = 0;

	if ( (byte)sFamily < 97 || (byte)sFamily > 112 || nGroup < 1 || nGroup > 4 || nDevice < 1 || nDevice > 4) {
		return '\0';
	}

	char* sDeviceGroupCode =  dec2binWzerofill(  (nDevice-1) + (nGroup-1)*4, 4  );
	char familycode[16][5] = { "0000", "F000", "0F00", "FF00", "00F0", "F0F0", "0FF0", "FFF0", "000F", "F00F", "0F0F", "FF0F", "00FF", "F0FF", "0FFF", "FFFF" };
	for (int i = 0; i<4; i++) {
		sReturn[nReturnPos++] = familycode[ (int)sFamily - 97 ][i];
	}
	for (int i = 0; i<4; i++) {
		sReturn[nReturnPos++] = (sDeviceGroupCode[3-i] == '1' ? 'F' : '0');
	}
	sReturn[nReturnPos++] = '0';
	sReturn[nReturnPos++] = 'F';
	sReturn[nReturnPos++] = 'F';
	if (bStatus) {
		sReturn[nReturnPos++] = 'F';
	} else {
		sReturn[nReturnPos++] = '0';
	}
	sReturn[nReturnPos] = '\0';
	return sReturn;
}

/**
 * Decoding for the REV Switch Type
 *
 * Returns a char[13], representing the Tristate to be send.
 * A Code Word consists of 7 address bits and 5 command data bits.
 * A Code Bit can have 3 different states: "F" (floating), "0" (low), "1" (high)
 *
 * +-------------------------------+--------------------------------+-----------------------+
 * | 4 bits address (switch group) | 3 bits address (device number) | 5 bits (command data) |
 * | A=1FFF B=F1FF C=FF1F D=FFF1   | 1=0FFF 2=F0FF 3=FF0F 4=FFF0    | on=00010 off=00001    |
 * +-------------------------------+--------------------------------+-----------------------+
 *
 * Source: http://www.the-intruder.net/funksteckdosen-von-rev-uber-arduino-ansteuern/
 *
 * @param sGroup        Name of the switch group (A..D, resp. a..d)
 * @param nDevice       Number of the switch itself (1..3)
 * @param bStatus       Wether to switch on (true) or off (false)
 *
 * @return char[13]
 */

char* RCSwitch::getCodeWordD(char sGroup, int nDevice, boolean bStatus) {
	static char sReturn[13];
	int nReturnPos = 0;

	// Building 4 bits address
	// (Potential problem if dec2binWcharfill not returning correct string)
	char *sGroupCode;
	switch(sGroup) {
		case 'a':
		case 'A':
			sGroupCode = dec2binWcharfill(8, 4, 'F'); break;
		case 'b':
		case 'B':
			sGroupCode = dec2binWcharfill(4, 4, 'F'); break;
		case 'c':
		case 'C':
			sGroupCode = dec2binWcharfill(2, 4, 'F'); break;
		case 'd':
		case 'D':
			sGroupCode = dec2binWcharfill(1, 4, 'F'); break;
		default:
			return '\0';
	}

	for (int i = 0; i<4; i++) {
		sReturn[nReturnPos++] = sGroupCode[i];
	}


	// Building 3 bits address
	// (Potential problem if dec2binWcharfill not returning correct string)
	char *sDevice;
	switch(nDevice) {
		case 1: sDevice = dec2binWcharfill(4, 3, 'F'); break;
		case 2: sDevice = dec2binWcharfill(2, 3, 'F'); break;
		case 3: sDevice = dec2binWcharfill(1, 3, 'F'); break;
		default: return '\0';
	}

	for (int i = 0; i<3; i++) {
		sReturn[nReturnPos++] = sDevice[i];
	}

	// fill up rest with zeros
	for (int i = 0; i<5; i++) {
		sReturn[nReturnPos++] = '0';
	}

    // encode on or off
	if (bStatus) {
		sReturn[10] = '1';
	} else {
		sReturn[11] = '1';
	}

	// last position terminate string
	sReturn[12] = '\0';
	return sReturn;
}

/**
 * @param sCodeWord   /^[10FS]*$/  -> see getCodeWord
 */
void RCSwitch::sendTriState(char* sCodeWord) {
	for (int nRepeat=0; nRepeat<nRepeatTransmit; nRepeat++) {
		int i = 0;
		while (sCodeWord[i] != '\0') {
			switch(sCodeWord[i]) {
				case '0': this->sendT0(); break;
				case 'F': this->sendTF(); break;
				case '1': this->sendT1(); break;
			}
			i++;
		}
		this->sendSync();
	}
}

void RCSwitch::send(unsigned long Code, unsigned int length) {
	this->send( this->dec2binWzerofill(Code, length) );
}

void RCSwitch::send(char* sCodeWord) {
	for (int nRepeat=0; nRepeat<nRepeatTransmit; nRepeat++) {
		int i = 0;
		while (sCodeWord[i] != '\0') {
			switch(sCodeWord[i]) {
				case '0': this->send0(); break;
				case '1': this->send1(); break;
			}
			i++;
		}
		this->sendSync();
	}
}

void RCSwitch::transmit(int nHighPulses, int nLowPulses) {
	if (this->nTransmitterPin == -1) {
		return;
	}

	#if not defined ( RCSwitchDisableReceiving )
	boolean disabled_receive = false;
	RCSwitch::InterruptData receiverInterrupts_backup[RCSWITCH_MAX_RX_PINS] =
		this->receiverInterrupts;
		// Arrays are copied by value -- i.e., a deep copy.
	for (int i=0; i<RCSWITCH_MAX_RX_PINS; ++i) {
		if (this->receiverInterrupts[i].interrupt != -1) {
			this->disableReceive();
			disabled_receive = true;
			break;
		}
	}
	#endif

	digitalWrite(this->nTransmitterPin, HIGH);
	delayMicroseconds( this->nPulseLength * nHighPulses);
	digitalWrite(this->nTransmitterPin, LOW);
	delayMicroseconds( this->nPulseLength * nLowPulses);

	#if not defined( RCSwitchDisableReceiving )
	if (disabled_receive) {
		for (int i=0; i<RCSWITCH_MAX_RX_PINS; ++i) {
			this->enableReceive(receiverInterrupts_backup[i].interrupt);
			this->receiverInterrupts[getInterruptIndex(receiverInterrupts_backup[i].interrupt)] =
				receiverInterrupts_backup[i];
				// The default assignment operator does a member-wise,
				// recursive assignment of each member.
		}
	}
	#endif
}

/**
 * Sends a "0" Bit
 *                       _
 * Waveform Protocol 1: | |___
 *                       _
 * Waveform Protocol 2: | |__
 */
void RCSwitch::send0() {
	if (this->nProtocol == 1){
		this->transmit(1,3);
	} else if (this->nProtocol == 2) {
		this->transmit(1,2);
	} else if (this->nProtocol == 3) {
		this->transmit(4,11);
	}
}

/**
 * Sends a "1" Bit
 *                       ___
 * Waveform Protocol 1: |   |_
 *                       __
 * Waveform Protocol 2: |  |_
 */
void RCSwitch::send1() {
	if (this->nProtocol == 1){
		this->transmit(3,1);
	} else if (this->nProtocol == 2) {
		this->transmit(2,1);
	} else if (this->nProtocol == 3) {
		this->transmit(9,6);
	}
}


/**
 * Sends a Tri-State "0" Bit
 *            _     _
 * Waveform: | |___| |___
 */
void RCSwitch::sendT0() {
  this->transmit(1,3);
  this->transmit(1,3);
}

/**
 * Sends a Tri-State "1" Bit
 *            ___   ___
 * Waveform: |   |_|   |_
 */
void RCSwitch::sendT1() {
	this->transmit(3,1);
	this->transmit(3,1);
}

/**
 * Sends a Tri-State "F" Bit
 *            _     ___
 * Waveform: | |___|   |_
 */
void RCSwitch::sendTF() {
	this->transmit(1,3);
	this->transmit(3,1);
}

/**
 * Sends a "Sync" Bit
 *                       _
 * Waveform Protocol 1: | |_______________________________
 *                       _
 * Waveform Protocol 2: | |__________
 */
void RCSwitch::sendSync() {

	if (this->nProtocol == 1){
		this->transmit(1,31);
	} else if (this->nProtocol == 2) {
		this->transmit(1,10);
	} else if (this->nProtocol == 3) {
		this->transmit(1,71);
	}
}

#if not defined( RCSwitchDisableReceiving )
/**
 * Enable receiving data
 */
void RCSwitch::enableReceive(int interrupt) {
	// check if interrupt is already enabled
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		if (interrupt == this->receiverInterrupts[i].interrupt) {
			Serial.printlnf("Interrupt already enabled on pin %d -- skipping", interrupt);
			return;
		}
	}

	Serial.printlnf("Searching for empty spot: interrupt on pin %d", interrupt);
	int index = -1;
	// find an unused element
	// TODO: add a `isEnabled` member to the `InterruptData` struct;
	// reset `nReceivedValue` and `nReceivedBitLength` only if previously not enabled
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		Serial.printlnf("element: %d  |  pin: %d", i, RCSwitch::receiverInterrupts[i].interrupt);
		if (this->receiverInterrupts[i].interrupt == -1) {
			this->receiverInterrupts[i].interrupt = interrupt;
			index = i;
			break;
		}
	}
	Serial.println("");

	attachInterrupt(interrupt, handleInterrupt, CHANGE);

	noInterrupts();
	Serial.printlnf("Attached interrupt: %d", interrupt);
	Serial.printlnf("   interrupt index: %d", index);
	Serial.printlnf("     fetched index: %d", getInterruptIndex(interrupt));
	Serial.println("");
	interrupts();
}

/**
 * Disable receiving data
 */
void RCSwitch::disableReceive(int interrupt) {
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		if (this->receiverInterrupts[i].interrupt == interrupt) {
			detachInterrupt(RCSwitch::receiverInterrupts[i].interrupt);
			this->receiverInterrupts[i].interrupt = -1;
		}
	}
}

void RCSwitch::disableReceive() {
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		detachInterrupt(this->receiverInterrupts[i].interrupt);
		this->receiverInterrupts[i].interrupt = -1;
	}
}

bool RCSwitch::available() {
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		if (this->receiverInterrupts[i].nReceivedValue != NULL) {
			return true;
		}
	}
	return false;
}

void RCSwitch::resetAvailable() {
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		this->receiverInterrupts[i].nReceivedValue = NULL;
	}
}

int RCSwitch::getReceivedPin() {
	return this->nInterruptSourcePin;
}

unsigned long RCSwitch::getReceivedValue(int interrupt) {
	return this->receiverInterrupts[getInterruptIndex(interrupt)].nReceivedValue;
}

unsigned int RCSwitch::getReceivedBitlength(int interrupt) {
	return this->receiverInterrupts[getInterruptIndex(interrupt)].nReceivedBitlength;
}

unsigned int RCSwitch::getReceivedDelay(int interrupt) {
	return this->receiverInterrupts[getInterruptIndex(interrupt)].nReceivedDelay;
}

unsigned int RCSwitch::getReceivedProtocol(int interrupt) {
	return this->receiverInterrupts[getInterruptIndex(interrupt)].nReceivedProtocol;
}

unsigned int* RCSwitch::getReceivedRawdata(int interrupt) {
	return this->timings[getInterruptIndex(interrupt)];
}

/**
 *
 */
bool RCSwitch::receiveProtocol1(int interrupt_i, unsigned int changeCount){
	Serial.println("Triggered: protocol 1");

	unsigned long code = 0;
	unsigned long delay = RCSwitch::timings[interrupt_i][0] / 31;
	unsigned long delayTolerance = delay * RCSwitch::nReceiveTolerance * 0.01;

	for (int i = 1; i<changeCount ; i=i+2) {
		if (	RCSwitch::timings[interrupt_i][i]   > delay   - delayTolerance &&
				RCSwitch::timings[interrupt_i][i]   < delay   + delayTolerance &&
				RCSwitch::timings[interrupt_i][i+1] > delay*3 - delayTolerance &&
				RCSwitch::timings[interrupt_i][i+1] < delay*3 + delayTolerance ) {
			code = code << 1;
		} else if (	RCSwitch::timings[interrupt_i][i]   > delay*3 - delayTolerance &&
					RCSwitch::timings[interrupt_i][i]   < delay*3 + delayTolerance &&
					RCSwitch::timings[interrupt_i][i+1] > delay   - delayTolerance &&
					RCSwitch::timings[interrupt_i][i+1] < delay   + delayTolerance ) {
			code+=1;
			code = code << 1;
		} else {
			// Failed
			i = changeCount;
			code = 0;
		}
	}
	code = code >> 1;

	if (changeCount > 6) {	// ignore < 4bit values as there are no devices sending 4bit values => noise
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedValue = code;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedBitlength = changeCount / 2;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedDelay = delay;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedProtocol = 1;
	}

	if (code == 0) {
		return false;
	} else if (code != 0) {
		nInterruptSourcePin = RCSwitch::receiverInterrupts[interrupt_i].interrupt;
		//for (int j=1; j<changeCount; ++j) {
		//	Serial.printf("%d ", RCSwitch::timings[interrupt_i][j]);
		//}
		//Serial.println("");
		Serial.printf("code = %d\n", code);
		return true;
	}
}

bool RCSwitch::receiveProtocol2(int interrupt_i, unsigned int changeCount){
	Serial.println("Triggered: protocol 2");

	unsigned long code = 0;
	unsigned long delay = RCSwitch::timings[interrupt_i][0] / 10;
	unsigned long delayTolerance = delay * RCSwitch::nReceiveTolerance * 0.01;

	for (int i = 1; i<changeCount ; i=i+2) {
		if (	RCSwitch::timings[interrupt_i][i]   > delay   - delayTolerance &&
				RCSwitch::timings[interrupt_i][i]   < delay   + delayTolerance &&
				RCSwitch::timings[interrupt_i][i+1] > delay*2 - delayTolerance &&
				RCSwitch::timings[interrupt_i][i+1] < delay*2 + delayTolerance ) {
			code = code << 1;
		} else if (	RCSwitch::timings[interrupt_i][i]   > delay*2 - delayTolerance &&
					RCSwitch::timings[interrupt_i][i]   < delay*2 + delayTolerance &&
					RCSwitch::timings[interrupt_i][i+1] > delay   - delayTolerance &&
					RCSwitch::timings[interrupt_i][i+1] < delay   + delayTolerance ) {
			code+=1;
			code = code << 1;
		} else {
			// Failed
			i = changeCount;
			code = 0;
		}
	}
	code = code >> 1;

	if (changeCount > 6) {	// ignore < 4bit values as there are no devices sending 4bit values => noise
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedValue = code;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedBitlength = changeCount / 2;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedDelay = delay;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedProtocol = 2;
	}

	if (code == 0) {
		return false;
	} else if (code != 0) {
		nInterruptSourcePin = RCSwitch::receiverInterrupts[interrupt_i].interrupt;
		//for (int j=1; j<changeCount; ++j) {
		//	Serial.printf("%d ", RCSwitch::timings[interrupt_i][j]);
		//}
		//Serial.println("");
		Serial.printf("code = %d\n", code);
		return true;
	}
}

/** Protocol 3 is used by BL35P02.
 *
 */
bool RCSwitch::receiveProtocol3(int interrupt_i, unsigned int changeCount){
	Serial.println("Triggered: protocol 3");

	unsigned long code = 0;
	unsigned long delay = RCSwitch::timings[interrupt_i][0] / PROTOCOL3_SYNC_FACTOR;
	unsigned long delayTolerance = delay * RCSwitch::nReceiveTolerance * 0.01;

	for (int i = 1; i<changeCount ; i=i+2) {
		if (	RCSwitch::timings[interrupt_i][i]   > delay*PROTOCOL3_0_HIGH_CYCLES - delayTolerance &&
				RCSwitch::timings[interrupt_i][i]   < delay*PROTOCOL3_0_HIGH_CYCLES + delayTolerance &&
				RCSwitch::timings[interrupt_i][i+1] > delay*PROTOCOL3_0_LOW_CYCLES  - delayTolerance &&
				RCSwitch::timings[interrupt_i][i+1] < delay*PROTOCOL3_0_LOW_CYCLES  + delayTolerance ) {
			code = code << 1;
		} else if (	RCSwitch::timings[interrupt_i][i]   > delay*PROTOCOL3_1_HIGH_CYCLES - delayTolerance &&
					RCSwitch::timings[interrupt_i][i]   < delay*PROTOCOL3_1_HIGH_CYCLES + delayTolerance &&
					RCSwitch::timings[interrupt_i][i+1] > delay*PROTOCOL3_1_LOW_CYCLES  - delayTolerance &&
					RCSwitch::timings[interrupt_i][i+1] < delay*PROTOCOL3_1_LOW_CYCLES  + delayTolerance ) {
			code+=1;
			code = code << 1;
		} else {
			// Failed
			i = changeCount;
			code = 0;
		}
	}
	code = code >> 1;

	if (changeCount > 6) {	// ignore < 4bit values as there are no devices sending 4bit values => noise
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedValue = code;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedBitlength = changeCount / 2;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedDelay = delay;
		RCSwitch::receiverInterrupts[interrupt_i].nReceivedProtocol = 3;
	}

	if (code == 0) {
		return false;
	} else if (code != 0) {
		nInterruptSourcePin = RCSwitch::receiverInterrupts[interrupt_i].interrupt;
		//for (int j=1; j<changeCount; ++j) {
		//	Serial.printf("%d ", RCSwitch::timings[interrupt_i][j]);
		//}
		//Serial.println("");
		Serial.printf("code = %d\n", code);
		return true;
	}
}

void RCSwitch::handleInterrupt() {
	bool doSwap = false;
	unsigned long time = micros() * 0.983; // Spark Core micros() calibration

	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		bool isCurrentHigh = (digitalRead(RCSwitch::receiverInterrupts[i].interrupt) == HIGH);
		bool isCurrentLow  = (digitalRead(RCSwitch::receiverInterrupts[i].interrupt) == LOW);
		bool isPinPrevHigh_tmp = RCSwitch::receiverInterrupts[i].isPinPrevHigh;

		if ( (isCurrentHigh && isPinPrevHigh_tmp) || (isCurrentLow && !isPinPrevHigh_tmp) ) {
			continue;
		}

		RCSwitch::receiverInterrupts[i].duration = time - RCSwitch::receiverInterrupts[i].lastTime;
		int duration_tmp = RCSwitch::receiverInterrupts[i].duration;
		if (duration_tmp > 4600 && duration_tmp > RCSwitch::timings[i][0] - 200 && duration_tmp < RCSwitch::timings[i][0] + 200) {
			RCSwitch::receiverInterrupts[i].repeatCount++;
			RCSwitch::receiverInterrupts[i].changeCount--;

			if (RCSwitch::receiverInterrupts[i].repeatCount == 2) {
				int sendInterruptIndex = getInterruptIndex(RCSwitch::receiverInterrupts[i].interrupt);
				int sendChangeCount = RCSwitch::receiverInterrupts[i].changeCount;
				if (receiveProtocol1(sendInterruptIndex, sendChangeCount) == false) {
					if (receiveProtocol2(sendInterruptIndex, sendChangeCount) == false) {
						if (receiveProtocol3(sendInterruptIndex, sendChangeCount) == false) {
							// failed
						}
					}
				}
				RCSwitch::receiverInterrupts[i].repeatCount = 0;
			}
			RCSwitch::receiverInterrupts[i].changeCount = 0;
		} else if (duration_tmp > 4600) {
			RCSwitch::receiverInterrupts[i].changeCount = 0;
		}

		if (RCSwitch::receiverInterrupts[i].changeCount >= RCSWITCH_MAX_CHANGES) {
			RCSwitch::receiverInterrupts[i].changeCount = 0;
			RCSwitch::receiverInterrupts[i].repeatCount = 0;

			//Serial.printf("dump(%d) -- ", i);
			//for (int j=1; j<RCSWITCH_MAX_CHANGES; ++j) {
			//	Serial.printf("%d ", RCSwitch::timings[i][j]);
			//}
			//Serial.println("");
			doSwap = true;
		}

		RCSwitch::timings[i][RCSwitch::receiverInterrupts[i].changeCount++] = RCSwitch::receiverInterrupts[i].duration;
		RCSwitch::receiverInterrupts[i].lastTime = time;
		RCSwitch::receiverInterrupts[i].isPinPrevHigh = (digitalRead(RCSwitch::receiverInterrupts[i].interrupt) == HIGH);
	}
}

int RCSwitch::getInterruptIndex(int interrupt) {
	for (int i = 0; i < RCSWITCH_MAX_RX_PINS; ++i) {
		if (interrupt == RCSwitch::receiverInterrupts[i].interrupt) {
			return i;
		}
	}
	return -1;
}

/**
 * Turns a decimal value to its binary representation
 */
char* RCSwitch::dec2binWzerofill(unsigned long Dec, unsigned int bitLength) {
	return dec2binWcharfill(Dec, bitLength, '0');
}

char* RCSwitch::dec2binWcharfill(unsigned long Dec, unsigned int bitLength, char fill) {
	static char bin[64];
	unsigned int i=0;

	while (Dec > 0) {
		bin[32+i++] = ((Dec & 1) > 0) ? '1' : fill;
		Dec = Dec >> 1;
	}

	for (unsigned int j = 0; j< bitLength; j++) {
		if (j >= bitLength - i) {
			bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
		} else {
			bin[j] = fill;
		}
	}
	bin[bitLength] = '\0';

	return bin;
}

#endif
