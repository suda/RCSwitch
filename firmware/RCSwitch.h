/* ======================= .h =============================== */

#ifndef _RCSwitch_h
#define _RCSwitch_h


// https://community.sparkdevices.com/t/fix-for-include-arduino-h/953

#define ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include "spark_wiring.h"
#include "spark_wiring_interrupts.h"

// to make it compile (by Frido)

#define boolean bool
#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wconversion-null"
#pragma GCC diagnostic ignored "-Wpointer-arith"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wreturn-type"

// Maximum number of supported RX pins.
// Setting this to 0 will #define RCSwitchDisableReceiving
#define RCSWITCH_MAX_RX_PINS 1

// Number of maximum High/Low changes per packet.
// We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
#define RCSWITCH_MAX_CHANGES 67

#define PROTOCOL3_SYNC_FACTOR   71
#define PROTOCOL3_0_HIGH_CYCLES  4
#define PROTOCOL3_0_LOW_CYCLES  11
#define PROTOCOL3_1_HIGH_CYCLES  9
#define PROTOCOL3_1_LOW_CYCLES   6

// #define RCSwitchDisableReceiving if there are no supported RX pins.
#ifdef RCSWITCH_MAX_RX_PINS
	#if (RCSWITCH_MAX_RX_PINS == 0)
	// Two checks needed: preprocessor replaces undefined symbols with 0.
	#define RCSwitchDisableReceiving
	#endif
#endif

class RCSwitch {

public:
	RCSwitch();

	void switchOn(int nGroupNumber, int nSwitchNumber);
	void switchOff(int nGroupNumber, int nSwitchNumber);
	void switchOn(char* sGroup, int nSwitchNumber);
	void switchOff(char* sGroup, int nSwitchNumber);
	void switchOn(char sFamily, int nGroup, int nDevice);
	void switchOff(char sFamily, int nGroup, int nDevice);
	void switchOn(char* sGroup, char* sDevice);
	void switchOff(char* sGroup, char* sDevice);
	void switchOn(char sGroup, int nDevice);
	void switchOff(char sGroup, int nDevice);

	void sendTriState(char* Code);
	void send(unsigned long Code, unsigned int length);
	void send(char* Code);

	#if not defined( RCSwitchDisableReceiving )
	void enableReceive(int interrupt);
	// Enabling "all" is only useful if InterruptData.isDisabled exists.
	//void enableReceive();
	void disableReceive(int interrupt);
	void disableReceive();
	bool available();
	void resetAvailable();

	int getReceivedPin();
	unsigned long getReceivedValue(int interrupt);
	unsigned int getReceivedBitlength(int interrupt);
	unsigned int getReceivedDelay(int interrupt);
	unsigned int getReceivedProtocol(int interrupt);
	unsigned int* getReceivedRawdata(int interrupt);
	#endif

	void enableTransmit(int nTransmitterPin);
	void disableTransmit();
	void setPulseLength(int nPulseLength);
	void setRepeatTransmit(int nRepeatTransmit);
	#if not defined( RCSwitchDisableReceiving )
	void setReceiveTolerance(int nPercent);
	#endif
	void setProtocol(int nProtocol);
	void setProtocol(int nProtocol, int nPulseLength);

	char* dec2binWzerofill(unsigned long dec, unsigned int length);
	char* dec2binWcharfill(unsigned long dec, unsigned int length, char fill);

private:
	char* getCodeWordB(int nGroupNumber, int nSwitchNumber, boolean bStatus);
	char* getCodeWordA(char* sGroup, int nSwitchNumber, boolean bStatus);
	char* getCodeWordA(char* sGroup, char* sDevice, boolean bStatus);
	char* getCodeWordC(char sFamily, int nGroup, int nDevice, boolean bStatus);
	char* getCodeWordD(char group, int nDevice, boolean bStatus);
	void sendT0();
	void sendT1();
	void sendTF();
	void send0();
	void send1();
	void sendSync();
	void transmit(int nHighPulses, int nLowPulses);

	#if not defined ( RCSwitchDisableReceiving )
	static void handleInterrupt();
	static bool receiveProtocol1(int interrupt_i, unsigned int changeCount);
	static bool receiveProtocol2(int interrupt_i, unsigned int changeCount);
	static bool receiveProtocol3(int interrupt_i, unsigned int changeCount);
	static int getInterruptIndex(int interrupt);

	// TODO: Possibly switch to using an explicit `bool is_disabled` (as opposed to -1)
	// TODO: Possibly implement an interrupt-specific "isAvailable"? Such that the user
	//		 can loop through all interrupts they are interested in.
	struct InterruptData {
		int interrupt;
		//bool isDisabled;
		//bool isAvailable;

		unsigned long nReceivedValue;
		unsigned int nReceivedBitlength;
		unsigned int nReceivedDelay;
		unsigned int nReceivedProtocol;

		unsigned long lastTime;
		unsigned int duration;
		unsigned int changeCount, repeatCount;
		bool isPinPrevHigh;

		InterruptData() {
			interrupt = -1;
			//isDisabled = true;
			//isAvailable = false;

			nReceivedValue = NULL;
			nReceivedBitlength = 0;
			nReceivedDelay = 0;
			nReceivedProtocol = 0;

			lastTime = 0;
			duration = 0;
			changeCount = 0;
			repeatCount = 0;
			isPinPrevHigh = false;
		}
	};

	static InterruptData receiverInterrupts[RCSWITCH_MAX_RX_PINS];
	static int nInterruptSourcePin;
	#endif

	int nTransmitterPin;
	int nPulseLength;
	int nRepeatTransmit;
	char nProtocol;
	#if not defined( RCSwitchDisableReceiving )
	static int nReceiveTolerance;
	#endif

	/*
	 * timings[0] contains sync timing, followed by a number of bits
	 */
	static unsigned int timings[RCSWITCH_MAX_RX_PINS][RCSWITCH_MAX_CHANGES];
};

#endif
