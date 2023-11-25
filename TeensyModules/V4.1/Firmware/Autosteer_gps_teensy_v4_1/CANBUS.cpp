// OK, this is a template class for CANBUS
// Until we get proper CANBUS support in AOG, this will have to do
// You'll need to check your board version and use the correct #define in the FlexCAN_T4 line

#include "CANBUS.h"

#define AIO2x_CAN1 CAN3 // AIO 2.x version board, pins 16 and 17, use this
#define AIO2x_CAN2 CAN2 // AIO 2.x version board, pins 18 and 19, use this
#define AIO4x_CAN1 CAN3 // AIO 4.x version board, pins 16 and 17, use this
FlexCAN_T4<AIO4x_CAN1, RX_SIZE_256, TX_SIZE_256> CANBUS;

const int8_t filterID = 0;
const int8_t byteOffset = 1;
const int8_t ANDValue = 2;

// Then, because these are machine specific, you'll need to set up the filters
// pick from this list according to your machine, and uncomment the one you need

// CaseNH
int CANInfo[3] = { 0x18FFB306 , 2, 0x10}; // Case Puma CVX 160 2015, button behind joystick
//int CANInfo[3] = { 0x18FFB031 , 2, 0x10}; // Case Puma CVX 160 2015, steer button on armrest
//int CANInfo[3] = { 0x98FFB306 , 2, 0x10}; // Case Puma CVX 200 2015, button behind joystick
//int CANInfo[3] = { 0x98FFB031 , 2, 0x10}; // Case Puma CVX 200 2015, steer button on armrest
//int CANInfo[3] = { 0x14FF7706 , 0, 0x82}; // Case Puma CVX 165 2022, steer button on armrest (note, this is full byte, not bit)
//int CANInfo[3] = { 0x14FF7706 , 0, 0xB2}; // Case Puma CVX 165 2022, steer button on armrest (note, this is full byte, not bit)
//int CANInfo[3] = { 0x14FF7706 , 2, 0xC1}; // Case Puma CVX 165 2022, steer button on armrest (note, this is full byte, not bit)
//int CANInfo[3] = { 0x14FF7706 , 2, 0xC4}; // Case Puma CVX 165 2022, steer button on armrest (note, this is full byte, not bit)

// Massey Ferguson
//int CANInfo[3] = { 0x45a , 1, 0x04}; // MF headland management button
//int CANInfo[3] = { 0xCFF2621 , 3, 0x04}; // MF S-series steering engage


// Codes below here need validated and checked !!!

// Fendt
//int CANInfo[3] = { 0x613 , 1, 0x04}; // Fendt arm rest button
//int CANInfo[3] = { 0xCFFD899 , 3, 0xF6}; // FendtOne engage // not a bit state

// Claas
//int CANInfo[3] = { 0x18EF1CD2 , 2, 0xC4}; // Claas engage

// Valtra
//int CANInfo[3] = { 0x18EF1C32 , 2, 0x01}; // Valtra engage  // code in firmware is unclear for bit states here
//int CANInfo[3] = { 0x18EF1C00 , 2, 0xC4}; // Valtra engage (alternative)

// JCB
//int CANInfo[3] = { 0x18EFAB27 , 2, 0x01}; // code in firmware is unclear for bit states here

// Caterpillar MT
//int CANInfo[3] = { 0x18EF1CF0 , 0, 0x0F}; //code unsure here, could be && [1] == 0x60 as well




bool debugCANBUS = true;

void CAN_Setup() {
	// we're only listening, so no need to claim an address?
	CANBUS.begin();
	CANBUS.setBaudRate(250000, LISTEN_ONLY);
	delay(1000);
	if (CANInfo[filterID] > 0xffff) {
		CANBUS.setFIFOFilter(0, CANInfo[filterID], EXT);
	}
	else {
		CANBUS.setFIFOFilter(0, CANInfo[filterID], STD);
	}
	if (debugCANBUS) Serial.println("Initialised CANBUS");
}

void CANBUS_Receive() {

}