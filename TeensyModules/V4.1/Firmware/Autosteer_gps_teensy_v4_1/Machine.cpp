#include "Machine.h"
#include <FlexCAN_T4.h>

uint8_t aogSections[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t machineSections[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t helloFromMachine[11] = { 0x80, 0x81, 0x7b, 0x7b, 5, 0, 0, 0, 0, 0, 71 };
byte AmaClick_addressClaim[8] = { 0x28, 0xEC, 0x44, 0x0C, 0x00, 0x80, 0x1A, 0x20 };
CAN_message_t MachineBusSendData;
const bool debugMachine = true;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Machine_Bus;

void printByteInBinary(byte b) {
	for (int i = 7; i >= 0; i--) {
		if (b & (1 << i)) {
			Serial.print("1");
		}
		else {
			Serial.print("0");
		}
	}
	Serial.print(" ");
}

void printArrayInBinary(byte arr[], int size) {
	for (int i = 0; i < size; i++) {
		printByteInBinary(arr[i]);
	}
	Serial.println();
}

void MachineBusSend(uint8_t data[8]) {
	memcpy(MachineBusSendData.buf, data, 8);
	Machine_Bus.write(MachineBusSendData);
}

void MachineCANBUS_Setup()
{

	Serial.println("In Machine CAN-Setup");
	Machine_Bus.begin();
	Machine_Bus.setBaudRate(250000);
	delay(1000);
	MachineBusSendData.id = 0x18EEFFCE;
	MachineBusSendData.flags.extended = true;
	MachineBusSendData.len = 8;
	MachineBusSend(AmaClick_addressClaim);
	if (debugMachine) Serial.println("Initialised Machine CANBUS");
	delay(2000);
}

void UpdateTeensySections()
{
}

void ReceiveSectionDataFromAOG()
{
	Serial.println("got update from AOG");
	MachineBusSend(aogSections);
}

void UpdateAOGSectionData()
{
}

void pollMachineForSections()
{
}

void SendMachineSectionCommand()
{
}

