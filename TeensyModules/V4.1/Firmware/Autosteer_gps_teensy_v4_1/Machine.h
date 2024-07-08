#ifndef _MACHINE_H
#define _MACHINE_H

#include <Arduino.h>

extern uint8_t aogSections[8];			// the sections as sent from AOG
extern uint8_t machineSections[8];		// the sections as mainained by Teensy and sync'd with machine
extern uint8_t helloFromMachine[11];

void printByteInBinary(byte b);
void printArrayInBinary(byte arr[], int size);

void MachineCANBUS_Setup();				// sets up the CANBUS for the machine

void pollMachineForSections();			// responds to requests, or polls for status, from machine. Updates machineSections
void SendMachineSectionCommand();		// sends the machineSections to the machine

void ReceiveSectionDataFromAOG();		// receives the aogSections from AOG and updates machineSections
void UpdateAOGSectionData();			// updates the aogSections from machineSections and sends back to AOG

#endif
