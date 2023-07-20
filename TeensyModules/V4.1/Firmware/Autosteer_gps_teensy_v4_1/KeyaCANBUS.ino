
// KeyaCANBUS
// Trying to get Keya to steer the tractor over CANBUS

#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

//Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
//Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
//Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
//Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
//Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
//Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)

uint8_t KeyaSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0,0,0,0 }; // save us populating the first 4 bytes each time
// this is just a reminder
uint8_t KeyaHeartbeat[] = { 0, 0, 0, 0, 0, 0, 0, 0, };
uint8_t keyaCurrent[] = { 0x60, 0x12, 0x21, 0x01 };

uint64_t KeyaPGN = 0x06000001;

boolean intendToSteer = 0;
boolean engageCAN = 0;

void CAN_Setup() {
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000);
	// Dedicated bus, zero chat from others. No need for filters
	// claim an address
	CAN_message_t msgV;
	msgV.id = KeyaPGN;
	msgV.flags.extended = true;
	msgV.len = 8;
	msgV.buf[0] = 0x00;
	msgV.buf[1] = 0x00;
	msgV.buf[2] = 0xC0;
	msgV.buf[3] = 0x0C;
	msgV.buf[4] = 0x00;
	msgV.buf[5] = 0x17;
	msgV.buf[6] = 0x02;
	msgV.buf[7] = 0x20;
	Keya_Bus.write(msgV);
	delay(1000); // take a breath
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
	return memcmp(message.buf, pattern, patternSize) == 0;
}

void Disable_Steer() {
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x0c;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	KeyaBusSendData.buf[4] = 0;
	KeyaBusSendData.buf[5] = 0;
	KeyaBusSendData.buf[6] = 0;
	KeyaBusSendData.buf[7] = 0;
	Keya_Bus.write(KeyaBusSendData);
}
void Enable_Steer() {
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x0d;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	KeyaBusSendData.buf[4] = 0;
	KeyaBusSendData.buf[5] = 0;
	KeyaBusSendData.buf[6] = 0;
	KeyaBusSendData.buf[7] = 0;
	Keya_Bus.write(KeyaBusSendData);
}

void KeyaBus_SendSteer(char KeyaDirection) {
	if (intendToSteer == 0 || !engageCAN) {
		Disable_Steer();
		return; // don't need to go any further, if we're disabling, we're disabling
	}
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x00;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	// TODO Hardcoded relatively slow speed, let's not kill ourselves to start. This should be from PWM tho
	// Max speed is -1999 to 1999 tho
	// speed is related to direction too - clockwise, speed should be negative, anti-clock, should be positive
	// TODO placeholder till you work out where direction comes from
	if (KeyaDirection == 'L') {
		KeyaBusSendData.buf[4] = 0x01;
		KeyaBusSendData.buf[5] = 0xF4;
		KeyaBusSendData.buf[6] = 0;
		KeyaBusSendData.buf[7] = 0;
	}
	else {
		KeyaBusSendData.buf[4] = 0xFE;
		KeyaBusSendData.buf[5] = 0x0C;
		KeyaBusSendData.buf[6] = 0xFF;
		KeyaBusSendData.buf[7] = 0xFF;
	}
	Keya_Bus.write(KeyaBusSendData);
	Enable_Steer(); // TODO this might be a little back to front, let's see. In SavvyCAN, this appeared to play the "current" steer command
}

void KeyaBus_Receive() {
	CAN_message_t KeyaBusReceiveData;
	if (Keya_Bus.read(KeyaBusReceiveData)) {
		// parse the different message types
		// heartbeat 0x07000001
		if (KeyaBusReceiveData.id == 0x07000001) {
			// 0-1 - Cumulative value of angle (360 def / circle)
			// 2-3 - Motor speed, signed int eg -500 or 500
			// 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
			//		is that accurate enough for us?
			// 6-7 - Control_Close (error code)
			// TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
		}

		// response from most commands 0x05800001
		// could have been separate PGNs, but oh no...

		if (KeyaBusReceiveData.id == 0x05800001) {
			// response to current request (this is also in heartbeat)
			if (isPatternMatch(KeyaBusReceiveData, keyaCurrent, sizeof(keyaCurrent))) {
				// Current is unsigned float in [4]
				// set the motor current variable, when you find out what that is
			}
			else if (1) {
			}
		}
	}
}
