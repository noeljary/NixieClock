#include <Wire.h>
#include <String.h>

#define DS1307_ID 0x68

#define HRS_EXP 0x22
#define MIN_EXP 0x20
#define SEC_EXP 0x21

#define SECONDS_FROM_1970_TO_2000 946684800
#define SECONDS_PER_DAY           86400L

//Pin Definitions
#define CLOCK_PIN  2
#define BRIGHT_PIN 9
#define LED_PIN    10
#define IR_IN_PIN  13
#define IR_OUT_PIN 14

// Store Exact Current Timestamp
byte timestamp[7];

// Update Clock Signal
boolean clockAdv = 0;

// Set LED Toggle Status
boolean ledToggle = 0;

// Nixie Tube Brightness Values
byte bright = 255;
boolean autoBright = 0;

// Serial Command Receive Controls
boolean sOL = 0, eOL = 0, serialUnlocked = 1;
char cmd;
char cmd_arg[25];
byte rx_idx;

void setup() {
	Wire.begin();                                  // Start I2C Connection
	Serial.begin(9600);                            // Start Serial RX/TX Connection

	analogReference(EXTERNAL);                     // Analog Inputs Referenced to 3V3

	i2cWrite(HRS_EXP, 0x00, 1, 0x00);              // Enable Hours Outputs
	i2cWrite(MIN_EXP, 0x00, 1, 0x00);              // Enable Minutes Outputs
	i2cWrite(SEC_EXP, 0x00, 1, 0x00);              // Enable Seconds Outputs
	i2cWrite(DS1307_ID, 0x07, 1, (byte[]) {0x10}); // Enable Clock Timepulse
	
	pinMode(CLOCK_PIN, INPUT);                     // Setup Interrupt Pin
	attachInterrupt(0, clockInterrupt, FALLING);   // Attach Interrupt Function

	pinMode(BRIGHT_PIN, OUTPUT);                   // Setup Brightness Pin
	analogWrite(BRIGHT_PIN, bright);               // Start with Full Brightness
	
	pinMode(LED_PIN, OUTPUT);                      // Setup LED Pin
}

void loop() {
	// If Auto Brightness enabled in EEPROM or runtime
	if(autoBright) {
		analogWrite(BRIGHT_PIN, (bright = map(constrain(analogRead(A0), 300, 850), 300, 850, 25, 255)));
	}

	// If 1Hz signal received from DS1307
	if(clockAdv) {
		// Update Tubes with Time
		updateClock();

		// Toggle LED ON/OFF
		ledToggle = !ledToggle;
		digitalWrite(LED_PIN, ledToggle);

		clockAdv = 0;
	}

	// If Serial Command Received (Not from WiFly)
	if(sOL && eOL) {
		processSerialCommand();
	}
}

// Declare Software Reset Function
void(* resetClock) (void) = 0;

void serialEvent() {
	// Only receive if not already processing request
	if(serialUnlocked) {
		while(Serial.available ()) {
			char rx_char = Serial.read();

			// Receive Serial Commands in {} notation 
			if(rx_char == '{') {
				sOL = 1;
				continue;
			} else if(rx_char == '}') {
				cmd_arg[rx_idx] = '\0';
				Serial.flush();
				eOL = 1;
				break;
			}

			if(sOL && !eOL) {
				if(cmd == '\0') {
					cmd = rx_char;
				} else {
					cmd_arg[rx_idx++] = rx_char;
					cmd_arg[rx_idx]   = '\0';
				}
			}
		}
	}
}



void processSerialCommand() {
	// Block Serial Event Method
	serialUnlocked = 0;

	returnSerialResp("ACK", cmd_arg);
	delay(50);

	switch(cmd) {
		case 'c':
			// Cycle Cathodes to avoid poisoning through unuse
			antiCathodePoison(atoi(cmd_arg));
		break;

		case 'h':
			// Manually set Hours Digits
			updateDigitPair(HRS_EXP, atoi(cmd_arg));
		break;

		case 'l':
			// Enable/Disable AutoBrightness & Set Levels
			char resp[8];
			byte old_bright;
			old_bright = bright;
			bright = constrain(atoi(cmd_arg), 0, 255);

			if(bright == 0) {
				enableAutoBright();
			} else {
				disableAutoBright(bright);
				sprintf(resp, "%d|%d", (byte) ((old_bright / 255.0) * 100), (byte) ((bright / 255.0) * 100));
			}

			returnSerialResp("RSP", resp);
		break;

		case 'm':
			// Manually set Minutes Digits
			updateDigitPair(MIN_EXP, atoi(cmd_arg));
		break;

		case 's':
			// Manually set Seconds Digits
			updateDigitPair(SEC_EXP, atoi(cmd_arg));
		break;

		case 't':
			// Force Time Update
			if(rx_idx > 0) {
				// Update from Computer Value
				setTime(atol(cmd_arg));
				returnSerialResp("RSP", cmd_arg);
			} else {
				// Update from WiFly NTP
				unsigned long ts = setTime();
				char resp[12];
				sprintf(resp, "%lu", ts);
				if(ts > 0) {
					returnSerialResp("RSP", resp);
				} else {
					returnSerialResp("RSP", "ERR");
				}
			}
		break; 
	}

	// Enable Serial Event Method
	returnSerialResp("AOK", cmd_arg);

	// Clear all Serial Data as now processed
	sOL = 0;
	eOL = 0;
	rx_idx = 0;
	cmd = '\0';
	serialUnlocked = 1;
}

void returnSerialResp(char hdr[4], char *val) {
	Serial.print("{");
	Serial.print(hdr);
	Serial.print("-");
	Serial.print(cmd);
	if(strcmp(val, "") != 0) {
		Serial.print("-");
		Serial.print(val);
	}
	Serial.print("}\n");
}

void clockInterrupt() {
	// Signal main loop to advance clock from interrupt
	clockAdv = true;
}

boolean connectWiFly() {
	Serial.print("$$$");
	return Serial.find("CMD");
}

boolean disconnectWiFly() {
	Serial.print("exit\r");
	return Serial.find("EXIT");
}

unsigned long setTime() {
	unsigned long ts = 0;

	// Force NTP update on WiFly
	if(connectWiFly()) {
		Serial.print("time\r");
		if(Serial.find("time")) {
			char line[20];
			byte line_pos = 0;

			Serial.flush();
			Serial.print("show t t\r");

			delay(150);

			while(Serial.available()) {
				line[line_pos++] = Serial.read();
				line[line_pos]   = '\0';

				if(line[line_pos - 1] == '\n') {
					if(line_pos > 2 && line[0] == 'R' && line[1] == 'T' && line[2] == 'C') {
						break;
					} else {
						line[0]  = '\0';
						line_pos = 0;
					}
				}
			}

			sscanf(line, "RTC=%lu", &ts);
		}

		if(!disconnectWiFly()) {
			resetClock();
		}
	}


	// Call overloaded setTime function with timestamp
	if(ts > 0) {
		setTime(ts);
	}

	return ts;
}

void setTime(unsigned long t) {
	// Set DS1307 with time data
	int yOff, m, d, hh, mm, ss, days, leap;
	const byte daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	// Get Time Since 2000
	t -= SECONDS_FROM_1970_TO_2000;

	// Calculate Seconds, Minutes, Hours into Day
	ss = t % 60;
	t /= 60;
	mm = t % 60;
	t /= 60;
	hh = t % 24;

	// Calculate Number of Days Since 2000
	days = t / 24;

	// Breakdown number of Years accounting for Leaps
	for(yOff = 0; ; ++yOff) {
		leap = yOff % 4 == 0;
		if(days < 365 + leap) {
			 break; 
		}
		days -= 365 + leap;
	}

	// Breakdown for Months
	for(m = 1; ; ++m) {
		 byte daysPerMonth = daysInMonth[m - 1];

		 if(leap && m == 2) {
			 ++daysPerMonth;
		 }

		 if(days < daysPerMonth) {
			 break;
		 }

		 days -= daysPerMonth;
	}

	d = days + 1;

	// Begin Update DS1307
	stopClock();

	byte timeData[] = {(dec2bcd(ss) | 0x80), dec2bcd(mm), (dec2bcd(hh) & 0x3f), dec2bcd(0), dec2bcd(d), dec2bcd(m), dec2bcd(yOff)};
	i2cWrite(DS1307_ID, 0x00, 7, timeData);

	startClock();
	// End Update DS1307
}

void stopClock() {
	byte ss[1];
	i2cRead(DS1307_ID, 0x00, 1, ss);
	i2cWrite(DS1307_ID, 0x00, 1, (byte []) {ss[0] | 0x80});
}

void startClock() {
	byte ss[1];
	i2cRead(DS1307_ID, 0x00, 1, ss);
	i2cWrite(DS1307_ID, 0x00, 1, (byte []) {ss[0] & 0x7f});
}

void updateClock() {
	i2cRead(DS1307_ID, 0x00, 7, timestamp);                     // Request Timestamp
	updateDigitPair(SEC_EXP, bcd2dec(timestamp[0] & 0x7f));     // Set Seconds
	updateDigitPair(MIN_EXP, bcd2dec(timestamp[1]));            // Set Minutes
	updateDigitPair(HRS_EXP, bcd2dec(timestamp[2]));            // Set Hours
}

void updateDigitPair(byte addr, byte timeVal) {
	byte digitPair[] = {(dec2bcd(numConvert(timeVal % 10)) << 4) + dec2bcd(numConvert((byte) timeVal / 10))};
	i2cWrite(addr, 0x09, 1,digitPair);
}

void antiCathodePoison(int count) {
	for(int i = 0; i < count; i++) {
		for(byte j = 0; j < 10; j++) {
			delay(75);
			byte digitPair[] = {(dec2bcd(numConvert(j)) << 4) + dec2bcd(numConvert(j))};
			i2cWrite(HRS_EXP, 0x09, 1, digitPair);
			i2cWrite(MIN_EXP, 0x09, 1, digitPair);
			i2cWrite(SEC_EXP, 0x09, 1, digitPair);
		}
	}
}

byte numConvert(byte in) {
	// Pin Mapping Function for incorrect routing
	return (11 - in) % 10;
}

boolean enableAutoBright() {
	autoBright = 1;
}

boolean disableAutoBright(byte bright) {
	autoBright = 0;
	analogWrite(BRIGHT_PIN, bright);
}

void i2cWrite(byte addr, byte reg, byte num, byte val[]) {
	Wire.beginTransmission(addr);
	Wire.write(reg);

	for(byte i = 0; i < num; i++) {
		Wire.write(val[i]); 
	}

	Wire.endTransmission();
}

void i2cRead(byte addr, byte reg, byte num, byte res[]) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(addr, num);

	for(byte i = 0; i < num; i++) {
		 res[i] = Wire.read();
	}
}

void eepromWrite(byte deviceaddress, unsigned int eeaddress, byte data) {
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(data);
    Wire.endTransmission();
    delay(5);
}

byte eepromRead(byte deviceaddress, int eeaddress) {
    byte rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int) (eeaddress >> 8)); // MSB
    Wire.write((int) (eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom((int) deviceaddress, 1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}

uint8_t dec2bcd(uint8_t num) {
	return ((num / 10 * 16) + (num % 10));
}

uint8_t bcd2dec(uint8_t num) {
	return ((num / 16 * 10) + (num % 16));
}
