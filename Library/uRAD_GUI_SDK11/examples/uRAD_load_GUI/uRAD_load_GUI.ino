#include <uRAD_GUI_SDK11.h>	// include the library

#define pin_ON_OFF 6
#define ACK 170
#define NACK 204
#define command_loadConfiguration 14
#define command_detection 15
#define command_turnON 16
#define command_turnOFF 17

uRAD_GUI_SDK11 uRAD;       	// create the object uRAD

// variables
uint8_t i, command;
int return_code;

void setup() {

	Serial.begin(500000);	// serial port baud rate
	digitalWrite(pin_ON_OFF, HIGH);	 // switch ON uRAD communication

}

void loop() {

	while (Serial.available() <= 0) {
		i = 0;
	}
	command = uint8_t(Serial.read());
	if (command == command_turnON) {
		return_code = uRAD.turnON();
		if (return_code == 0) {
			Serial.write(ACK);
		}
		else {
			Serial.write(NACK);
		}
	}
	else if (command == command_turnOFF) {
		return_code = uRAD.turnOFF();
		if (return_code == 0) {
			Serial.write(ACK);
		}
		else {
			Serial.write(NACK);
		}
	}
	else if (command == command_loadConfiguration) {
		return_code = uRAD.loadConfiguration_GUI();
		if (return_code == 0) {
			Serial.write(ACK);
		}
		else {
			Serial.write(NACK);
		}
	}
	else if (command == command_detection) {
		uRAD.detection_GUI();
	}
	
}
