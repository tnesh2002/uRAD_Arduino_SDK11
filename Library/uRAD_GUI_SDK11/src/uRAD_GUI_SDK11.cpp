/*
 * uRAD_GUI_SDK11.h - Library for controlling uRAD SHIELD with the Grafical User Interface
 * Created by Victor Torres, Diego Gaston - uRAD 2020
*/

#include "Arduino.h"
#include "uRAD_GUI_SDK11.h"
#include "SPI.h"

#define ACK 170
#define pin_ON_OFF 6
#define pin_SS 7
#define command_loadConfiguration 14
#define command_detection 15
#define command_turnON 16
#define command_turnOFF 17
#define command_ready 18
#define command_I 20
#define command_Q 21


uRAD_GUI_SDK11::uRAD_GUI_SDK11() {

	pinMode(pin_ON_OFF, OUTPUT);  	 // ON/OFF pin
	digitalWrite(pin_ON_OFF, LOW);	 // switch OFF uRAD 
	pinMode(pin_SS, OUTPUT);  		 // Slave Select pin SPI

}

int uRAD_GUI_SDK11::loadConfiguration_GUI(void) {

	// Variables
	uint8_t i, rx_ACK;
	uint16_t read_timeout = 200;
	uint64_t t0, ti;
	
	_bytesDisponibles = 0;
	while (_bytesDisponibles < 8) {
		_bytesDisponibles = Serial.available();
	}
	for (i = 0; i < 8; i++) {
		_configuration[i] = Serial.read();
	}
	
	// SPI configuration
	digitalWrite(pin_SS, HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
	
	//Send configuration by SPI
	rx_ACK = 0;
	t0 = millis();
	ti = millis();
	while ((rx_ACK != ACK) && ((ti - t0) < read_timeout)) {
		digitalWrite(pin_SS, LOW);
		delayMicroseconds(500);
		SPI.transfer(command_loadConfiguration);
		delayMicroseconds(500);
		for (i = 0; i < sizeof(_configuration); i++) {
			SPI.transfer(_configuration[i]);
		}
		delayMicroseconds(1500);
		rx_ACK = SPI.transfer(0);
		digitalWrite(pin_SS, HIGH);
		delayMicroseconds(500);
		ti = millis();
	}
	SPI.endTransaction();

	if (rx_ACK == ACK) {
		return 0;
	} else {
		return -1;
	}

}

int uRAD_GUI_SDK11::detection_GUI(void) {

	// Variables
	uint8_t mode, Ns, Ns_3, Ns_temp, rx_ACK;
	uint16_t i, total_bytes_int, two_blocks_1, two_blocks_2, two_blocks_3, read_timeout = 200;
	float total_bytes;
	int return_code = 0;
	uint64_t t0, ti;	

	mode = (_configuration[0] & 0b11100000) >> 5;
	Ns = ((_configuration[2] & 0b00011111) << 3) + ((_configuration[3] & 0b11100000) >> 5);
	Ns_temp = Ns;
	if (mode == 3) {
		Ns_temp = 2 * Ns_temp;
	} else if (mode == 4) {
		Ns_temp = 2 * Ns_temp + 2 * ceil(0.75 * Ns_temp);
	}		
	total_bytes = 0;
	if ((Ns % 2) == 0) {
		total_bytes += Ns*1.5;
		two_blocks_1 = floor(Ns*1.5/3);
	} else {
		total_bytes += (Ns+1)*1.5;
		two_blocks_1 = floor((Ns+1)*1.5/3);
	}
	if ((mode == 3) || (mode == 4)) {
		two_blocks_2 = two_blocks_1;
		total_bytes *= 2;
	}
	if (mode == 4) {
		Ns_3 = ceil(0.75*Ns);
		if ((Ns_3 % 2) == 0) {
			total_bytes += 2*Ns_3*1.5;
			two_blocks_3 = floor(Ns_3*1.5/3);
		} else {
			total_bytes += 2*(Ns_3+1)*1.5;
			two_blocks_3 = floor((Ns_3+1)*1.5/3);
		}
	}
	total_bytes_int = uint16_t(total_bytes);	
	
	// SPI configuration
	digitalWrite(pin_SS, HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
	
	rx_ACK = 0;
	t0 = millis();
	ti = millis();
	while ((rx_ACK != ACK) && ((ti - t0) < read_timeout)) {
		digitalWrite(pin_SS, LOW);
		delayMicroseconds(500);
		SPI.transfer(command_detection);
		delayMicroseconds(1500);
		rx_ACK = SPI.transfer(0);
		digitalWrite(pin_SS, HIGH);
		delayMicroseconds(500);
		ti = millis();
	}
	if ((ti-t0) >= read_timeout) {
		return_code = -1;
	}
	
	if (return_code == 0) {
		// Results are ready?
		rx_ACK = 0;
		t0 = millis();
		ti = millis();
		while ((rx_ACK != ACK) && ((ti - t0) < read_timeout)) {
			digitalWrite(pin_SS, LOW);
			//delayMicroseconds(1000);
			SPI.transfer(command_ready);
			delayMicroseconds(1500);
			rx_ACK = SPI.transfer(0);
			digitalWrite(pin_SS, HIGH);
			//delayMicroseconds(500);
			ti = millis();
			//if (rx_ACK != ACK) {
			//	delayMicroseconds(1000);
			//}
		}
		if ((ti-t0) >= read_timeout) {
			return_code = -1;
		}
		
		if (return_code == 0) {
			
			// Receive and transmit I,Q	
			uint8_t bufferTbytes[total_bytes_int];				
			digitalWrite(pin_SS, LOW);				
			delayMicroseconds(500);
			SPI.transfer(command_I);
			delayMicroseconds(500);
			for (i = 0; i < total_bytes_int; i++) {
				bufferTbytes[i] = SPI.transfer(0);
				//Serial.write(bufferTbytes[i]);
			}
			digitalWrite(pin_SS, HIGH);
			for (i = 0; i < total_bytes_int; i++) {
				Serial.write(bufferTbytes[i]);
			}	
							
			digitalWrite(pin_SS, LOW);				
			delayMicroseconds(500);
			SPI.transfer(command_Q);
			delayMicroseconds(500);
			for (i = 0; i < total_bytes_int; i++) {
				bufferTbytes[i] = SPI.transfer(0);
				//Serial.write(bufferTbytes[i]);
			}
			digitalWrite(pin_SS, HIGH);
			for (i = 0; i < total_bytes_int; i++) {
				Serial.write(bufferTbytes[i]);
			}			
		}	
	}	
	SPI.endTransaction();
	return return_code;

}

int uRAD_GUI_SDK11::turnON(void) {
	
	// Variables
	uint8_t rx_ACK;
	unsigned long t0, ti, read_timeout = 200;
	
	digitalWrite(pin_ON_OFF, HIGH);
	delay(50);
	
	// SPI configuration
	digitalWrite(pin_SS, HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
	
	//Send turnON by SPI
	rx_ACK = 0;
	t0 = millis();
	ti = millis();
	while ((rx_ACK != ACK) && ((ti - t0) < read_timeout)) {
		digitalWrite(pin_SS, LOW);
		delayMicroseconds(500);
		SPI.transfer(command_turnON);	
		delayMicroseconds(1500);
		rx_ACK = SPI.transfer(0);
		digitalWrite(pin_SS, HIGH);
		delayMicroseconds(500);
		ti = millis();
	}
	SPI.endTransaction();
	
	if (rx_ACK == ACK) {
		return 0;
	} else {
		return -1;
	}
	
}

int uRAD_GUI_SDK11::turnOFF(void) {
	
	// Variables
	uint8_t rx_ACK;
	unsigned long t0, ti, read_timeout = 200;
	
	// SPI configuration
	digitalWrite(pin_SS, HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
	
	//Send turnOFF by SPI
	rx_ACK = 0;
	t0 = millis();
	ti = millis();
	while ((rx_ACK != ACK) && ((ti - t0) < read_timeout)) {
		digitalWrite(pin_SS, LOW);
		delayMicroseconds(500);
		SPI.transfer(command_turnOFF);	
		delayMicroseconds(1500);
		rx_ACK = SPI.transfer(0);
		digitalWrite(pin_SS, HIGH);
		delayMicroseconds(500);
		ti = millis();
	}
	SPI.endTransaction();
	digitalWrite(pin_ON_OFF, LOW);
	
	if (rx_ACK == ACK) {
		return 0;
	} else {
		return -1;
	}

}