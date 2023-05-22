/*
 * uRAD_GUI_SDK11.h - Library for controlling uRAD SHIELD with the Grafical User Interface
 * Created by Victor Torres, Diego Gaston - uRAD 2020
*/

#ifndef uRAD_GUI_SDK11_h
#define uRAD_GUI_SDK11_h
#include "Arduino.h"

class uRAD_GUI_SDK11
{
	public:
		uRAD_GUI_SDK11();
		int loadConfiguration_GUI(void);
		int detection_GUI(void);
		int turnON(void);
		int turnOFF(void);
	private:
		uint8_t _configuration[8], _bytesDisponibles;
};

#endif
