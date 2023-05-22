#include <uRAD_SDK11.h> 	// include the library

uRAD_SDK11 uRAD;       		// create the object uRAD

// input parameters
uint8_t mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth, alpha;
bool distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true;
int return_code;

// results output array
uint8_t NtarDetected[1];
float distance[5], SNR[5];
uint16_t I[200], Q[200];

void setup() {
	Serial.begin(250000);   	// serial port baud rate

	mode = 2;					// sawtooth mode
	f0 = 5;						// starting at 24.005 GHz
	BW = 240;					// using all the BW available = 240 MHz
	Ns = 200;					// 200 samples
	Ntar = 3;					// 3 target of interest
	Rmax = 100;					// searching along the full distance range
	MTI = 0;					// MTI mode disable because we want information of static and moving targets
	Mth = 0;					// parameter not used because "movement" is not requested
	alpha = 10;					// signal has to be 10 dB higher than its surrounding
	distance_true = false; 		// request distance information
	velocity_true = false;		// mode 2 does not provide velocity information
	SNR_true = false; 			// Signal-to-Noise-Ratio information requested
	I_true = true; 			// In-Phase Component (RAW data) not requested
	Q_true = true; 			// Quadrature Component (RAW data) not requested
	movement_true = false; 		// not interested in boolean movement detection

	
	// switch ON uRAD
	return_code = uRAD.turnON();
	if (return_code != 0) {
		Serial.println("Error turning on uRAD");
	}
	
	// load the configuration
	return_code = uRAD.loadConfiguration(mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth, alpha, distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true);
	if (return_code != 0) {
		Serial.println("Error configuring uRAD");
	}

}

void loop() {
	// target detection request
	return_code = uRAD.detection(0, 0, 0, 0, I, Q, 0);

	// iterate through desired targets
	if (return_code == 0) {
		for (uint8_t i = 0; i < Ns; i++) {
			// if target is reflective enough, print its distance
			Serial.print(i);
			Serial.print(" ");
			Serial.print(I[i]);
			Serial.print(" ");
			Serial.println(Q[i]);
		}

	} else {
		Serial.println("Error in detection");
	}
	
	Serial.println(" ");
	delay(500);
}
