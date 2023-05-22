#include <uRAD_SDK11.h> 	// include the library

uRAD_SDK11 uRAD;       		// create the object uRAD

// input parameters
uint8_t mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth, alpha;
bool distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true;
int return_code;

// results output array
uint8_t NtarDetected[1];
float distance[5], velocity[5], SNR[5];

void setup() {
	Serial.begin(250000);  		// serial port baud rate

	mode = 3;					// triangular dual rate mode
	f0 = 5;						// start frequency 24.005 GHz
	BW = 240;					// sweep 240 MHz
	Ns = 200;					// 200 samples
	Ntar = 3;					// 3 target of interest
	Rmax = 100;					// searching along the full distance range
	MTI = 0;					// MTI mode disable because we want information of static and moving targets
	Mth = 0;					// parameter not used because "movement" is not requested
	alpha = 10;					// signal has to be 10 dB higher than its surrounding
	distance_true = true; 		// request distance information
	velocity_true = true;		// mode 2 does not provide velocity information
	SNR_true = true; 			// Signal-to-Noise-Ratio information requested
	I_true = false; 			// In-Phase Component (RAW data) not requested
	Q_true = false; 			// Quadrature Component (RAW data) not requested
	movement_true = false; 		// Not interested in boolean movement detection

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
	return_code = uRAD.detection(NtarDetected, distance, velocity, SNR, 0, 0, 0);

	// iterate through desired targets
	if (return_code == 0) {
		for (uint8_t i = 0; i < NtarDetected[0]; i++) {
			// if target is reflective enough, print its distance
			if (SNR[i] > 0){   
				Serial.print("Target: ");
				Serial.print(i+1);
				Serial.print(", Distance: ");
				Serial.print(distance[i]);
				Serial.print(" m, Velocity: ");
				Serial.print(velocity[i]);
				Serial.print(" m/s, SNR: ");
				Serial.print(SNR[i]);
				Serial.println(" dB");
			}
		}
		// If number of detected targets is greater than 0 prints an empty line for a smarter output
		if (NtarDetected[0] > 0) {
			Serial.println(" ");	
		}
	} else {
		Serial.println("Error in detection");
	}
}
