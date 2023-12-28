#include <BasicLinearAlgebra.h>
#include "Printer.h"

extern const bool DIAGNOSTIC_PRINT_MASK[8];
extern const char HEADER_STRINGS[8][40];
extern const char SPACER;

Printer::Printer() {
	memcpy(this->mask, DIAGNOSTIC_PRINT_MASK, 8*sizeof(bool));
	for (int i = 0; i < 8; i++) {
		strcpy(this->header_strings[i], HEADER_STRINGS[i]);
	}
	this->spacer = SPACER;
}

void Printer::print_header() {
	for (int i = 0; i < 8; i++) {
		if (mask[i]) {
			Serial.print(header_strings[i]);
			Serial.print(spacer);
		}
	}
	Serial.print("\n");
}

void Printer::print(float setpoint, float error, BLA::Matrix<4> state_estimate, float action, unsigned int servo_cmd, unsigned long time_change) {

	float vals[] = { 1000 * setpoint,
					1000 * (error + setpoint),
					1000 * (state_estimate(0) + setpoint),
					50 * state_estimate(1),
					50 * state_estimate(2),
					500 * action,
					0.1 * float(servo_cmd),
					float(time_change)
	};


	for (int i = 0; i < 8; i++) {
		if (mask[i]) {
			Serial.print(vals[i], 6);
			Serial.print(spacer);
		}
	}
	Serial.print("\n");
}
