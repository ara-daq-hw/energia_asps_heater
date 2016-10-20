/*
 * Code for the ASPS-DAQ heater uC.
 *
 * This code is designed to control current through
 * a FET+Resistor attached to a heatsink, to generate
 * heat before the system comes up fully.
 *
 */
#include <PID.h>
#include <ArduinoJson.h>
#include <msp430.h>
#include <HardwareSerial.h>

#define TEMPINT 15
#define VIN_DISABLE 29
#define VIN_FAULT 23
#define EN_3V3 16
#define HEATER 22
#define GREEN 10
#define RED 9

#define CURRENT A1
#define MON_15V A8
#define MON_VIN A9

#define SERIAL_ENABLE 25

PID heaterPID;

#define INFO_SIGNATURE 0x03
#define CUR_REVISION 1

typedef struct info_t {
  unsigned char signature;              //< Indicates if the info structure is up to date.
  unsigned char revision;               //< What board revision this is.
  unsigned int centidegrees_per_count;  //< Count conversion (0.01 C/count)
  unsigned int temperature_offset;      //< Offset (in 0.01 C) of temperature.
  unsigned int heater_params[6];        //< Heater parameters. Min temp (0), current setting (1), max wait time (2), turn-off delay time (3), PID defaults (4, 5, 6).
} info_t;

info_t *my_info = (info_t *) 0x1800;

// REVISION DIFFERENCES
// Rev 0: No serial enable. Unused GPIOs are PJ.0 and PJ.1 (25/26).
// Rev 1: Serial enable on PJ.0. Unused GPIOs are P2.4 and P2.3.

typedef enum system_state {
	STATE_BOOT = 0,
	STATE_POWER_BAD = 1,
	STATE_HEATING = 2,
	STATE_READY_WAIT_OFF = 3,
	STATE_READY = 4,
  STATE_TIMED_OUT = 5
} system_state_t;

system_state_t __attribute__ ((section(".noinit"))) state;

#define CMD_BUFFER_LENGTH 80
char __attribute__ ((section (".noinit"))) cmd_buffer[CMD_BUFFER_LENGTH];
unsigned char cmd_buffer_ptr = 0;

bool __attribute__ ((section (".noinit"))) serial_running;

unsigned long serialTime;
// Output every 500 millis.
#define SERIAL_PERIOD 500

unsigned long computeTime;
// Compute every 100 millis.
#define COMPUTE_PERIOD 100

// Number of serial periods (half-seconds) we've waited.
unsigned int waitingTime;

// Heater parameter defaults.

// Minimum low temp before we wait.
#define DEFAULT_TEMPERATURE_TOO_LOW 3000
// Current output when too low.
#define DEFAULT_HEATER_CURRENT 500
// Half-seconds to wait before just trying a turn on (1 hour).
#define DEFAULT_MAX_HEAT_WAIT_TIME 7200
// Half-seconds to wait from temp above -40 to heater-off.
#define DEFAULT_PID_WAIT_TIME 600
// Default P (0.5)
#define DEFAULT_HEATER_P 128
// Default I (0.25)
#define DEFAULT_HEATER_I 64
// Default D (0)
#define DEFAULT_HEATER_D 0

enum heaterParamEnum {
	TEMPERATURE_TOO_LOW = 0,
	HEATER_CURRENT = 1,
	MAX_HEAT_WAIT_TIME = 2,
	PID_WAIT_TIME = 3,
	HEATER_P = 4,
	HEATER_I = 5,
	HEATER_D = 6
};

// High-current limit. This isn't configurable.
#define CURRENT_LIMIT 1000



long readCurrent();
void parseJsonInput();

// Find calibration tags.
typedef struct adc_calibration {
	unsigned char tag;					//< Identifier tag of the ADC calibration block.
	unsigned char len;					//< Length of the ADC calibration block.
	unsigned int gain;					//< Inverse gain of the ADC, times 2^15. To correct ADC, multiply by gain, divide by 2^15.
	unsigned int offset;				//< Offset of the ADC. To correct ADC, add offset (after gain correction)
	unsigned int temp30_1v5;			//< Raw reading of temperature sensor at 30 C using 1.2V internal reference
	unsigned int temp85_1v5;			//< Raw reading of temperature sensor at 85 C using 1.2V internal reference.
	unsigned int temp30_2v0;			//< Raw reading of temperature sensor at 30 C using 2.0V internal reference.
	unsigned int temp85_2v0;			//< Raw reading of temperature sensor at 85 C using 2.0V internal reference.
	unsigned int temp30_2v5;			//< Raw reading of temperature sensor at 30 C using 2.5V internal reference.
	unsigned int temp85_2v5;			//< Raw reading of temperature sensor at 85 C using 2.5V internal reference.
} adc10b_calibration_t;

// Die record tags.
typedef struct die_record {
	unsigned char tag;					//< Identifier tag of the die record block.
	unsigned char len;					//< Length of the die record block.
	unsigned int lot_wafer[2];			//< Lot/wafer ID.
	unsigned int xpos;					//< Die X position.
	unsigned int ypos;					//< Die Y position.
	unsigned int test_results;			//< Test results.
} die_record_t;

#define TLV_START (0x1A08)
#define TLV_ADC10BCAL (0x13)
#define TLV_DIERECORD (0x08)

inline unsigned char *msp430_tag_find(unsigned char tag) {
	unsigned char *p;
	p = (unsigned char *) TLV_START;
	while (*p != tag) {
		unsigned char len;
		p++;
		len = *p++;
		p += len;
	}
	return p;
}

/** \brief Read current board temperature. */
int readTemperature() {
	int temperature;

	// should *really* use the actual real temp sensor at some point...
	temperature = analogRead(A10);
	temperature -= my_info->temperature_offset;
	temperature = temperature*(my_info->centidegrees_per_count);
	temperature += 3000;
	return temperature;
}

/** \brief Read input voltage. */
unsigned int readVin() {
	unsigned long voltage;

	// Voltage goes through 10k | 100k resistor divider, giving
	// 1 V => 0.090909 mV/mV. We have 1500 mV/1023 counts, so 0.682 counts/mV
	// 1 V => 0.061999938 counts/mV
	// or 16.129048 mV/count.
	// or 1057033.
	// e.g. a 15V input yields 1.3636 mV, or 930 counts.
	// 930 counts * 1057033/65536 = 15000 mV.
	voltage = analogRead(MON_VIN);
	voltage = voltage*1057033;
	voltage >>= 16;
	return voltage;
}

/** \brief Read +15V. Same divider as VIN. */
unsigned int read15V() {
	unsigned long voltage;

	voltage = analogRead(MON_15V);
	voltage = voltage*1057033;
	voltage >>= 16;
	return voltage;
}

/** \brief Get heater current, measured in mA. */
long readCurrent() {
	long val;

	// analogRead returns 0-1023 corresponding to 0-1500 mV.
	// We read 128 times, so it's now 0->130944, corresponding to 0-1500 mV.
	// The sense resistor is 0.2 ohms, so it's 0-7500 mA.
	// Therefore the milliamp to count conversion is 0.05727639.. mA/count.
	val = 0;
	for (int i=128;i;i--) {
		val = val + analogRead(CURRENT);
	}
	// So we want to multiply by 0.05727639. We want to avoid the hell
	// out of the float library, so:
	// 0.05727639:
	// 3753/65536 is 0.057266235.
	// At 1500 mA, that would be off by something like 0.4 mA.
	// Good enough!
	val = val * 3753;
	val >>= 16;
	return val;
}

void setup()
{
	unsigned char *tmp;
	unsigned int tmp2;
	unsigned int delta;
	int curTemp;

	adc10b_calibration_t *adc_calib;

	// Check immediately the state of VIN_DISABLE. If it's low, that means we're either at power-on or
	// power was previously disabled anyway.
	if (digitalRead(VIN_DISABLE) == 0) {
		// Kill power.
		digitalWrite(VIN_DISABLE, 0);
		digitalWrite(EN_3V3, 0);
		state = STATE_BOOT;
	} else {
    // Keep power on.
    digitalWrite(VIN_DISABLE, 1);
    digitalWrite(EN_3V3, 1);
		state = STATE_READY;
	}
  // Actually drive the outputs.
  pinMode(VIN_DISABLE, OUTPUT);
  pinMode(EN_3V3, OUTPUT);

  // Have we ever been programmed? If not, set it up.
  if (my_info->signature != INFO_SIGNATURE) {
	unsigned int tmp_cnt;
    my_info->revision = CUR_REVISION;
    tmp = msp430_tag_find(TLV_ADC10BCAL);
  	adc_calib = (adc10b_calibration_t *) tmp;

  	delta = adc_calib->temp85_1v5;
  	delta -= adc_calib->temp30_1v5;

  	tmp_cnt = 0;
  	tmp2 = 55*100;
  	while (tmp2 > delta) {
  		tmp2 -= delta;
  		tmp_cnt++;
  	}
    my_info->centidegrees_per_count = tmp_cnt;
  	my_info->temperature_offset = adc_calib->temp30_1v5;

  	my_info->heater_params[TEMPERATURE_TOO_LOW] = DEFAULT_TEMPERATURE_TOO_LOW;
  	my_info->heater_params[HEATER_CURRENT] = DEFAULT_HEATER_CURRENT;
  	my_info->heater_params[MAX_HEAT_WAIT_TIME] = DEFAULT_MAX_HEAT_WAIT_TIME;
  	my_info->heater_params[PID_WAIT_TIME] = DEFAULT_PID_WAIT_TIME;
  	my_info->heater_params[HEATER_P] = DEFAULT_HEATER_P;
  	my_info->heater_params[HEATER_I] = DEFAULT_HEATER_I;
  	my_info->heater_params[HEATER_D] = DEFAULT_HEATER_D;

  	my_info->signature = INFO_SIGNATURE;
  }

  // Set things up.

  // 1.5V internal reference.
  analogReference(INTERNAL1V5);
  // 12-bit analog resolution.
  analogResolution(4096);
  // Disable the heater.
  analogWrite(HEATER, 0);
  // Set up the serial ports. Has to be handled differently between board revisions.
  if (my_info->revision > 0) {
    if (state != STATE_READY) digitalWrite(SERIAL_ENABLE, 1);
    else digitalWrite(SERIAL_ENABLE, 0);
    pinMode(SERIAL_ENABLE, OUTPUT);
    Serial.begin(9600);
    Serial1.begin(9600);
    serial_running = true;
  } else {
    // Don't turn on the serial ports unless we're already running.
    if (state == STATE_READY) {
      Serial.begin(9600);
      Serial1.begin(9600);
      serial_running = true;
    } else {
      serial_running = false;
    }
  }

  pinMode(HEATER, OUTPUT);
  digitalWrite(GREEN, 1);
  digitalWrite(RED, 1);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);

  // Initial state machine run.
  if (state == STATE_BOOT) {
    heaterPID.SetTuningParametersRaw(my_info->heater_params[HEATER_P],
    								 my_info->heater_params[HEATER_I],
									 my_info->heater_params[HEATER_D]);
    heaterPID.SetOutputLimits(1500, 4096);
    curTemp = readTemperature();
    if (curTemp < ((int) my_info->heater_params[TEMPERATURE_TOO_LOW])) {
  	  heaterPID.setpoint = my_info->heater_params[HEATER_CURRENT];
  	  state = STATE_HEATING;
    } else {
  	  heaterPID.setpoint = 0;
  	  state = STATE_READY;
      powerTurnOn();
      digitalWrite(GREEN, 0);
    }
  } else {
    heaterPID.setpoint = 0;
    digitalWrite(GREEN, 0);
  }

  heaterPID.input = readCurrent();
  heaterPID.Reset(heaterPID.input);
  heaterPID.Compute();

  // If we have a setpoint, try to reach it. Otherwise, turn off
  // the heater completely.
  if (heaterPID.setpoint) analogWrite(HEATER, heaterPID.output);
  else analogWrite(HEATER, 0);

  serialTime = 0;
  computeTime = millis() + COMPUTE_PERIOD;
}

void loop()
{
  if ((long) (millis() - computeTime) > 0) {
	  heaterPID.input = readCurrent();
	  heaterPID.Compute();
	  if (heaterPID.setpoint) analogWrite(HEATER, heaterPID.output);
	  else analogWrite(HEATER, 0);
	  if (heaterPID.setpoint) {
		  // Figure out if we're out of bounds.
		  if ((heaterPID.input- heaterPID.setpoint) > 10 || (heaterPID.setpoint - heaterPID.input) > 10) {
			  digitalWrite(RED, 0);
		  } else {
			  digitalWrite(RED, 1);
		  }
	  }
	  computeTime = computeTime + COMPUTE_PERIOD;
  }
  
  if ((long) (millis() - serialTime) > 0)  {
  	int temperature;
  	unsigned int vin;
  	unsigned int v15;

  	temperature = readTemperature();
  	vin = readVin();
  	v15 = read15V();

    if (serial_running) {
    	Serial1.print("{\"pid\":[");
    	Serial1.print(heaterPID.setpoint);
    	Serial1.print(",");
    	Serial1.print(heaterPID.input);
    	Serial1.print(",");
    	Serial1.print(heaterPID.output);
    	Serial1.print("],\"temps\":[");
    	Serial1.print(temperature);
    	Serial1.print("],\"volts\":[");
    	Serial1.print(vin);
    	Serial1.print(",");
    	Serial1.print(v15);
    	Serial1.println("]}");

    	Serial.print("state ");
    	Serial.println(state);
    	Serial.print("pid ");
    	Serial.print(heaterPID.setpoint);
    	Serial.print(" ");
    	Serial.print(heaterPID.input);
    	Serial.print(" ");
    	Serial.println(heaterPID.output);

    	Serial.print("temp ");
    	Serial.println(temperature);

    	Serial.print("volts ");
    	Serial.print(vin);
    	Serial.print(" ");
    	Serial.println(v15);
    }

    // State machine.
  	if (state == STATE_HEATING) {
  		if (temperature >= ((int) my_info->heater_params[TEMPERATURE_TOO_LOW])) {
  			state = STATE_READY_WAIT_OFF;
  			powerTurnOn();
  			waitingTime = 0;
  		} else if (waitingTime == my_info->heater_params[MAX_HEAT_WAIT_TIME]) {
  			// Screw it, try to turn on. Leave the heater on.
  			state = STATE_TIMED_OUT;
  			powerTurnOn();
  			waitingTime = 0;
  		} else
  			waitingTime++;
  	} else if (state == STATE_READY_WAIT_OFF) {
  		if (waitingTime == my_info->heater_params[PID_WAIT_TIME]) {
  			state = STATE_READY;
  			heaterPID.setpoint = 0;
  			digitalWrite(GREEN, 0);
  			waitingTime = 0;
  		} else waitingTime++;
  	} else if (state == STATE_TIMED_OUT) {
      if (temperature >= ((int) my_info->heater_params[TEMPERATURE_TOO_LOW])) {
        state = STATE_READY_WAIT_OFF;
        waitingTime = 0;
      }
  	}

  	serialTime += SERIAL_PERIOD;
  }

  if (serial_running) {
    while (Serial1.available()) {
  	  char c = Serial1.read();
  	  if (c == '\n') {
  		  cmd_buffer[cmd_buffer_ptr] = 0;
  		  parseJsonInput();
  		  cmd_buffer_ptr = 0;
  	  } else {
  		  cmd_buffer[cmd_buffer_ptr] = c;
  		  if (cmd_buffer_ptr < CMD_BUFFER_LENGTH-1) cmd_buffer_ptr++;
  	  }
    }
  }
}

void parseJsonInput() {
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(cmd_buffer);
	if (!root.success()) {
		Serial1.println("{\"dbg\":\"invalid JSON\"}");
		return;
	}
	if (root.containsKey("current")) {
		unsigned int cur;
		cur = root["current"].as<int>();
		if (cur <= CURRENT_LIMIT && cur >= 0) {
			heaterPID.setpoint = cur;
		}
	}
	if (root.containsKey("heaterparams")) {
		JsonArray& params = root["heaterparams"];
		if (params.size() >= 2) {
		  unsigned int idx = params[0];
		  int val = params[1];
		  if (idx < 6) {
			my_info->heater_params[idx] = val;
		  }
		}
		Serial1.print("{\"heaterparams\":[");
		for (unsigned int i=0;i<7;i++) {
		  if (i) {
			  Serial1.print(",");
			  Serial1.print(my_info->heater_params[i]);
		  } else {
			  Serial1.print((int) my_info->heater_params[i]);
		  }
		}
		Serial1.println("]}");
	}
}

void powerTurnOn() {
    digitalWrite(VIN_DISABLE, 1);
    digitalWrite(EN_3V3, 1);
    if (my_info->revision > 0) {
      digitalWrite(SERIAL_ENABLE, 0);
    } else {
      Serial.begin(9600);
      Serial1.begin(9600);
      serial_running = true;
    }
}
