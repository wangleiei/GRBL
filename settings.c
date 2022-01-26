/*
  settings.c - eeprom configuration handling 
  Part of Grbl

  The MIT License (MIT)

  Grbl(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "settings.h"

// settings_t settings;

// Version 1 outdated settings record
typedef struct {
  double steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  double default_feed_rate;
  double default_seek_rate;
  uint8_t invert_mask;
  double mm_per_arc_segment;
} settings_v1_t;

void settings_reset() {
  settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
  settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
  settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
  settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
  settings.default_feed_rate = DEFAULT_FEEDRATE;
  settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
  settings.acceleration = DEFAULT_ACCELERATION;
  settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
  settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
  settings.max_jerk = DEFAULT_MAX_JERK;
}

void settings_dump(GRBL_METH *meth) {
  meth->printPgmString("$0 = "); printFloat(meth->settings.steps_per_mm[X_AXIS]);
  meth->printPgmString(" (steps/mm x)\r\n$1 = "); printFloat(meth->settings.steps_per_mm[Y_AXIS]);
  meth->printPgmString(" (steps/mm y)\r\n$2 = "); printFloat(meth->settings.steps_per_mm[Z_AXIS]);
  meth->printPgmString(" (steps/mm z)\r\n$3 = "); printInteger(meth->settings.pulse_microseconds);
  meth->printPgmString(" (microseconds step pulse)\r\n$4 = "); printFloat(meth->settings.default_feed_rate);
  meth->printPgmString(" (mm/min default feed rate)\r\n$5 = "); printFloat(meth->settings.default_seek_rate);
  meth->printPgmString(" (mm/min default seek rate)\r\n$6 = "); printFloat(meth->settings.mm_per_arc_segment);
  meth->printPgmString(" (mm/arc segment)\r\n$7 = "); printInteger(meth->settings.invert_mask); 
  meth->printPgmString(" (step port invert mask. binary = "); //printIntegerInBase(meth->settings.invert_mask, 2);  
  meth->printPgmString(")\r\n$8 = "); printFloat(meth->settings.acceleration);
  meth->printPgmString(" (acceleration in mm/sec^2)\r\n$9 = "); printFloat(meth->settings.max_jerk);
  meth->printPgmString(" (max instant cornering speed change in delta mm/min)");
  meth->printPgmString("\r\n'$x=value' to set parameter or just '$' to dump current settings\r\n");
}

void write_settings() {
  // eeprom_put_char(0, SETTINGS_VERSION);
  // memcpy_to_eeprom_with_checksum(1, (uint8_t*)&settings, sizeof(settings_t));
}

int read_settings() {
  // Check version-byte of eeprom
  // uint8_t version = eeprom_get_char(0);
  
  // if (version == SETTINGS_VERSION) {
  //   // Read settings-record and check checksum
  //   if (!(memcpy_from_eeprom_with_checksum((uint8_t*)&settings, 1, sizeof(settings_t)))) {
  //     return(0);
  //   }
  // } else if (version == 1) {
  //   // Migrate from old settings version
  //   if (!(memcpy_from_eeprom_with_checksum((uint8_t*)&settings, 1, sizeof(settings_v1_t)))) {
  //     return(0);
  //   }
  //   settings.acceleration = DEFAULT_ACCELERATION;
  //   settings.max_jerk = DEFAULT_MAX_JERK;
  // } else {      
  //   return(0);
  // }
  return(1);
}

// A helper method to set settings from command line
void settings_store_setting(GRBL_METH *meth,int parameter, double value) {
  switch(parameter) {
		case 0: 
		case 1: 
		case 2:
			meth->settings.steps_per_mm[parameter] = value; break;
		case 3: 
			meth->settings.pulse_microseconds = round(value); 
		break;
		case 4: 
			meth->settings.default_feed_rate = value; 
		break;
		case 5: 
			meth->settings.default_seek_rate = value; 
		break;
		case 6: 
			meth->settings.mm_per_arc_segment = value; 
		break;
		case 7: 
			meth->settings.invert_mask = trunc(value); 
		break;
		case 8: 
			meth->settings.acceleration = value; 
		break;
		case 9: 
			meth->settings.max_jerk = fabs(value); 
		break;
		default: 
			GrblMeth.printPgmString("Unknown parameter\r\n");
		return;
	}
	meth->printPgmString("Stored new setting\r\n");
}

// Initialize the config subsystem
void settings_init() {
  // if(read_settings()) {
  //   meth->printPgmString("'$' to dump current settings\r\n");
  // } else {
  meth->printPgmString("Warning: Failed to read EEPROM settings. Using defaults.\r\n");
  settings_reset();
  // write_settings();
  settings_dump();
  // }
}
