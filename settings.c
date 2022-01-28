#include "settings.h"

void settings_reset(GRBL_METH *meth) {
  meth->settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
  meth->settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
  meth->settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
  meth->settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
  meth->settings.default_feed_rate = DEFAULT_FEEDRATE;
  meth->settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
  meth->settings.acceleration = DEFAULT_ACCELERATION;
  meth->settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
  meth->settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
  meth->settings.max_jerk = DEFAULT_MAX_JERK;
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
void settings_store_setting(GRBL_METH *meth,int parameter, double value){
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
			meth->printPgmString("Unknown parameter\r\n");
		return;
	}
	meth->printPgmString("Stored new setting\r\n");
}

// Initialize the config subsystem
void settings_init(GRBL_METH *meth) {
  // if(read_settings()) {
  //   meth->printPgmString("'$' to dump current settings\r\n");
  // } else {
  meth->printPgmString("Warning: Failed to read EEPROM settings. Using defaults.\r\n");
  settings_reset(meth);
  // write_settings();
  settings_dump(meth);
  // }
}
