/*
 * HAPAccessory.c
 * Define the homekit accessories for thermostat
 *
 *  Created on: 2020-08-31
 *      Author: Ulrich Mai
 */

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "HAPAccessory.h"

void accessory_identify(homekit_value_t _value) {
	printf("accessory identify\n");
}

homekit_characteristic_t accessory_name                           = HOMEKIT_CHARACTERISTIC_(NAME, "Thermostat");
homekit_characteristic_t accessory_manufacturer                   = HOMEKIT_CHARACTERISTIC_(MANUFACTURER, "UM");
homekit_characteristic_t accessory_serial_number                  = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "SN_00000001");
homekit_characteristic_t accessory_model                          = HOMEKIT_CHARACTERISTIC_(MODEL, "BHT-002-GALW");
homekit_characteristic_t accessory_firmware_revision              = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, "0.0.1");
homekit_characteristic_t accessory_identify_cb                    = HOMEKIT_CHARACTERISTIC_(IDENTIFY, accessory_identify);

homekit_characteristic_t thermostat_name                          = HOMEKIT_CHARACTERISTIC_(NAME, "Thermostat");
homekit_characteristic_t thermostat_current_temperature           = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 20.0, .min_step  = (float[]) {0.5},  .getter=NULL );
homekit_characteristic_t thermostat_target_temperature            = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,  21.0, .min_value = (float[]) {5.0}, .max_value = (float[]) {35.0}, .min_step = (float[]) {0.5}, .getter=NULL, .setter=NULL );
homekit_characteristic_t thermostat_current_heating_cooling_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0, .valid_values = {.count = 2, .values = (uint8_t[]) {0,1} }, .getter=NULL );
homekit_characteristic_t thermostat_target_heating_cooling_state  = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  0, .valid_values = {.count = 2, .values = (uint8_t[]) {0,1} }, .getter=NULL, .setter=NULL  );
homekit_characteristic_t thermostat_temperature_display_units     = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0); //Celsius
homekit_characteristic_t thermostat_current_humidity              = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 50.0, .min_step  = (float[]) {1.0},  .getter=NULL );

homekit_accessory_t *accessories[] =
		{
				HOMEKIT_ACCESSORY(
						.id = 1,
						.category = homekit_accessory_category_thermostat,
						.services=(homekit_service_t*[]){
							HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
								.characteristics=(homekit_characteristic_t*[]){
									&accessory_name,
									&accessory_manufacturer,
									&accessory_serial_number,
									&accessory_model,
									&accessory_firmware_revision,
                  &accessory_identify_cb,
									NULL
								}),
             
              HOMEKIT_SERVICE(THERMOSTAT, .primary=true,
                .characteristics=(homekit_characteristic_t*[]) {
                  &thermostat_name,
                  &thermostat_current_temperature,
                  &thermostat_target_temperature,
                  &thermostat_current_heating_cooling_state,
                  &thermostat_target_heating_cooling_state,
                  &thermostat_temperature_display_units,
				  &thermostat_current_humidity,
                  NULL
                }),

							NULL
						}),
				NULL
		};

homekit_server_config_t config = {
		.accessories = accessories,
		.password = "111-11-111",
		//.on_event = on_homekit_event,
		.setupId = "ABCD"
};

static homekit_value_t old_value[5];
static homekit_value_t new_value;

#define NOTIFY_WHEN_CHANGED(no,cha) \
  new_value = (cha).getter(); \
  if (!homekit_value_equal(&old_value[(no)],&new_value)) { \
    old_value[(no)] = new_value; \
    (cha).value = new_value; \
    homekit_characteristic_notify(&(cha),new_value); \
  }  

void homekit_notify_loop() {
  NOTIFY_WHEN_CHANGED(0,thermostat_current_temperature);
  NOTIFY_WHEN_CHANGED(1,thermostat_target_temperature);
  NOTIFY_WHEN_CHANGED(2,thermostat_current_heating_cooling_state);
  NOTIFY_WHEN_CHANGED(3,thermostat_target_heating_cooling_state);
  NOTIFY_WHEN_CHANGED(4,thermostat_current_humidity);
}

homekit_value_t HOMEKIT_FLOAT_CPPX(float value, float min_value, float max_value) {
	homekit_value_t homekit_value;
  homekit_value.is_null = false;
	homekit_value.format = homekit_format_float;
	homekit_value.float_value = (value < min_value ? min_value: (value > max_value ? max_value : value) );
	return homekit_value;
}