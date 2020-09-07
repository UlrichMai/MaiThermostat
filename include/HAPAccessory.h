#pragma once

#include <homekit/types.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#ifdef __cplusplus
    #define EXPORT_C extern "C"
#else
    #define EXPORT_C
#endif

EXPORT_C homekit_server_config_t config;

EXPORT_C homekit_characteristic_t accessory_name;
EXPORT_C homekit_characteristic_t accessory_manufacturer;
EXPORT_C homekit_characteristic_t accessory_serial_number;
EXPORT_C homekit_characteristic_t accessory_model;
EXPORT_C homekit_characteristic_t accessory_firmware_revision;
EXPORT_C homekit_characteristic_t accessory_identify_cb;                      //callback

EXPORT_C homekit_characteristic_t thermostat_name;
EXPORT_C homekit_characteristic_t thermostat_current_temperature;             //.getter
EXPORT_C homekit_characteristic_t thermostat_target_temperature;              //.getter,.setter
EXPORT_C homekit_characteristic_t thermostat_current_heating_cooling_state;   //.getter
EXPORT_C homekit_characteristic_t thermostat_target_heating_cooling_state;    //.getter,.setter
EXPORT_C homekit_characteristic_t thermostat_temperature_display_units;
EXPORT_C homekit_characteristic_t thermostat_current_humidity;                //.getter

EXPORT_C void homekit_notify_loop();

EXPORT_C homekit_value_t HOMEKIT_FLOAT_CPPX(float value, float min_value, float max_value);

