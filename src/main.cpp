#include <Arduino.h>
#include <ArduinoOTA.h>
#include <arduino_homekit_server.h>
#include <Ticker.h>

// time includes
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>                  // settimeofday_cb()

#include "Debug.h"
#include "HAPAccessory.h"
#include "BHT002.h"

#include <Adafruit_BME280.h>
#include <math.h>

#define PIN_RELAY_MONITOR 13

#ifdef DBG_TCP_ENABLED
  #define DBG_TCP_PORT  8888
  WiFiServer  serverDebug(DBG_TCP_PORT);
  WiFiClient clientDebug;
#endif // DBG_TCP

// Private.h contains the ssid and password as a temporary measure until a config page is added
#include "Private_UM.h"

#define TIMEZONE 	"CET-1CEST,M3.5.0,M10.5.0/3" // Europe/Berlin FROM https://github.com/nayarsystems/posix_tz_db/blob/master/zones.json

TUYAThermostatState state;

char* hostname =  "Thermostat-%02X%02X%02X";

void generateHostname() {
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  sprintf(hostname, hostname, mac[3], mac[4], mac[5]);
}

static void initTime() {

	// set function to call when time is set
	// is called by NTP code when NTP is used
	settimeofday_cb(
    []() -> void { state.setTimeAvailable(true); 
    DBG("state.setTimeAvailable(true)");}
  );

	// set time from RTC
	// Normally you would read the RTC to eventually get a current UTC time_t
	// this is faked for now.
	time_t rtc_time_t = 1541267183; // fake RTC time for now

	timezone tz = { 0, 0};
	timeval tv = { rtc_time_t, 0};

	// DO NOT attempt to use the timezone offsets
	// The timezone offset code is really broken.
	// if used, then localtime() and gmtime() won't work correctly.
	// always set the timezone offsets to zero and use a proper TZ string
	// to get timezone and DST support.

	// set the time of day and explicitly set the timezone offsets to zero
	// as there appears to be a default offset compiled in that needs to
	// be set to zero to disable it.
	settimeofday(&tv, &tz);


	// set up TZ string to use a POSIX/gnu TZ string for local timezone
	// TZ string information:
	// https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
	///setenv("TZ", TIMEZONE, 1);

	///tzset(); // save the TZ variable

	// enable NTP by setting up NTP server(s)
	// up to 3 ntp servers can be specified
	// configTime(tzoffset, dstflg, "ntp-server1", "ntp-server2", "ntp-server3");
	// set both timezone offet and dst parameters to zero 
	// and get real timezone & DST support by using a TZ string
	configTime(TIMEZONE, "pool.ntp.org");
}

// setup BME280 sensor
Adafruit_BME280 bme;
float humidity = 50.0f;
float temperature = 20.0f;
float pressure = 0.0f;

Ticker bmeTimer(
  [](const event_t e) ->  void {
    switch (e) {
      case START:
        if (!bme.begin(0x76,&Wire)) {
          bmeTimer.stop();
          DBG("Could not find a valid BME280 sensor, check wiring! Code=%x",bme.sensorID());
        }
        break;
      case UPDATE:
        humidity = roundf(bme.readHumidity()) + 2.0f;
        temperature = roundf(bme.readTemperature()*10.0f)/10.0f - 3.0f;
        pressure = roundf(bme.readPressure());
        DBG("BME280: humidity=%f,temperature=%f,pressure=%f",humidity,temperature,pressure);
        break;
    }
  },  
  10000);

void setup() {

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.disconnect(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }

  generateHostname();
  
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();

  #ifdef DBG_TCP_ENABLED
    serverDebug.begin();
  #endif 
  
 	Serial.begin(9600);

	state.setWifiConfigCallback([]() {
    DBG("Configuration portal opened");
    homekit_server_reset();
   });
 
  initTime();

  accessory_name.value = HOMEKIT_STRING_CPP(hostname);

  thermostat_current_temperature.getter = 
    []() -> homekit_value_t { return HOMEKIT_FLOAT_CPPX(state.getInternalTemp(),0.0,100.0 );  };

  thermostat_target_temperature.getter = 
    []() -> homekit_value_t { return HOMEKIT_FLOAT_CPPX(state.getSetPointTemp(),5.0,35.0 ); };
  thermostat_target_temperature.setter = 
    [](const homekit_value_t v) -> void { state.setSetPointTemp(v.float_value,true); thermostat_target_temperature.value = v; };

  thermostat_current_heating_cooling_state.getter = 
    []() -> homekit_value_t { return HOMEKIT_UINT8_CPP( state.getIsHeating() ? 1 : 0 ); };
// solution if IsHeating is not working:
//  []() -> homekit_value_t { return HOMEKIT_UINT8_CPP( state.getSetPointTemp() > state.getInternalTemp()  ? 1 : 0 ); };

  thermostat_target_heating_cooling_state.getter = 
    []() -> homekit_value_t { return HOMEKIT_UINT8_CPP( state.getPower() ? 1 : 0 ); };
  thermostat_target_heating_cooling_state.setter =
    [](const homekit_value_t v) -> void { state.setPower(v.uint8_value != 0,true); thermostat_target_heating_cooling_state.value = v; };

  thermostat_current_humidity.getter = 
    []() -> homekit_value_t { return HOMEKIT_FLOAT_CPPX(humidity,0.0,100.0 );  };

  arduino_homekit_setup(&config);

  bmeTimer.start();
  
  pinMode(PIN_RELAY_MONITOR, INPUT);
}

void loop() {
  ArduinoOTA.handle();

  arduino_homekit_loop();
  if (arduino_homekit_is_paired()) 
    homekit_notify_loop();
  
	state.loop();

  bmeTimer.update();

  state.setIsHeating(digitalRead(PIN_RELAY_MONITOR));   

  #ifdef DBG_TCP_ENABLED
  if (!clientDebug.connected()) {
    clientDebug.flush();
    clientDebug.stop();
    clientDebug = serverDebug.available();
  } else {
    if (clientDebug.available() && clientDebug.readString().equals(DBG_TCP_CLIENT_PING)) {
      clientDebug.write(DBG_TCP_CLIENT_PONG);
    }
  }
  uint32_t currtime = millis();
  static uint32_t nextDbgOut = 0;
  if (currtime > nextDbgOut) {
    nextDbgOut = currtime + 5000;
    DBG("getPower=%d,getLock=%d,getExternalTemp=%f,getInternalTemp=%f,getSetPointTemp=%f,getIsHeating=%d",
    state.getPower(),state.getLock(),state.getExternalTemp(),state.getInternalTemp(),state.getSetPointTemp(),state.getIsHeating() );
  }
  #endif 

  delay(100);

}
