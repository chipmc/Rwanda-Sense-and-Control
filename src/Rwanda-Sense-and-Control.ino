/*
* Project Environmental Sensor - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Uses two sensors - SHT-31 for Temp and Humidity and the Soil Watch Sensor for Soil moisture
* Tindie storefront - https://www.tindie.com/products/pinotech/soilwatch-10-soil-moisture-sensor/
* Seeed Studio for SHT-31 - http://wiki.seeedstudio.com/Grove-TempAndHumi_Sensor-SHT31/
* Author: Chip McClelland chip@seeinsights.com
* Sponsor: Thom Harvey ID&D
* Date: 27 Jan 2020
*/

// v1 - Initial Release Temp / Humidity / Soil Functionality
// v4 - First release on the Rwanda Product
// v5 - Updates to improve recoverability in LowPowerMode
// v6 - Explicitly tell the device to connect after waking from the hourly sleep
// v7 - Still not fixed.  Simplified sleep state and added another chance to connect
// v8 - Adding Blue Led for awake detection
// v9 - Ensuring we stay conected and deliver data to Ubidots - Minor fixes
// v10 - Fixed userSwitch pin assignment
// v11 - Adding new features from sensor expansion board - Motor Contol, Pressure and 2x Soil Sensors
// v11.1 - Moved to a new approach to managing persistent memory - took out Timezone stuff
// v12 - More safeties on the Solenoid Control
// v13 - Fixed Solar charging and set default hold time to 20mSec
// v14 - Added switches to determine configuration / Fixes Solenoid function as well
// v15 - Bug fixes and interface enhancements - added support for light sensor and a new webhook - deviceOS@1.5.1-rc2
// v16 - Fixed light level readings
// v17 - Fixed issue where SHT-31 initialization reported true even if no sensor present
// v18 - Fix for the soil moisture bug
// v19 - Adding support for watering - fixed threshold and period, hourly tests
// v20 - updated to Sleep 2.0
// v21 - Added Battery Context and Reporting, Fixed bug on lowPowerMode console status, Fixed cellular status bug
// v22 - All about reducing config errors on deployment - Made Solar Power Mode the default, set to lowPowerMode after 30 minutes and turns off verboseMode each day
// v23 - Temp/Humidity "nan" issue and reset cycle fixed. Fixed Solenoid Not present stuck on issue
// v24 - Moving back to PMIC control for charging
// v25 - Using the setPowerConfiguration API again with new values assigned to better suite a solar implementation
// v26 - Consistent "1" and "0" for all commands, Explicitly enabled charging, added battery context
// v27 - Trying some new values for the duration to turn on the solenoid
// v28 - Will now nap for two hours overnight (7pm - 5am)
// v28 - Took out the nap for two hours thing for now.  Added vairable duration and sensitivity to water.  Will report pressure greater than 1 psi.
// v29 - Safety - watering not needed will close the valve. 
// v30 - Added time of day awareness to support watering during specific hours
// v31 - Fixed broken logic on watering window
// v32 - Added a battery check on watering, raised the low power threshold, and reduced reporting at night to even hours between 2000 and 0400, watering periods at 8,12,17 hours each with own duration
// v33 - Fixed issue with reporting at night.
// v34 - Added pmic.enableBuck() to fix charging issue
// v35 - Fixed settings for power - 5.080V for panel threshold
// v36 - Solar charge fix (enableBuck) and double-tap open and close
// v37 - This fix is to move to deviceOS@2.0.0-rc2 and take out the PMIC fix required above.  
// v38 - Moves to deviceOS@2.0.0-rc3 and removing the extra Webhooks

// Particle Product definitions
PRODUCT_ID(10709);                                   // Connected Counter Header
PRODUCT_VERSION(38);
const char releaseNumber[6] = "38";                  // Displays the release on the menu


// Included Libraries
#include "math.h"
#include "adafruit-sht31.h"
#include "DevicePinoutdoc.h"
#include "BH1750.h"

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize the Sleep 2.0 API
Adafruit_SHT31 tempHumidSensor = Adafruit_SHT31();  // Temp and Humidity Sensor - Grove connected on i2c
BH1750 lightSensor(0x23, Wire);                     // Light sensor measures light level in Lux

namespace MEM_MAP {                                 // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentStatusAddr     = 0x50                    // Where we store the current status data structure
  };
};

const int MemVersionNumber = 2;                    // Increment this number each time the memory map is changed

struct systemStatus_structure {                     // currently 14 bytes long
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  uint8_t placeholder;                              // available for future use
  uint8_t metricUnits;                              // Status of key system states
  uint8_t connectedStatus;                          // Connected or not
  uint8_t verboseMode;                              // Extra text to facilitate problem solving
  uint8_t solarPowerMode;                           // Settings appropriate for solar power
  volatile uint8_t lowPowerMode;                             // Energy conserving mode
  uint8_t lowBatteryMode;                           // Reduced functionality when low battery conditiions exist
  int stateOfCharge;                                // Battery charge level
  uint8_t TempHumidConfig;                          // Do we have a temp / humidity sensor or not
  uint8_t powerState;                               // Stores the current power state
  uint8_t soilSensorConfig;                         // Number of soil sensors - 0,1,2
  uint8_t pressureSensorConfig;                     // Pressure sensor or not - 0,1
  uint8_t lightSensorConfig;                        // Light sensor or not - 0,1
  uint8_t solenoidConfig;                           // Solenoid or not - 0,1
  int solenoidHoldTime;                             // How long do we pulse the solenoid
  int resetCount;                                   // reset count of device (0-256)
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
  int wateringDurationMorning;                      // How long do we water for each watering event
  int wateringDurationLunch;
  int wateringDurationEvening;
  float wateringThresholdPct;                       // Soil Moisture that triggers watering
  int wateringWindow;                               // How many hours each day - starting at 5am will we enable watering
} sysStatus;

struct currentStatus_structure {                    // currently 10 bytes long
  int soilMoisture1;                                // Soil moisutre 0 if not connected
  int soilMoisture2;
  int pressure;                                     // Water line pressure
  int solenoidState;                                // Open (1) or closed (0)
  unsigned long lastCountTime;                      // When did we record our last count
  float temperature;                                // Current Temperature
  float humidity;                                   // Current Humidity
  float lightLevel;                                 // Light level in lux
  int alertCount;                                   // What is the current alert count
} current;

#define SEALEVELPRESSURE_HPA (1013.25)              // Universal variables

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, WATERING_STATE, REPORTING_STATE, RESP_WAIT_STATE, NAPPING_STATE, LOW_BATTERY_STATE};
char stateNames[9][14] = {"Initialize", "Error", "Idle", "Measuring", "Watering", "Reporting", "Response Wait", "Napping", "Low Battery"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

Timer wateringTimer(1200000, wateringTimerISR, true);     // Watering timer, calls the WateringTimerISR and is a one-shot timer
Timer awakeTimer(1800000, awakeTimerISR, true);           // 30 minute timer, calles the awakeTimerISR and is one-shot

// Pin Constants
const int blueLED =       D7;                     // This LED is on the Electron itself
const int userSwitch =    D4;                     // User switch with a pull-up resistor
const int soilPin1 =      A0;                     // First Soil Sensor
const int soilPin2 =      A1;                     // Second Soil Sensor
const int pressurePin =   A2;                     // Pressure Sensor
const int sensorShutdown =A5;                     // Disables the 5V sensors - Pressure / Soil1 and Soil 2
const int solEnablePin =  D3;                     // Active LOW - enables the solenoid
const int solDirection =  D2;                     // Soleniod direction HIGH = ON,


// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwake = 90000;              // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop

// Program Variables
bool dataInFlight = true;
bool volatile systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentStatusWriteNeeded = false;
bool volatile wateringTimerFlag = false;


// Variables Related To Particle Mobile Application Reporting
char SignalString[64];                     // Used to communicate Wireless RSSI and Description
char temperatureString[16];
char humidityString[16];
char soilMoisture1String[16];
char soilMoisture2String[16];
char waterPressureString[16];
char batteryString[8];
char batteryContextStr[16];                                        // One word that describes whether the device is getting power, charging, discharging or too cold to charge
char lightLevelString[16];
char wateringThresholdPctStr[8];
char lowPowerString[8];

// Time Period Related Variables
byte currentHourlyPeriod;                                         // This is where we will know if the period changed

// Battery monitoring
int lowBattLimit = 30;                                            // Trigger for Low Batt State

void setup()                                                      // Note: Disconnected Setup()
{
  char StartupMessage[64] = "Startup Successful";                 // Messages from Initialization
  state = IDLE_STATE;

  pinSetFast(solEnablePin);                                       // Make sure the Solenoid is off
  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(userSwitch,INPUT);                                      // Momentary contact button on board for direct user input
  pinMode(soilPin1, INPUT);
  pinMode(soilPin2, INPUT);
  pinMode(pressurePin, INPUT);
  pinMode(sensorShutdown, OUTPUT);
  digitalWrite(sensorShutdown,HIGH);                              // Enable the sensors
  pinMode(solEnablePin,OUTPUT);
  pinMode(solDirection,OUTPUT);
  digitalWrite(solEnablePin,HIGH);                               // Disables the solenoid valve
  digitalWrite(solDirection,LOW);                                // Set to close the valve

  char responseTopic[125];
  String deviceID = System.deviceID();                            // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);  // Subscribe to the integration response event

  Particle.variable("Signal", SignalString);                      // Particle variables that enable monitoring using the mobile app
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("StateOfChg", batteryString);
  Particle.variable("BatteryContext",batteryContextStr);
  Particle.variable("DurationMorning", sysStatus.wateringDurationMorning);
  Particle.variable("DurationLunch", sysStatus.wateringDurationLunch);
  Particle.variable("DurationEvening", sysStatus.wateringDurationEvening);
  Particle.variable("WateringThreshold",wateringThresholdPctStr);
  Particle.variable("Temperature", temperatureString);
  Particle.variable("Humidity", humidityString);
  Particle.variable("Luminosity",lightLevelString);
  Particle.variable("SoilMoisture1", current.soilMoisture1);
  Particle.variable("SoilMoisture2", current.soilMoisture2);
  Particle.variable("Pressure", current.pressure);
  Particle.variable("WaterWindow",sysStatus.wateringWindow);

  Particle.function("Measure-Now",measureNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Watering",controlValve);
  Particle.function("SetSoilSensors",setSoilSensors);
  Particle.function("SetPressureSensor", setPressureSensor);
  Particle.function("SetLightSensor",setLightSensor);
  Particle.function("SolenoidPresent",setSolenoidPresent);
  Particle.function("SetTempHumidSensor",setTempHumidSensor);
  Particle.function("SetDurationMorning", setWaterDurationMorning);
  Particle.function("SetDurationLunch", setWaterDurationLunch);
  Particle.function("SetDurationEvening", setWaterDurationEvening);
  Particle.function("SetWaterThreshold",setWaterThreshold);

  if (MemVersionNumber != EEPROM.read(MEM_MAP::versionAddr)) {          // Check to see if the memory map is the right version
    EEPROM.put(MEM_MAP::versionAddr,MemVersionNumber);
    for (int i=1; i < 0xF0; i++) {
      EEPROM.put(i,0);                                                  // Zero out the memory - new map or new device
    }
  }

  EEPROM.get(MEM_MAP::systemStatusAddr,sysStatus);                      // Load the System Status Object
  EEPROM.get(MEM_MAP::currentStatusAddr,current);

  if (!sysStatus.lowPowerMode) awakeTimer.start();                      // If we are not in low power mode, will start a 30 min timer then set it

  if (sysStatus.TempHumidConfig) {                                      // If there is a sensor present - initialize it
    if (!tempHumidSensor.begin(0x44)) {
      sysStatus.TempHumidConfig = false;                                // Set to 0x45 for alternate i2c addr - turns off the sensor if it fails to initalize
      strcpy(StartupMessage,"Temp/Humidity Sensor Failed to Inialize - disabling");
    } 
  }

  if (sysStatus.lightSensorConfig) {                                    // This will tell us if we need to initialize the sensor or not
    lightSensor.begin();
    lightSensor.set_sensor_mode(BH1750::forced_mode_high_res);
  }

  if (System.resetReason() == RESET_REASON_PIN_RESET) {                 // Check to see if we are starting from a pin reset
    sysStatus.resetCount++;
  }
  if (sysStatus.resetCount >=6) {                                       // If we get to sysStatus.resetCount 4, we are resetting without entering the main loop
    sysStatus.resetCount = 4;                                           // The hope here is to get to the main loop and report a value of 4 which will indicate this issue is occuring
    fullModemReset();                                                   // This will reset the modem and the device will reboot
  }

  if(sysStatus.solenoidConfig) {
    snprintf(wateringThresholdPctStr,sizeof(wateringThresholdPctStr),"%2.1f %%",sysStatus.wateringThresholdPct);
  }

  sysStatus.solenoidHoldTime = 6;                                      // Set a reasonable value - based on testing 8mSec

  if (sysStatus.solenoidConfig && current.solenoidState) controlValve("0");   // Can't start watering until we get to the main loop

  sysStatus.solarPowerMode = true;                                      // Set this as a default
  setPowerConfig();                                                     // Executes commands that set up the PMIC for Solar charging - once we know the Solar Mode

  if (!digitalRead(userSwitch)) setLowPowerMode("0");                   // Rescue mode to take out of low power mode and connect

  takeMeasurements();                                                   // For the benefit of monitoring the device

  if (sysStatus.stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;         // Only connect if we have battery
  else if(!connectToParticle()) {
    state = ERROR_STATE;                                                // We failed to connect can reset here or go to the ERROR state for remediation
    resetTimeStamp = millis();
    snprintf(StartupMessage, sizeof(StartupMessage), "Failed to connect");
  }

  if(Particle.connected() && sysStatus.verboseMode) Particle.publish("Startup",StartupMessage,PRIVATE);   // Let Particle know how the startup process went
  Serial.println(StartupMessage);

  Time.zone(2.0);                                                        // set timezone to Rwanda Time
  waitUntil(meterParticlePublish);                                                
  if(Particle.connected() && sysStatus.verboseMode) Particle.publish("Local Time",Time.timeStr(),PRIVATE);

  systemStatusWriteNeeded = true;                                       // likely something has changed
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (systemStatusWriteNeeded) {
      EEPROM.put(MEM_MAP::systemStatusAddr,sysStatus);
      systemStatusWriteNeeded = false;
    }
    if (currentStatusWriteNeeded) {
      EEPROM.put(MEM_MAP::currentStatusAddr ,current);
      currentStatusWriteNeeded = false;
    }
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp > stayAwake) && !current.solenoidState) state = NAPPING_STATE;    // These state assignments are in order of precedence
    if (Time.hour() != currentHourlyPeriod) state = MEASURING_STATE;                                                               // We want to report on the hour but not after bedtime
    if (sysStatus.stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;                                                        // The battery is low - sleep
    if (wateringTimerFlag) state = WATERING_STATE;                                                                                 // Most important - turn off water when done!
    break;

  case MEASURING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!takeMeasurements())
    {
      state = ERROR_STATE;
      resetTimeStamp = millis();
      if (sysStatus.verboseMode && Particle.connected()) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Error taking Measurements",PRIVATE);
      }
    }
    else if (sysStatus.solenoidConfig) state = WATERING_STATE;
    else state = REPORTING_STATE;
    break;

  case WATERING_STATE:                                                    // This state will examing soil values and decide on watering
    if (wateringTimerFlag) {                                              // Already watering - time to turn off the tap
      waitUntil(meterParticlePublish);
      Particle.publish("Watering","Done with watering cycle",PRIVATE);
      controlValve("0");
      wateringTimerFlag = false;
    }
    else if (Time.hour() != 8 && Time.hour() != 12 && Time.hour() != 17) {
      waitUntil(meterParticlePublish);
      Particle.publish("Watering","Not time to water",PRIVATE);
      if(current.solenoidState) controlValve("0");
    }
    else if (sysStatus.stateOfCharge < 50) {
      waitUntil(meterParticlePublish);
      Particle.publish("Watering","Watering Needed but battery too low",PRIVATE);
      if(current.solenoidState) controlValve("0");
    }
    else if (current.soilMoisture1 < sysStatus.wateringThresholdPct && !current.solenoidState) {  // Water if dry and if we are not already watering
      waitUntil(meterParticlePublish);
      Particle.publish("Watering","Watering needed - starting watering cycle",PRIVATE);
      controlValve("1");
      if (Time.hour() == 8) wateringTimer.changePeriod(sysStatus.wateringDurationMorning * 60 * 1000); // Start the timer to keep track of the watering time
      else if (Time.hour() == 12 ) wateringTimer.changePeriod(sysStatus.wateringDurationLunch * 60 * 1000);
      else wateringTimer.changePeriod(sysStatus.wateringDurationEvening * 60 * 1000);                                                
    }
    else {
      waitUntil(meterParticlePublish);
      Particle.publish("Watering","Watering not needed",PRIVATE);
      if(current.solenoidState) controlValve("0");
    }
    state = REPORTING_STATE;
    break;

  case REPORTING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (Time.hour() == 0) {
        sysStatus.verboseMode = false;                                    // Turn off Verbose mode
        Particle.syncTime();                                              // Set the clock each day
        current.alertCount = sysStatus.resetCount = 0;                    // Reset these each day as well
      }
      sendEvent();                                                        // Send data to Ubidots
      state = RESP_WAIT_STATE;                                            // Wait for Response
    }
    else if (!Particle.connected()) state = ERROR_STATE;                  // In case we timed out on our 1st attempt to connect
    break;

  case RESP_WAIT_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                // Response received back to IDLE state
    {
      stayAwakeTimeStamp = millis();
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      if (Particle.connected()) Particle.publish("spark/device/session/end", "", PRIVATE);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case NAPPING_STATE: {                                                // This state is triggered once the park closes and runs until it opens
    long secondsToHour;
    static bool pressureDetectedFlag = false;                          // Did we detect water pressure just before going to sleep
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();

    if (current.pressure > 1 && !pressureDetectedFlag) {               // If we detect pressure we will report again - once!
      pressureDetectedFlag = true;
      state = MEASURING_STATE;
      break;
    }

    if (Particle.connected()) {
      if (sysStatus.verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Taking a Nap",PRIVATE);
      }
      delay(1000);                                                      // Time to send last update
      disconnectFromParticle();                                         // If connected, we need to disconned and power down the modem
    }
    digitalWrite(blueLED,LOW);                                          // Turn off the LED
    digitalWrite(sensorShutdown,LOW);                                   // Turn off the sensors
    pressureDetectedFlag = false;
    secondsToHour = (60*(60 - Time.minute()));                     // Time till the top of the hour
    config.mode(SystemSleepMode::STOP).gpio(userSwitch,CHANGE).duration(secondsToHour * 1000);
    SystemSleepResult result = System.sleep(config);                    // Put the device to sleep
    if (result.wakeupPin() == userSwitch) setLowPowerMode("0");
    digitalWrite(blueLED,HIGH);                                         // On when the device is awake
    digitalWrite(sensorShutdown,HIGH);                                  // Turn on the sensors when awake
    if (!isDayTime() && Time.hour() % 2 == 0) {                         // At night, only connect every other hour
      connectToParticle();                                              // Wakey Wakey and get connected.
      state = IDLE_STATE;                                               // Awake now, we need to go back to the IDLE state for next tasking
    }
    else if (isDayTime()){                                              // During the day, connect every hour
      connectToParticle();                                              // Wakey Wakey and get connected.
      state = IDLE_STATE;                                               // Awake now, we need to go back to the IDLE state for next tasking
    }
    else state = NAPPING_STATE;                                         // Otherwise sleep
    } break;

  case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Low Battery - Sleeping",PRIVATE);
      delay(2000);                                                      // Time to send last update
      disconnectFromParticle();                                         // If connected, we need to disconned and power down the modem
    }
    digitalWrite(blueLED,LOW);                                          // Turn off the LED
    if (sysStatus.solenoidConfig) controlValve("0");                  // Make darn sure the water is off
    delay(5000);
    long secondsToHour = (60*(60 - Time.minute()));                     // Time till the top of the hour
    config.mode(SystemSleepMode::STOP).gpio(userSwitch,CHANGE).duration(secondsToHour * 1000);
    SystemSleepResult result = System.sleep(config);                    // Put the device to sleep
    state = IDLE_STATE;                                                 // Return to the IDLE_STATE
    } break;

  case ERROR_STATE:                                                     // To be enhanced - where we deal with errors
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (sysStatus.resetCount <= 3) {                                          // First try simple reset
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) Particle.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        sysStatus.resetCount = 0;      // Zero the sysStatus.resetCount
        EEPROM.put(MEM_MAP::systemStatusAddr,sysStatus);
        System.reset();
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the sysStatus.resetCount
        EEPROM.put(MEM_MAP::systemStatusAddr,sysStatus);
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
}

void sendEvent()
{
  char data[256];                                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"Temperature\":%4.1f, \"Humidity\":%4.1f, \"LightLevel\":%4.1f, \"Soilmoisture1\":%i, \"Soilmoisture2\":%i, \"waterPressure\":%i, \"Solenoid\":%i, \"battery\":%i, \"key1\":\"%s\", \"Resets\":%i, \"Alerts\":%i}", current.temperature, current.humidity, current.lightLevel, current.soilMoisture1, current.soilMoisture2, current.pressure, current.solenoidState, sysStatus.stateOfCharge, batteryContextStr, sysStatus.resetCount, current.alertCount );
  waitUntil(meterParticlePublish);
  // Particle.publish("Rwanda-Sense-And-Control-Elastic", data, PRIVATE);
  // waitUntil(meterParticlePublish);
  // Particle.publish("agriculture-aws-webhook",data,PRIVATE);
  // waitUntil(meterParticlePublish);
  Particle.publish("Rwanda-Sense-And-Control", data, PRIVATE);

  currentHourlyPeriod = Time.hour();                                      // Change the time period
  dataInFlight = true;                                                    // set the data inflight flag
  webhookTimeStamp = millis();
}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  waitUntil(meterParticlePublish);
  Particle.publish("Ubidots Hook", responseString, PRIVATE);
}

// These are the functions that are part of the takeMeasurements call

bool takeMeasurements() {
  // Read values from the sensor

  if (sysStatus.TempHumidConfig) {                                             // Only read the sensor if it is present
    if (isnan(tempHumidSensor.readTemperature())) current.temperature = current.humidity = 0;
    else {
    current.temperature = tempHumidSensor.readTemperature();
    current.humidity = tempHumidSensor.readHumidity();
    }

  }
  else current.temperature = current.humidity = 0.0;
  snprintf(temperatureString,sizeof(temperatureString), "%4.1f C", current.temperature);
  snprintf(humidityString,sizeof(humidityString), "%4.1f %%", current.humidity);

  if (sysStatus.lightSensorConfig) {
    lightSensor.make_forced_measurement();
    current.lightLevel = lightSensor.get_light_level();
  }
  else current.lightLevel = 0.0;
  snprintf(lightLevelString, sizeof(lightLevelString), "%4.1f lux", current.lightLevel);

  if (sysStatus.soilSensorConfig >= 1) current.soilMoisture1 = map(analogRead(soilPin1),0,3722,0,100);             // Sensor puts out 0-3V for 0% to 100% soil moisuture
  else current.soilMoisture1 = 0;
  if (sysStatus.soilSensorConfig == 2)  current.soilMoisture2 = map(analogRead(soilPin2),0,3722,0,100);
  else current.soilMoisture2 = 0;


  if (sysStatus.pressureSensorConfig == 1) current.pressure = map(analogRead(pressurePin),428,2816,0,30);         // Sensor range is 0.5V (0 psi) to 4.5V (30psi) and there is a voltage divider (330 / 480) so...
  else sysStatus.pressureSensorConfig = 0;

  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready

  // sysStatus.stateOfCharge = int(System.batteryCharge());                       // Percentage of full charge
  sysStatus.stateOfCharge = int(System.batteryCharge());
  snprintf(batteryString, sizeof(batteryString), "%i %%", sysStatus.stateOfCharge);

  getBatteryContext();                                                 // What is the battery doing.

  systemStatusWriteNeeded = currentStatusWriteNeeded = true;
  return 1;
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}


void getBatteryContext() {
  const char* batteryContext[7] ={"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};
  // Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-

  snprintf(batteryContextStr, sizeof(batteryContextStr),"%s", batteryContext[System.batteryState()]);

}


// These functions control the connection and disconnection from Particle
bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    Particle.process();
  }
  if (Particle.connected()) return 1;                               // Were able to connect successfully
  else return 0;                                                    // Failed to connect
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  Cellular.off();
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
    return !Particle.connected();
}


// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.

int measureNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

bool isDayTime() {
  if (Time.hour() >= 19) return 0;
  else if (Time.hour() < 6) return 0;
  else return 1;
}

// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration

  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(900) // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(1024) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    enableCharging(true);
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
        .powerSourceMinVoltage(4208)                                     // This is the default value for the Boron
        .batteryChargeCurrent(900)                                      // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                      // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    enableCharging(true);
    return res;
  }
}

bool enableCharging(bool enableCharge)
{
  PMIC pmic(true);
  if(enableCharge) {
    pmic.enableCharging();
    return TRUE;
  }
  else {
    pmic.disableCharging();
    return FALSE;
  }
}


int setSoilSensors (String command) // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.soilSensorConfig = 0;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","No Soil Sensors",PRIVATE);
    return 1;
  }
  else if (command == "1")
  {
    sysStatus.soilSensorConfig = 1;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","One Soil Sensor",PRIVATE);
    return 1;
  }
    else if (command == "2")
  {
    sysStatus.soilSensorConfig = 2;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","Two Soil Sensors",PRIVATE);
    return 1;
  }
  else return 0;
}

int setPressureSensor (String command) // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.pressureSensorConfig = 0;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","No Pressure Sensor",PRIVATE);
    return 1;
  }
  else if (command == "1")
  {
    sysStatus.pressureSensorConfig = 1;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","Pressure Sensor Present",PRIVATE);
    return 1;
  }
  else return 0;
}

int setLightSensor (String command) // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.lightSensorConfig = 0;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","No Light Sensor",PRIVATE);
    return 1;
  }
  else if (command == "1")
  {
    sysStatus.lightSensorConfig = 1;
    systemStatusWriteNeeded = true;
    lightSensor.begin();
    lightSensor.set_sensor_mode(BH1750::forced_mode_high_res);
    Particle.publish("Config","Light Sensor Present",PRIVATE);
    return 1;
  }
  else return 0;
}

int setTempHumidSensor (String command) // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.TempHumidConfig = 0;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","No Temp / Humidity Sensor Present",PRIVATE);
    return 1;
  }
  else if (command == "1")
  {
    sysStatus.TempHumidConfig = 1;
    systemStatusWriteNeeded = true;
    tempHumidSensor.begin(0x44);                                        // Set to 0x45 for alternate i2c addr
    Particle.publish("Config","Temp / Humidity Sensor Present",PRIVATE);
    return 1;
  }
  else return 0;
}

int setSolenoidPresent(String command) // Function to force sending data in current hour
{
  controlValve("0");                                            // Make sure it is turned off
  if (command == "1") {
    sysStatus.solenoidConfig = 1;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","Solenoid Attached",PRIVATE);
    return 1;
  }
  else if (command == "0") {
    sysStatus.solenoidConfig = 0;
    systemStatusWriteNeeded = true;
    Particle.publish("Config","No Solenoid Attached",PRIVATE);
    return 1;
  }
  else return 0;
}

int setWaterDurationMorning(String command)
{
  char * pEND;
  char data[256];
  int tempDuration = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempDuration < 1) || (tempDuration > 55)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringDurationMorning = tempDuration;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Morning Watering Duration set to %i",sysStatus.wateringDurationMorning);

  if (wateringTimer.isActive()){                                          // We can change the period of a running timer
    wateringTimer.changePeriod(1000*60*sysStatus.wateringDurationMorning);
  }

  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Duration",data, PRIVATE);
  return 1;
}

int setWaterDurationLunch(String command)
{
  char * pEND;
  char data[256];
  int tempDuration = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempDuration < 1) || (tempDuration > 55)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringDurationLunch = tempDuration;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Lunch Watering Duration set to %i",sysStatus.wateringDurationLunch);

  if (wateringTimer.isActive()){                                          // We can change the period of a running timer
    wateringTimer.changePeriod(1000*60*sysStatus.wateringDurationLunch);
  }

  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Duration",data, PRIVATE);
  return 1;
}

int setWaterDurationEvening(String command)
{
  char * pEND;
  char data[256];
  int tempDuration = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempDuration < 1) || (tempDuration > 55)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringDurationEvening = tempDuration;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Evening Watering Duration set to %i",sysStatus.wateringDurationEvening);

  if (wateringTimer.isActive()){                                          // We can change the period of a running timer
    wateringTimer.changePeriod(1000*60*sysStatus.wateringDurationEvening);
  }

  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Duration",data, PRIVATE);
  return 1;
}

int setWaterWindow(String command)
{
  char * pEND;
  char data[256];
  int tempWindow = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempWindow < 1) || (tempWindow > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringWindow = tempWindow;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Watering begins at 5am for %i hours",sysStatus.wateringWindow);

  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Window",data, PRIVATE);
  return 1;
}

int setWaterThreshold(String command)                                       // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  float tempThreshold = strtof(command,&pEND);                        // Looks for the first float and interprets it
  if ((tempThreshold < 0.0) | (tempThreshold > 100.0)) return 0;        // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringThresholdPct = tempThreshold;                          // debounce is how long we must space events to prevent overcounting
  systemStatusWriteNeeded = true;
  snprintf(wateringThresholdPctStr,sizeof(wateringThresholdPctStr),"%2.1f %%",sysStatus.wateringThresholdPct);
  if (sysStatus.verboseMode && Particle.connected()) {                                                  // Publish result if feeling verbose
    waitUntil(meterParticlePublish);
    Particle.publish("Threshold",wateringThresholdPctStr, PRIVATE);
  }
  return 1;                                                           // Returns 1 to let the user know if was reset
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    Particle.publish("Mode","Set Verbose Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    Particle.publish("Mode","Cleared Verbose Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode","Low Power Mode", PRIVATE);
    }
    sysStatus.lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!Particle.connected()) {                                      // In case we are not connected, we will do so now.
      connectToParticle();
      sysStatus.connectedStatus = true;
    }
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Normal Operations", PRIVATE);
    delay(1000);                                                      // Need to make sure the message gets out.
    awakeTimer.start();                                               // Wake for 30 minutes - then back to low power mode.  Resets timer if already running
    sysStatus.lowPowerMode = false;                                   // update the system variable to reflect the new lowPowerMode
  }
  systemStatusWriteNeeded = true;
  return 1;
}

int controlValve(String command)                                   // Function to force sending data in current hour
{
  if (command != "1" && command != "0") return 0;              // Before we begin, let's make sure we have a valid input
  else if (command == "1") {                                     // Open the water valve
    current.solenoidState = true;
    digitalWrite(solDirection,HIGH);                              // Open the valve
    digitalWrite(solEnablePin,LOW);                               // Enable the solenoid
    delay(sysStatus.solenoidHoldTime);
    digitalWrite(solEnablePin,HIGH);                              // Diable the solenoid
    delay(1000);
    digitalWrite(solEnablePin,LOW);                               // Enable the solenoid
    delay(sysStatus.solenoidHoldTime);
    digitalWrite(solEnablePin,HIGH);                              // Diable the solenoid
    Particle.publish("Watering","Open the Valve",PRIVATE);
  }
  else {                                                          // Close the water valve
    digitalWrite(solDirection,LOW);                               // Close the valve
    digitalWrite(solEnablePin,LOW);                               // Enable the solenoid
    delay(sysStatus.solenoidHoldTime);
    digitalWrite(solEnablePin,HIGH);                              // Diable the solenoid
    delay(1000);
    digitalWrite(solEnablePin,LOW);                               // Enable the solenoid
    delay(sysStatus.solenoidHoldTime);
    digitalWrite(solEnablePin,HIGH);                              // Diable the solenoid
    current.solenoidState = false;
    Particle.publish("Watering","Close the valve",PRIVATE);
  }
  currentStatusWriteNeeded = true;
  return true;
}

void wateringTimerISR() {
  wateringTimerFlag = true;
}

void awakeTimerISR() {
  sysStatus.lowPowerMode = true;
  systemStatusWriteNeeded = true;
}


void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("State Transition",stateTransitionString, PRIVATE);
  }
}

bool meterParticlePublish(void)
{
  static unsigned long lastPublish = 0;
  if(millis() - lastPublish >= 1000) {                            // Particle requires metering to once per second
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample

	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

