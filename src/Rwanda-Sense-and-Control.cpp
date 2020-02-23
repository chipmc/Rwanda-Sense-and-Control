/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Rwanda-Sense-and-Control/src/Rwanda-Sense-and-Control.ino"
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

// Particle Product definitions
void setup();
void loop();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
bool takeMeasurements();
void getSignalStrength();
bool connectToParticle();
bool disconnectFromParticle();
bool notConnected();
void PMICreset();
inline void softDelay(uint32_t t);
int measureNow(String command);
int setSolarMode(String command);
int setVerboseMode(String command);
int setTimeZone(String command);
int setLowPowerMode(String command);
int setSolenoid(String command);
void publishStateTransition(void);
bool meterParticlePublish(void);
void fullModemReset();
#line 23 "/Users/chipmc/Documents/Maker/Particle/Projects/Rwanda-Sense-and-Control/src/Rwanda-Sense-and-Control.ino"
PRODUCT_ID(10709);                                   // Connected Counter Header
PRODUCT_VERSION(11);
const char releaseNumber[4] = "11";                  // Displays the release on the menu 


// Included Libraries
#include "math.h"
#include "adafruit-sht31.h"
#include "DevicePinoutdoc.h"

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);          // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);               // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;             // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                           // Initalize the PMIC class so you can call the Power Management functions below.
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // Temp and Humidity Sensor - Grove connected on i2c

namespace MEM_MAP {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x0,                    // Where we store the memory map version number - 8 Bits
    alertCountAddr        = 0x1,                    // Where we store our current alert count - 8 Bits
    resetCountAddr        = 0x2,                    // This is where we keep track of how often the Electron was reset - 8 Bits
    timeZoneAddr          = 0x3,                    // Store the local time zone data - 8 Bits
    controlRegisterAddr   = 0x4,                    // This is the control register for storing the current state - 8 Bits
    currentCountsTimeAddr = 0x5,                    // Time of last report - 32 bits
  };
};

#define SEALEVELPRESSURE_HPA (1013.25)              // Universal variables
#define MEMORYMAPVERSION 1                          // Lets us know if we need to reinitialize the memory map

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, REPORTING_STATE, RESP_WAIT_STATE, SLEEPING_STATE, LOW_BATTERY_STATE};
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Measuring", "Reporting", "Response Wait", "Sleeping", "Low Battery"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

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
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
int alertCount;                                     // Keeps track of non-reset issues - think of it as an indication of health
bool waiting = false;
bool dataInFlight = true;
byte controlRegister;                               // Stores the control register values
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup

// Variables Related To Particle Mobile Application Reporting
char SignalString[64];                     // Used to communicate Wireless RSSI and Description
const char* radioTech[8] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154"};
char temperatureString[16];
char humidityString[16];
char soilMoisture1String[16];
char soilMoisture2String[16];
char waterPressureString[16];
char batteryString[16];
char powerContext[24];                              // One word that describes whether the device is getting power, charging, discharging or too cold to charge

// Time Period Related Variables
byte currentHourlyPeriod;                           // This is where we will know if the period changed

// Battery monitoring
int stateOfCharge = 0;                              // Stores battery charge level value
int lowBattLimit;                                   // Trigger for Low Batt State
bool lowPowerMode;                                  // Flag for Low Power Mode operations

// This section is where we will initialize sensor specific variables, libraries and function prototypes
float temperatureInC = 0;                           // Temp / Humidity Sensor variables
float relativeHumidity = 0;
int soilMoisture1 = 0;                               // Soil sensor variables
int soilMoisture2 = 0;                               // Soil sensor variables
int waterPressure = 0;                               // Water Pressure Value (0-5PSI)
int solenoidState = 0;                               // Solenoid State (-1 close, 0 disabled, 1 open)

void setup()                                                      // Note: Disconnected Setup()
{
  char StartupMessage[64] = "Startup Successful";                 // Messages from Initialization
  state = IDLE_STATE;

  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(userSwitch,INPUT);                                      // Momentary contact button on board for direct user input
  pinMode(soilMoisture1, INPUT);
  pinMode(soilMoisture2, INPUT);
  pinMode(pressurePin, INPUT);
  pinMode(sensorShutdown, OUTPUT);
  digitalWrite(sensorShutdown,HIGH);
  pinMode(solEnablePin,OUTPUT);                                     
  pinMode(solDirection,OUTPUT);                                      
  digitalWrite(solEnablePin,HIGH);                               // Disables the solenoid valve
  digitalWrite(solDirection,LOW);                                // Set to close the valve

  char responseTopic[125];
  String deviceID = System.deviceID();                            // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);  // Subscribe to the integration response event

  Particle.variable("Signal", SignalString);                      // Particle variables that enable monitoring using the mobile app
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("StateOfChg", batteryString);
  Particle.variable("PowerContext",powerContext);
  Particle.variable("LowPowerMode",lowPowerMode);
  Particle.variable("Temperature", temperatureString);
  Particle.variable("Humidity", humidityString);
  Particle.variable("SoilMoisture1", soilMoisture1);
  Particle.variable("SoilMoisture2", soilMoisture2);
  Particle.variable("Pressure", waterPressure);

  Particle.function("Measure-Now",measureNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("SetTimeZone",setTimeZone);
  Particle.function("SetSolenoid",setSolenoid);

  if (MEMORYMAPVERSION != EEPROM.read(MEM_MAP::versionAddr)) {          // Check to see if the memory map is the right version
    EEPROM.put(MEM_MAP::versionAddr,MEMORYMAPVERSION);
    for (int i=1; i < 10; i++) {
      EEPROM.put(i,0);                                                  // Zero out the memory - new map or new device
    }
  }

  if (!sht31.begin(0x44)) {                                             // Set to 0x45 for alternate i2c addr
    snprintf(StartupMessage,sizeof(StartupMessage),"Could not find SHT31");
    resetTimeStamp = millis();
    state = ERROR_STATE;
  }

  resetCount = EEPROM.read(MEM_MAP::resetCountAddr);                    // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    EEPROM.write(MEM_MAP::resetCountAddr, resetCount);                  // If so, store incremented number - watchdog must have done This
  }
  if (resetCount >=6) {                                                 // If we get to resetCount 4, we are resetting without entering the main loop
    EEPROM.write(MEM_MAP::resetCountAddr,4);                            // The hope here is to get to the main loop and report a value of 4 which will indicate this issue is occuring
    fullModemReset();                                                   // This will reset the modem and the device will reboot
  }

  int8_t tempTimeZoneOffset = EEPROM.read(MEM_MAP::timeZoneAddr);       // Load Time zone data from FRAM
  if (tempTimeZoneOffset <= 12 && tempTimeZoneOffset >= -12)  Time.zone((float)tempTimeZoneOffset);  // Load Timezone from FRAM
  else Time.zone(0.0);                                                    // Default is GMT in case proper value not in EEPROM

  // And set the flags from the control register
  controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);          // Read the Control Register for system modes so they stick even after reset
  lowPowerMode    = (0b00000001 & controlRegister);                     // Set the lowPowerMode
  solarPowerMode  = (0b00000100 & controlRegister);                     // Set the solarPowerMode
  verboseMode     = (0b00001000 & controlRegister);                     // Set the verboseMode
  
  PMICreset();                                                          // Executes commands that set up the PMIC for Solar charging - once we know the Solar Mode

  if (!digitalRead(userSwitch)) {                                       // Rescue mode to locally take lowPowerMode so you can connect to device
    lowPowerMode = false;                                               // Press the user switch while resetting the device
    controlRegister = (0b11111110 & controlRegister);                   // Turn off Low power mode
    EEPROM.write(controlRegister,MEM_MAP::controlRegisterAddr);         // Write to the EEMPROM
    snprintf(StartupMessage, sizeof(StartupMessage), "User Button - Detected");
  }

  takeMeasurements();                                                   // For the benefit of monitoring the device

  if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;         // Only connect if we have battery
  else if(!connectToParticle()) {
    state = ERROR_STATE;                                                // We failed to connect can reset here or go to the ERROR state for remediation
    resetTimeStamp = millis();
    snprintf(StartupMessage, sizeof(StartupMessage), "Failed to connect");
  }

  if(Particle.connected() && verboseMode) Particle.publish("Startup",StartupMessage,PRIVATE);   // Let Particle know how the startup process went
    Serial.println(StartupMessage);
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = SLEEPING_STATE;
    if (Time.hour() != currentHourlyPeriod) state = MEASURING_STATE;     // We want to report on the hour but not after bedtime
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;        // The battery is low - sleep
    break;

  case MEASURING_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!takeMeasurements())
    {
      state = ERROR_STATE;
      resetTimeStamp = millis();
      if (verboseMode && Particle.connected()) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Error taking Measurements",PRIVATE);
      }
    }
    else state = REPORTING_STATE;
    break;

  case REPORTING_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (Time.hour() == 12) Particle.syncTime();                         // Set the clock each day at noon
      sendEvent();                                                        // Send data to Ubidots
      state = RESP_WAIT_STATE;                                            // Wait for Response
    }
    else if (!connectToParticle()) state = ERROR_STATE;                   // In case we timed out on our 1st attempt to connect
    break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
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

  case SLEEPING_STATE: {                                                // This state is triggered once the park closes and runs until it opens
    if (verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Going to Sleep",PRIVATE);
      }
      delay(1000);                                                      // Time to send last update
      disconnectFromParticle();                                         // If connected, we need to disconned and power down the modem
    }
    digitalWrite(blueLED,LOW);                                          // Turn off the LED
    long secondsToHour = (60*(60 - Time.minute()));                     // Time till the top of the hour
    System.sleep(userSwitch, FALLING, secondsToHour);                   // Sleep till the next hour, then wakes and continues execution - Stop mode
    digitalWrite(blueLED,HIGH);                                         // On when the device is awake
    connectToParticle();                                                // Wakey Wakey and get connected.
    state = IDLE_STATE;                                                 // Awake now, we need to go back to the IDLE state for next tasking
    } break;

  case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
    if (verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Low Battery - Sleeping",PRIVATE);
      }
      delay(1000);                                                      // Time to send last update
      disconnectFromParticle();                                         // If connected, we need to disconned and power down the modem
    }
    digitalWrite(blueLED,LOW);                                          // Turn off the LED
    int secondsToHour = (60*(60 - Time.minute()));                      // Time till the top of the hour
    System.sleep(userSwitch,FALLING,secondsToHour);                     // Very deep sleep till the next hour - then resets
    state = IDLE_STATE;                                                 // Return to the IDLE_STATE
    } break;

  case ERROR_STATE:                                                     // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (resetCount <= 3) {                                          // First try simple reset
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - EEPROM.read(MEM_MAP::currentCountsTimeAddr) > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) Particle.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        EEPROM.write(MEM_MAP::resetCountAddr,0);                           // Zero the ResetCount
        System.reset();
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        EEPROM.write(MEM_MAP::resetCountAddr,0);                           // Zero the ResetCount
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
}

void sendEvent()
{
  char data[256];                                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"Temperature\":%4.1f, \"Humidity\":%4.1f, \"Soilmoisture1\":%i, \"Soilmoisture2\":%i, \"waterPressure\":%i, \"Solenoid\":%i, \"Battery\":%i, \"Resets\":%i, \"Alerts\":%i}", temperatureInC, relativeHumidity, soilMoisture1, soilMoisture2, waterPressure, solenoidState, stateOfCharge, resetCount, alertCount);
  Particle.publish("Rwanda_Irrigation_Hook", data, PRIVATE);
  currentHourlyPeriod = Time.hour();                                      // Change the time period
  dataInFlight = true;                                                    // set the data inflight flag
  webhookTimeStamp = millis();
}

void UbidotsHandler(const char *event, const char *data)              // Looks at the response from Ubidots - Will reset Photon if no successful response
{                                                                     // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                      // data needs to be copied since if (Particle.connected()) Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                          // Copy - overflow safe
  if (!strlen(dataCopy)) {                                            // First check to see if there is any data
    if (Particle.connected()) Particle.publish("Ubidots Hook", "No Data", PRIVATE);
    return;
  }
  int responseCode = atoi(dataCopy);                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if (Particle.connected()) Particle.publish("State","Response Received", PRIVATE);
    EEPROM.write(MEM_MAP::currentCountsTimeAddr,Time.now());          // Record the last successful Webhook Response
    dataInFlight = false;                                             // Data has been received
  }
  else if (Particle.connected()) Particle.publish("Ubidots Hook", dataCopy, PRIVATE);                    // Publish the response code
}

// These are the functions that are part of the takeMeasurements call

bool takeMeasurements() {
  // Read values from the sensor
  temperatureInC = sht31.readTemperature();
  snprintf(temperatureString,sizeof(temperatureString), "%4.1f C", temperatureInC);

  relativeHumidity = sht31.readHumidity();
  snprintf(humidityString,sizeof(humidityString), "%4.1f %%", relativeHumidity);

  soilMoisture1 = map(analogRead(soilPin1),0,3722,0,100);             // Sensor puts out 0-3V for 0% to 100% soil moisuture
  soilMoisture2 = map(analogRead(soilPin2),0,3722,0,100);
  waterPressure = map(analogRead(pressurePin),428,2816,0,30);         // Sensor range is 0.5V (0 psi) to 4.5V (30psi) and there is a voltage divider (330 / 480) so...

  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  stateOfCharge = int(batteryMonitor.getSoC());                       // Percentage of full charge
  snprintf(batteryString, sizeof(batteryString), "%i %%", stateOfCharge);

  if (temperatureInC < 0 || temperatureInC > 45) {                      // Need to add temp charging controls - 
    snprintf(powerContext, sizeof(powerContext), "Chg Disabled Temp");
    power.disableCharging();                                          // Disable Charging if temp is too low or too high
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Alert", "Charging disabled Temperature",PRIVATE);
  }

  return 1;
}

void getSignalStrength()
{
  // New Boron capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
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

// Power Management function
void PMICreset() {
  power.begin();                                            // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    lowBattLimit = 20;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4840);                       // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                        // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                    // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                           // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    lowBattLimit = 30;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4208);                       // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                       // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                    // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                           // default is 4.112V termination voltage
  }
}


// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.

inline void softDelay(uint32_t t) {
  for (uint32_t ms = millis(); millis() - ms < t; Particle.process());  //  safer than a delay()
}

int measureNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    solarPowerMode = true;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b00000100 | controlRegister);          // Turn on solarPowerMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister);// Write it to the register
    PMICreset();                                               // Change the power management Settings
    Particle.publish("Mode","Set Solar Powered Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b11111011 & controlRegister);           // Turn off solarPowerMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    PMICreset();                                                // Change the power management settings
    Particle.publish("Mode","Cleared Solar Powered Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    Particle.publish("Mode","Set Verbose Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    Particle.publish("Mode","Cleared Verbose Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  EEPROM.write(MEM_MAP::timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  Particle.publish("Time",data,PRIVATE);
  delay(1000);
  Particle.publish("Time",Time.timeStr(),PRIVATE);
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    Particle.publish("Mode","Low Power",PRIVATE);
    controlRegister = (0b00000001 | controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    Particle.publish("Mode","Normal Operations",PRIVATE);
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
  return 1;
}

int setSolenoid(String command) // Function to force sending data in current hour
{
  if (command == "1") {                               // Open the water valve
    digitalWrite(solEnablePin,LOW);                       // Enable the solenoid
    digitalWrite(solDirection,HIGH);                         // Open the valve
    Particle.publish("Solenoid","Open the Valve",PRIVATE);
    return 1;
  }
  else if (command == "0") {                          // Disable Solenoid (neither opens or closes)
    digitalWrite(solEnablePin,HIGH);                       // disable the solenoid
    Particle.publish("Solenoid","Value Control Disabled",PRIVATE);
    return 1;
  }
  else if (command == "-1") {                         // Close the water valve
    digitalWrite(solEnablePin,LOW);                       // Enable the solenoid
    digitalWrite(solDirection,LOW);                         // Open the valve
    Particle.publish("Solenoid","Close the valve",PRIVATE);
    return 1;
  }
  else return 0;
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
  Serial.println(stateTransitionString);
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
