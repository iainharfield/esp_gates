// ##include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <time.h>

#include "hh_defines.h"
#include "hh_utilities.h"
#include "hh_cntrl.h"

#include <AsyncMqttClient_Generic.hpp>

#define ESP8266_DRD_USE_RTC true
#define ESP_DRD_USE_LITTLEFS false
#define ESP_DRD_USE_SPIFFS false
#define ESP_DRD_USE_EEPROM false
#define DOUBLERESETDETECTOR_DEBUG true
#include <ESP_DoubleResetDetector.h>

//***********************
// Template functions
//***********************
bool onMqttMessageAppExt(char *, char *, const AsyncMqttClientMessageProperties &, const size_t &, const size_t &, const size_t &);
bool onMqttMessageAppCntrlExt(char *, char *, const AsyncMqttClientMessageProperties &, const size_t &, const size_t &, const size_t &);
void appMQTTTopicSubscribe();
void telnet_extension_1(char);
void telnet_extension_2(char);
void telnet_extensionHelp(char);
void startTimesReceivedChecker();
void processCntrlTOD_Ext();
// void app_WD_on(void *);
// void app_WE_off(cntrlState &);
// void app_WD_on(cntrlState &);
// void app_WE_off(cntrlState &);
// void app_WD_auto(cntrlState &);
// void app_WE_auto(cntrlState &);

//*************************************
// defined in asyncConnect.cpp
//*************************************
extern void mqttTopicsubscribe(const char *topic, int qos);
extern void platform_setup(bool);
extern void handleTelnet();
extern void printTelnet(String);
extern AsyncMqttClient mqttClient;
extern void wifiSetupConfig(bool);
extern templateServices coreServices;
extern char ntptod[MAX_CFGSTR_LENGTH];

//*************************************
// defined in cntrl.cpp
//*************************************
cntrlState GateCntrlState; // Create a controller

#define CmdStateWD "/house/cntrl/outside-gates-front/wd-command"			// UI Button press, ON, OFF, NEXT, SET
#define CmdStateWE "/house/cntrl/outside-gates-front/we-command"			// UI Button press
#define CommandWDTimes "/house/cntrl/outside-gates-front/wd-control-times"  // Times message from either UI or Python app
#define CommandWETimes "/house/cntrl/outside-gates-front/we-control-times"  // Times message from either UI or MySQL via Python app
#define GateState "/house/cntrl/outside-gates-front/state"					// The State of the Gates "AUTO", "MAN", "OPEN", "CLOSE"

#define WDBypass "/house/cntrl/outside-gates-front/wd-bypass-control-times" // if BYPASS then bypass WD control times - stays closed
#define WEBypass "/house/cntrl/outside-gates-front/we-bypass-control-times" // if BYPASS then bypass WE control times - stays closed


//        Type string : outsidegates-front-state [stateTopic="/house/cntrl/outside-gates-front/state", commandTopic="/house/cntrl/outside-gates-front/state"]
//        Type string : outsidegates-front-lwt [stateTopic="/house/cntrl/outside-gates-front/lwt"]
//        Type string : outsidegates-front-identity [stateTopic="/house/cntrl/outside-gates-front/identity"]
//        Type string : outsidegates-front-command-wd [stateTopic="/house/cntrl/outside-gates-front/wd-command", commandTopic="/house/cntrl/outside-gates-front/wd-command"]
//        Type string : outsidegates-front-command-we [stateTopic="/house/cntrl/outside-gates-front/we-command", commandTopic="/house/cntrl/outside-gates-front/we-command"]
//        Type string : outsidegates-front-times-wd [stateTopic="/house/cntrl/outside-gates-front/wd-control-times", commandTopic="/house/cntrl/outside-gates-front/wd-control-times"]
//        Type string : outsidegates-front-times-we [stateTopic="/house/cntrl/outside-gates-front/we-control-times", commandTopic="/house/cntrl/outside-gates-front/we-control-times"]
//        Type string : outsidegates-front-manual-state [stateTopic="/house/cntrl/outside-gates-front/manual-state"]

#define RefreshID "FRONTGATES"

// Double Reset Detection configuration
#define DRD_TIMEOUT 3
#define DRD_ADDRESS 0

DoubleResetDetector *drd;

// defined in telnet.cpp
extern int reporting;
extern bool telnetReporting;

//************************
// Application specific
//************************
String deviceName = "Front-Gates";
String deviceType = "CNTRL";
String app_id     = "GATES"; 			// configure

bool processCntrlMessageApp_Ext(char *, const char *, const char *, const char *);
void processAppTOD_Ext();
void setGates(void *cid, int state, String stateMsg, int gateDemand);

#define relay_pin 		13		// Gate open or close
#define GATEManualStatus 	12		// Manual over ride.  If low then gates held open manually

int gateDemand  = 0;
/************************************
 * Define relay states to demand heat
 ************************************/
#define CLOSE  0	 // pin LOW
#define OPEN   1   // pin HIGH

devConfig espDevice;

bool bManMode = false;

void setup()
{
	//***************************************************
	// Set-up Platform - hopefully dont change this
	//***************************************************
	bool configWiFi = false;
	Serial.begin(115200);
	while (!Serial)
		delay(300);

	espDevice.setup(deviceName, deviceType);
	Serial.println("\nStarting Front Gates Controller on ");
	Serial.println(ARDUINO_BOARD);

	drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
	if (drd->detectDoubleReset())
	{
		configWiFi = true;
	}

	// this app is a contoller
	// configure the MQTT topics for the Controller
	GateCntrlState.setCntrlObjRef(GateCntrlState);
	GateCntrlState.setCntrlName((String)app_id + "FRONT");
	GateCntrlState.setRefreshID(RefreshID);

	// Platform setup: Set up and manage: WiFi, MQTT and Telnet
	platform_setup(configWiFi);

	//***********************
	// Application setup
	//***********************
	// Define the output pins
    pinMode(relay_pin, OUTPUT);

	// Define the input pins
    pinMode(GATEManualStatus, INPUT);

	// Intialise start up status
	digitalWrite(12, LOW);

}

void loop()
{
	// char logString[MAX_LOGSTRING_LENGTH];
	drd->loop();

	// Go look for OTA request
	ArduinoOTA.handle();

	handleTelnet();
	
  	if (digitalRead(GATEManualStatus) == 1) 				// means manual switch ON and gates forced to stay open
  	{
  		//bManMode = true;
  		//gateControl(relay_pin, HIGH);

		if (bManMode == false)
		{
  			mqttLog("Manually held open", REPORT_WARN, true, true);
			mqttClient.publish(GateState, 1, true, "MAN"); // not sure this will work as whan not MANual what is state?
			bManMode = true;  // this just to avoid mqttLog from spuing out gzillions of messages in the loop
		}

  		//client.publish(outTopicGateManual, "MAN");
		//mqttClient.publish(GateState, 1, true, "MAN"); // not sure this will work as whan not MANual what is state?
  		//client.publish(outTopic, "MAN");
  	}
  	else
  	{
  		//client.publish(outTopicGateManual, "AUTO");
  		//processState();
  		// FIX THIS - Need execute a run against time of day to decide whether the gate opens of close - or can we wait unti next TOD?
		if (bManMode == true)
		{
			bManMode = false; // this just to avoid mqttLog from spuing out gzillions of messages in the loop
			GateCntrlState.processCntrlTOD_Ext();
		}
  	}
}

//****************************************************************
// Process any application specific inbound MQTT messages
// Return False if none
// Return true if an MQTT message was handled here
//****************************************************************
bool onMqttMessageAppExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
{
	(void)payload;

	char mqtt_payload[len + 1];
	mqtt_payload[len] = '\0';
	strncpy(mqtt_payload, payload, len);

	mqttLog(mqtt_payload, REPORT_DEBUG, true, true);

	// Prceess the messages for each controller created
	GateCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);

	return false; // FIXTHIS - I thiink I need to return true is a message is processed
}

void processAppTOD_Ext()
{
	mqttLog("FRONTGATES Application Processing TOD", REPORT_INFO, true, true);
}

bool processCntrlMessageApp_Ext(char *mqttMessage, const char *onMessage, const char *offMessage, const char *commandTopic)
{
	//String msg = "Application Specific message  handling: " + (String)mqttMessage ;
	//mqttLog(msg.c_str(), true, true);
	if (strcmp(mqttMessage, "SET") == 0)
	{
		// mqttClient.publish(StateUpstairsRuntime,1, true, "AUTO");			// This just sets the UI to show that MAN start is OFF
		return true;
	}
	return false;
}

// Subscribe to application specific topics
// the Framework will handle these messages
void appMQTTTopicSubscribe()
{
	//GateCntrlState.setWDUIcommandStateTopic(StateDownstairsRuntime);
	GateCntrlState.setWDCntrlTimesTopic(CommandWDTimes);
	GateCntrlState.setWDUIcommandStateTopic(CmdStateWD);
	//GateCntrlState.setWECntrlRunTimesStateTopic(StateDownstairsRuntime);
	GateCntrlState.setWECntrlTimesTopic(CommandWETimes);
	GateCntrlState.setWEUIcommandStateTopic(CmdStateWE);
	GateCntrlState.setWDBypassTimesTopic(WDBypass);
	GateCntrlState.setWEBypassTimesTopic(WEBypass);
}

void app_WD_on(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	
	if (coreServices.getWeekDayState() == 1)			// 1 means weekday
	{
		if (obj->getWDBypassMode() == false)			// If in Bypass mode ignore action
		{
			setGates(cid, OPEN, "WD OPEN", relay_pin);
		}
	}	
}

void app_WD_off(void *cid)
{
	if (coreServices.getWeekDayState() == 1)			// 1 means weekday
	{
		setGates(cid, CLOSE, "WD CLOSE", relay_pin);
	}	
}

void app_WE_on(void *cid)
{
	if (coreServices.getWeekDayState() == 0)			// 0 means weekend
	{
		setGates(cid, OPEN, "WE OPEN", relay_pin);
	}	
}

void app_WE_off(void *cid)
{
	if (coreServices.getWeekDayState() == 0)			// 0 means weekend
	{
		setGates(cid, CLOSE, "WE CLOSE", relay_pin);
	}	
}

void app_WD_auto(void *cid)
{
	if (coreServices.getWeekDayState() == 1)			// 1 means weekday
	{
		cntrlState *obj = (cntrlState *)cid;
		String msg = obj->getCntrlName() + " WD AUTO";
		mqttLog(msg.c_str(), REPORT_INFO, true, true);
							
		//mqttClient.publish(getWDCntrlRunTimesStateTopic().c_str(), 0, true, "AUTO");
		mqttClient.publish(obj->getWDUIcommandStateTopic().c_str(), 1, true, "SET"); //
	}	
}

void app_WE_auto(void *cid)
{
	if (coreServices.getWeekDayState() == 0)			// 0 means weekend
	{
		cntrlState *obj = (cntrlState *)cid;
		String msg = obj->getCntrlName() + " WE AUTO";
		mqttLog(msg.c_str(), REPORT_INFO, true, true);
		//mqttClient.publish(getWECntrlRunTimesStateTopic().c_str(), 0, true, "AUTO");
		mqttClient.publish(obj->getWEUIcommandStateTopic().c_str(), 1, true, "SET");
	}
}

void startTimesReceivedChecker()
{
	GateCntrlState.runTimeReceivedCheck();
}

void processCntrlTOD_Ext()
{
	GateCntrlState.processCntrlTOD_Ext();
}

void telnet_extension_1(char c)
{
	GateCntrlState.telnet_extension_1(c);
}

// Process any application specific telnet commannds
void telnet_extension_2(char c)
{
	char logString[501];
	memset(logString, 0, sizeof logString);
	/*
  sprintf(logString,
				"\r%s%i\n\r%s%i\n\r%s%i\n\r%s%i\n\r%s%i\n\r%s%i\n",
				"Upstairs Heating\t", digitalRead(HTG_UPSTAIRS_STATUS), 
				"Upstairs Demand\t\t", upHeatDemand,
				"Downstairs Heating\t", digitalRead(HTG_DOWNSTAIRS_STATUS),
				"Downstairs Demand\t", downHeatDemand,
				"Hot Water\t\t", digitalRead(HW_STATUS),
				"Hot Water Demand\t", waterHeatDemand);

	printTelnet((String)logString);
  */
}

// Process any application specific telnet commannds
void telnet_extensionHelp(char c)
{
	printTelnet((String) "x\t\tSome description");
}

bool onMqttMessageAppCntrlExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
{
	return false;
}

void setGates(void *cid, int state, String stateMsg, int gateDemand)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + "," +  stateMsg;
	mqttLog(msg.c_str(), REPORT_INFO, true, true);

	// FIXTHIS : Remove hard codeing of controller name

	if (obj->getCntrlName() == "GATESFRONT")
	{
		digitalWrite(gateDemand, state); 					// Open valve
		if (state == OPEN)
		{
			gateDemand = 1;							// Record value as digitalRead of OUT pin is not working (FIXTHIS)
			GateCntrlState.setOutputState(1);			// Feedback the Controller tho output of the requested action
		}
		else
		{
			gateDemand = 0;
			GateCntrlState.setOutputState(0);			// Feedback the Controller tho output of the requested action
		}										
	}
}
