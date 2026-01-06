/*
This is code to control a simple directional sensor and motor controller.

The goal is sensing Left to Right vs. Right to Left transitions of two IR LIDAR sensors.
When we are triggered, we send a specific MQTT payload to a MQTT server.

In ADDITION, we are listening to two MQTT topics, one a call for current sensor status
and one to control the state of a LGB two aspect mechanical semaphore - (OR a switch with updated code!)

This mechanical device is a LGB two aspect semaphore.  Important to note the semaphore rotary solenoid
is edintical to the switch motor rotary solenoid, so this code should be EASILY adaptable to switch use.

All of this is pretty easy work for a SEEED Studio XIAO ESP32-C3, which is our base micro controller.

This was MOSTLY written by asking ChatGPT very specific questions, then going ROUND and ROUND
over the details.  For something so "smart", it sure can be stupid sometimes...

If you hold the button for 15 seconds, it will cause the configuration portal to run, allowing various
parameters to be input using your cell phone or iPad.  Once entered, they are saved to NVRAM and are
persistent through future power cycles.

It is designed to work in concert with JMRI and the sensor and signal operation are TOTALLY separate 
functions.  You need a JMRI instance running and sending commands to the Mast to operate if you want
to see it actually change.

Version: 5.7
This version is a likely "Release Candidate".  The sensor is working, MQTT is publishing,
the whole custom parameters (and their persistence between reboots!) has been 
polished and appears to work - the sensor has yet to falsly tirgger and the motor
controller is snapping the little semaphore head around quite well..

Eric Timberlake
January, 2026

*/

/**********************************************************************
 * Libraries
 **********************************************************************/
#include <WiFiManager.h>            // WiFiManager for ESP32
#include <Preferences.h>            // Store WiFiManager params
#include <PubSubClient.h>           // MQTT client
#include <OneButton.h>              // Button handling

/**********************************************************************
 * Hardware pin definitions (XIAO ESP32-C3)
 **********************************************************************/
#define SENSOR_LEFT  D7             // Left TOF sensor, LOW = active
#define SENSOR_RIGHT D8             // Right TOF sensor, LOW = active
#define RED_LED      D3             // Red LED, LOW = on
#define GREEN_LED    D2             // Green LED, LOW = on
#define BUTTON_PIN   D6             // Button input, LOW = pressed
#define MOTOR_IN1    D0             // Motor driver IN1, HIGH = active
#define MOTOR_IN2    D1             // Motor driver IN2, HIGH = active

/**********************************************************************
 * Global objects
 **********************************************************************/
WiFiClient espClient;               // WiFi client for MQTT
PubSubClient mqtt(espClient);       // MQTT client using WiFi
Preferences preferences;            // Preferences (NVS storage)
WiFiManager wifiManager;            // WiFiManager instance

// --- Preferences namespace ---
const char* PREF_NAMESPACE = "sensorcfg";   // NVS namespace for saved params

/**********************************************************************
 * A little LED state support
 **********************************************************************/
enum LedState {
  LED_BOOTING,        // yellow
  LED_WIFI_LOST,      // yellow
  LED_MQTT_LOST,      // solid red
  LED_IDLE,           // green (connected & waiting)
  LED_MOTION,          // red (motion detected)
  LED_OFF             // both LED's in the of state..
};

LedState currentLedState = LED_BOOTING;



/**********************************************************************
 * Motor state and timing
 **********************************************************************/
#define OFF  0                      // Motor off state
#define CW   1                      // Clockwise motor direction
#define CCW  2                      // Counter-clockwise motor direction

int motorState = OFF;               // Current motor state
unsigned long motorStartTime = 0;   // When motor was started
unsigned long motorDuration = 250;  // Motor run duration (ms)

/**********************************************************************
 * Sensor state variables
 **********************************************************************/
bool virtualL2RActive = false;      // Virtual sensor Left→Right active
bool virtualR2LActive = false;      // Virtual sensor Right→Left active
bool passInProgress = false;        // Train pass currently in progress
bool firstSensorLeft = false;       // Which physical sensor triggered first
unsigned long lastLeftActive = 0;   // Last time left sensor was active
unsigned long lastRightActive = 0;  // Last time right sensor was active
unsigned long persistence_ms = 2000;// Persistence timeout (ms)

/**********************************************************************
 * Button handling
 **********************************************************************/
OneButton button(BUTTON_PIN, true); // OneButton instance, active LOW

/**********************************************************************
 * WiFiManager custom parameters
 **********************************************************************/
WiFiManagerParameter custom_html("<p style=\"color:red;font-weight:bold;\">Configure your Mast/Sensor here:</p>");  // HTML header in config portal
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP Address", "10.10.10.2", 40);                      // MQTT broker address
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", "1883", 8);                                              // MQTT port
WiFiManagerParameter custom_mqtt_username("username", "MQTT Username", "track", 32);                                // MQTT username
WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", "track1", 32);                               // MQTT password
WiFiManagerParameter custom_sensor_id("sensor_id", "Sensor Base ID", "0001", 6);                                    // MQTT Sensor Based ID (will be two!)
WiFiManagerParameter custom_persistence("persistence_ms", "Sensor Persistence in ms", "2000", 6);                   // how long the sensor persists after being activated
WiFiManagerParameter custom_mast_id("mast_id", "Mast ID", "0001", 8);                                               // Signal Mast ID, one only - could be switch, too.
WiFiManagerParameter custom_motor_duration("motor_ms", "Motor Duration in ms", "250", 6);                           // how long to drive the rotary solenoid

String sensor_id, sensor_id_plus_1, mastID;

/**********************************************************************
 * Generate a unique MQTT client ID
 **********************************************************************/
String makeMqttClientId() {
    uint64_t mac = ESP.getEfuseMac();      // Unique ESP32 MAC
    uint32_t macLow = (uint32_t)(mac & 0xFFFFFF);  // last 3 bytes
    char buf[32];
    sprintf(buf, "sensor-%s-%06X", sensor_id.c_str(), macLow);
    return String(buf);
}


/**********************************************************************
 * Helper: increment ID with preserved leading zeros
 **********************************************************************/
String addOneWithLeadingZeros(String s){
    int val = s.toInt() + 1;
    char buf[10];
    sprintf(buf, "%0*d", s.length(), val);
    return String(buf);
}

/**********************************************************************
 * Publish sensor state over MQTT
 **********************************************************************/
void publishSensor(String id, bool active){
    if(!mqtt.connected()) return;
    String topic = "track/sensor/status/" + id;
    String payload = active ? "ACTIVE" : "INACTIVE";
    mqtt.publish(topic.c_str(), payload.c_str());
    Serial.print("MQTT PUBLISH: "); Serial.print(topic); Serial.print(" → "); Serial.println(payload);
}


/**********************************************************************
 * LED phys pin support at boot/setup time..
 **********************************************************************/
void initStatusLed() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
}

/**********************************************************************
 * LED support, init and status
 **********************************************************************/
void setStatusLed(LedState state) {
  currentLedState = state;

  switch (state) {
    case LED_BOOTING:           // yellow
    case LED_WIFI_LOST:
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      break;

    case LED_MQTT_LOST:         // solid red
    case LED_MOTION:            // solid red for duration of trigger
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      break;

    case LED_IDLE:              // green
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      break;
    
    case LED_OFF:               // both off
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        break;
  }
}

/**********************************************************************
 * Motor control (non-blocking)
 * Driver is DRV8231A dual H Bridge, high on IN$(1-2) makes motor turn
 **********************************************************************/
void runMotor(int dir){
    motorState = dir;
    motorStartTime = millis();
    if(dir == CW){
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        Serial.println("Motor CW");
    }
    else if(dir == CCW){
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        Serial.println("Motor CCW");
    }
    else{
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        Serial.println("Motor OFF");
    }
}
//

/**********************************************************************
 * Check on WiFi and MQTT connditions occasionaly and set the LED if needed..
 **********************************************************************/

void connectionHealth(){
 // --- Connection health check ---
  if (WiFi.status() != WL_CONNECTED) {
    if (currentLedState != LED_WIFI_LOST) {
      setStatusLed(LED_WIFI_LOST);
    }
    return;  // nothing else makes sense without WiFi
  }

  if (!mqtt.connected()) {
    if (currentLedState != LED_MQTT_LOST) {
      setStatusLed(LED_MQTT_LOST);
    }
    //connectMQTT();
    ensureMQTTConnected();
    return;
  }
}

/**********************************************************************
 * MQTT callback
 **********************************************************************/
void mqttCallback(char* topic, byte* payload, unsigned int length){
    String t = String(topic);
    String p = "";
    for(unsigned int i=0; i<length; i++) p += (char)payload[i];
    Serial.print("MQTT RX: "); Serial.print(t); Serial.print(" → "); Serial.println(p);

    // Sensor commands
    
    if(t == "track/sensor/command/" + sensor_id){
        if(p.equalsIgnoreCase("ACTIVE")){
            Serial.println("Responding with L2R sensor status");
            publishSensor(sensor_id, virtualL2RActive);
        }
    }
    else if(t == "track/sensor/command/" + sensor_id_plus_1){
        if(p.equalsIgnoreCase("ACTIVE")){
            Serial.println("Responding with R2L sensor status");
            publishSensor(sensor_id_plus_1, virtualR2LActive);
        }
    }

    // Mast commands
    else if(t == "track/signalmast/" + mastID){
        if(p == "Clear; Lit; Unheld") {
            Serial.println("Setting track clear semaphore");
            runMotor(CW);
        } 
        else if(p == "Stop; Lit; Unheld") {
            Serial.println("Setting STOP semaphore");
            runMotor(CCW);
        }
    }
}

/**********************************************************************
 * Sensor FSM update
 **********************************************************************/
void updateSensors(){
    bool left = digitalRead(SENSOR_LEFT) == LOW;                // Left sensor active?
    bool right = digitalRead(SENSOR_RIGHT) == LOW;              // Right sensor active? 
    unsigned long now = millis();                               // Current time

    if(left) lastLeftActive = now;
    if(right) lastRightActive = now;

    if(left || right){                                          // Any sensor active
        if(!passInProgress){
            passInProgress = true;
            firstSensorLeft = left;
        }
    }

    if(passInProgress && left && right){                        // Both active    
        if(firstSensorLeft && !virtualL2RActive){
            virtualL2RActive = true;
            publishSensor(sensor_id,true);
            Serial.println("DIR L→R ACTIVE");
        }
        else if(!firstSensorLeft && !virtualR2LActive){
            virtualR2LActive = true;
            publishSensor(sensor_id_plus_1,true);
            Serial.println("DIR R→L ACTIVE");
        }
    setStatusLed(LED_MOTION);   // red while active

    }

    if(passInProgress && !left && !right && now - max(lastLeftActive,lastRightActive) > persistence_ms){
        if(virtualL2RActive){
            virtualL2RActive = false;
            publishSensor(sensor_id,false);
            Serial.println("DIR L→R INACTIVE");
        }
        if(virtualR2LActive){
            virtualR2LActive = false;
            publishSensor(sensor_id_plus_1,false);
            Serial.println("DIR R→L INACTIVE");
        }
    setStatusLed(LED_IDLE);
    passInProgress = false;
    }
}

/**********************************************************************
 * Button callbacks
 **********************************************************************/
void shortPress(){
    static bool toggleCW = true;
    String payload = toggleCW ? "Clear; Lit; Unheld" : "Stop; Lit; Unheld";
    toggleCW = !toggleCW;
    if(mqtt.connected()){
        mqtt.publish(("track/signalmast/" + mastID).c_str(), payload.c_str());
        Serial.print("Short button press → MQTT publish: "); Serial.println(payload);
    }
}

void longPress(){
    setStatusLed(LED_BOOTING);      // set boTH led's on, "Yelow", while in the config portal
    Serial.println("Long button press → entering config portal");
    wifiManager.startConfigPortal("Train Sensor Config");
    savePreferences();   // Persist custom parameters after config
}

/**********************************************************************
 * MQTT reconect logic
 **********************************************************************/
unsigned long lastMQTTRetry = 0;
const unsigned long mqttRetryInterval = 5000; // retry every 5 seconds
bool initialMotorTriggered = false;

void ensureMQTTConnected() {
    if(mqtt.connected()) return;

    static String mqttClientId = makeMqttClientId();

    unsigned long now = millis();
    if(now - lastMQTTRetry < mqttRetryInterval) return;
    lastMQTTRetry = now;

    Serial.println("MQTT connect attempt...");
    if(mqtt.connect(mqttClientId.c_str(), custom_mqtt_username.getValue(), custom_mqtt_password.getValue())) {
        Serial.println("MQTT connected");
        setStatusLed(LED_IDLE);             // green, ready for motion
        mqtt.subscribe(("track/sensor/command/" + sensor_id).c_str());
        mqtt.subscribe(("track/sensor/command/" + sensor_id_plus_1).c_str());
        mqtt.subscribe(("track/signalmast/" + mastID).c_str());

        // Initial CW trigger after first successful MQTT connection
        if(!initialMotorTriggered){
            runMotor(CW);
            initialMotorTriggered = true;
        }

    } else {
        setStatusLed(LED_MQTT_LOST);        // solid red!  Alert user to fault..
        Serial.print("MQTT connect failed, rc=");
        Serial.println(mqtt.state());
    }
}

/**********************************************************************
 * Save WiFiManager custom parameters to Preferences
 **********************************************************************/
void savePreferences() {
    preferences.begin(PREF_NAMESPACE, false);     // RW mode
    preferences.putString("mqtt_server", custom_mqtt_server.getValue());
    preferences.putString("mqtt_port", custom_mqtt_port.getValue());
    preferences.putString("mqtt_user", custom_mqtt_username.getValue());
    preferences.putString("mqtt_pass", custom_mqtt_password.getValue());
    preferences.putString("sensor_id", custom_sensor_id.getValue());
    preferences.putString("mast_id", custom_mast_id.getValue());
    preferences.putString("persistence", custom_persistence.getValue());
    preferences.putString("motor_ms", custom_motor_duration.getValue());
    preferences.end();
    Serial.println("Preferences saved");
}

/**********************************************************************
 * Load WiFiManager custom parameters from Preferences
 **********************************************************************/
void loadPreferences() {
    preferences.begin(PREF_NAMESPACE, true);      // RO mode

    if (preferences.isKey("mqtt_server")) {
        custom_mqtt_server.setValue(preferences.getString("mqtt_server").c_str(),40);
        custom_mqtt_port.setValue(preferences.getString("mqtt_port").c_str(),8);
        custom_mqtt_username.setValue(preferences.getString("mqtt_user").c_str(),32);
        custom_mqtt_password.setValue(preferences.getString("mqtt_pass").c_str(),32);
        custom_sensor_id.setValue(preferences.getString("sensor_id").c_str(),6);
        custom_mast_id.setValue(preferences.getString("mast_id").c_str(),8);
        custom_persistence.setValue(preferences.getString("persistence").c_str(),6);
        custom_motor_duration.setValue(preferences.getString("motor_ms").c_str(),6);
        Serial.println("Preferences loaded");
    } else {
        Serial.println("No saved preferences found");
    }

    preferences.end();
}

/**********************************************************************
 * Setup
 **********************************************************************/
void setup() {
    Serial.begin(115200);
    Serial.println("Booting up now!");

    initStatusLed();
    setStatusLed(LED_BOOTING);
    pinMode(SENSOR_LEFT, INPUT_PULLUP);
    pinMode(SENSOR_RIGHT, INPUT_PULLUP);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, HIGH);      // turn motor off right a way!
    digitalWrite(MOTOR_IN2, HIGH);      // turn motro off too!

    loadPreferences();   // Restore saved custom parameters before portal

    // WiFiManager menu
    std::vector<const char *> menu = {"wifi","info","restart","exit"};
    wifiManager.setMenu(menu);


    wifiManager.addParameter(&custom_html);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_username);
    wifiManager.addParameter(&custom_mqtt_password);
    wifiManager.addParameter(&custom_sensor_id);
    wifiManager.addParameter(&custom_persistence);
    wifiManager.addParameter(&custom_mast_id);
    wifiManager.addParameter(&custom_motor_duration);

    wifiManager.autoConnect("Train Sensor Config");
    savePreferences();   // Persist custom parameters after config

    Serial.print("*wm:STA IP Address: "); Serial.println(WiFi.localIP());
    Serial.println("WiFi connected");

    // Load params
    sensor_id = custom_sensor_id.getValue();
    sensor_id_plus_1 = addOneWithLeadingZeros(sensor_id);
    mastID = custom_mast_id.getValue();
    persistence_ms = atol(custom_persistence.getValue());
    motorDuration = atol(custom_motor_duration.getValue());

    mqtt.setServer(custom_mqtt_server.getValue(), atoi(custom_mqtt_port.getValue()));
    mqtt.setCallback(mqttCallback);

    // Button setup
    button.attachClick(shortPress);                 // recognize a short press 
    button.attachLongPressStart(longPress);         // start checking for a long press all the time..
    button.setPressTicks(10000);                    // how long is a "long press"


    setStatusLed(LED_IDLE);     // green, ready for motion
}

/**********************************************************************
 * Main loop, do this forever!
 **********************************************************************/
void loop() {
    button.tick();                  // Check the button status!
    updateSensors();                // check the IR laser sensors!
    connectionHealth();             // Check that we're on WiFi and connected to MQ server!
    mqtt.loop();                    // take a moment and listen to MQTT topics in case we should be doing something!

    // Motor timeout
    if(motorState != OFF && millis() - motorStartTime >= motorDuration){
        runMotor(OFF);               // turn the motor off, we've passed the motor on variable limit..
    }
}
