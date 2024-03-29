/**
 * Board: NodeMCU 1.0
*/
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <SimpleTimer.h> // https://github.com/jfturcot/SimpleTimer
#include <ESP8266WebServer.h>
#include <Bounce2.h> // https://github.com/thomasfredericks/Bounce2
#include "soundFx.h"
#include "config.h"

#define BOUNCE_WITH_PROMPT_DETECTION

enum TrayStatus {
  STOPPED,
  MOVING_BACKWARDS,
  MOVING_FORWARD
};

enum DeviceStatus {
  INITIALIZING,
  IDLE,
  DISPENSING,
  ERROR
};

Servo myservo;  // create servo object to control a servo
SimpleTimer timer;
ESP8266WebServer server(80);

bool volatile finalPosReached, initialPosReached = false;
TrayStatus volatile trayStatus = STOPPED;
DeviceStatus volatile deviceStatus = INITIALIZING;
unsigned long lastOperationTime;
char logBuffer[200];

#ifndef PWMRANGE
#define PWMRANGE 1023
#endif

uint16_t ledIntensity = PWMRANGE;
int blinkTimerId; // made 0 when set

// input relay is mechanical so we need to debounce it
Bounce initialPosContact = Bounce();
Bounce finalPosContact = Bounce();

// -----------------------------------------------------------------


/**
 * Stop tray
 * 
 * bool isError - indicates if stopping occured because of error
 */
void stopTray(bool isError = false)
{
  if (!myservo.attached()) {
    return;
  }
  
  myservo.write(90);
  trayStatus = STOPPED;
  Serial.println("tray stopped");
  deviceStatus = isError ? ERROR : (deviceStatus != DISPENSING ? IDLE : deviceStatus);  
}

/**
 * Main arduino setup function, executed at the begining
 */
void setup()
{
  pinMode(LIGHT_PIN, OUTPUT);
  analogWrite(LIGHT_PIN, ledIntensity);  
  initialPosContact.attach(INITIAL_POS_PIN, INPUT_PULLUP);
  finalPosContact.attach(FINAL_POS_PIN, INPUT_PULLUP);
  initialPosContact.interval(1);
  finalPosContact.interval(1);  
  pinMode(BUZZER_PIN, OUTPUT);  

  initialPosReached = !initialPosContact.read();
  finalPosReached = !finalPosContact.read();

  Serial.begin(115200);
  wifiSetup();  
  webserverSetup();
  deviceStatus = IDLE;
}

/**
 * Main loop() arduino function, executed continuously
 * 
 */
void loop()
{
  timer.run();
  if (deviceStatus == IDLE) {
    server.handleClient();    // Handling of incoming requests  
  }

  initialPosContact.update(); 
  finalPosContact.update(); 
  initialPosReached = !initialPosContact.read();
  finalPosReached = !finalPosContact.read();
    
  if (initialPosReached && trayStatus == MOVING_BACKWARDS) {    
    stopTray();    
    Serial.println("Stopped becayse initial pos reached");
  }
  
  if (finalPosReached && trayStatus == MOVING_FORWARD) {      
    stopTray();
    Serial.println("Stopped because final pos reached");
  }
    

  if (trayStatus != STOPPED && millis() - lastOperationTime > JAM_PROTECTION_TIME) {    
    stopTray(true); // stop tray due to errors passing true as param
    Serial.println("Jam protection activated !");
    systemError();
  }
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    char command = Serial.read();

    if (command == 'b') {
      moveTrayBackwards();
    } else if (command == 'f') {
      moveTrayForward();      
    } else if (command == 'd') {
      dispense();
    }
  }
}

// -----------------------------------------------------------------

/**
 * Moves tray forward until it reached final pos
 */
void moveTrayForward()
{
  Serial.println("Received command to move tray forward.");
  //Serial.printf("FinalPosReached: %d, DeviceStatus: %d\n", finalPosReached, deviceStatus);
  if (finalPosReached || deviceStatus == ERROR) {
    Serial.println("Skipping moveTrayForward command because already at final pos or error.");
    return;
  }

  if (!myservo.attached()) {
    myservo.attach(SERVO_PIN);
  }
  
  trayStatus = MOVING_FORWARD;
  lastOperationTime = millis();
  Serial.println("Moving tray forward...");
  myservo.write(0); // 0 .. 60
}

/**
 * Moves tray backwards until it reached initial pos
 */
void moveTrayBackwards()
{
  Serial.println("Received command to move tray backwards.");
  //Serial.printf("initialPosReached: %d, DeviceStatus: %d\n", initialPosReached, deviceStatus);
  if (initialPosReached || deviceStatus == ERROR) {
    Serial.println("Skipping moveTrayBackwards command because alraedty at initial pos or error.");
    return;
  };

  if (!myservo.attached()) {
    myservo.attach(SERVO_PIN);
  }
  trayStatus = MOVING_BACKWARDS;
  lastOperationTime = millis();
  Serial.println("Moving tray backwards...");
  myservo.write(180); // 91 .. 180
}

/**
 * Connect to WiFI
 */
void wifiSetup()
{  
  sprintf(logBuffer, "wifiSetup(): Connecting to %s", WIFI_SSID);
  Serial.println(logBuffer);
  WiFi.mode(WIFI_STA);
  
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(DEVICE_NAME); 
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int startMillis = millis();

  while (ledIntensity > PWMRANGE / 3) {
      analogWrite(LIGHT_PIN, ledIntensity--);
      delay(1);
  }
  bool shouldFadeOut = true;
  
  while (WiFi.status() != WL_CONNECTED) {
    analogWrite(LIGHT_PIN, shouldFadeOut ? ledIntensity-- : ledIntensity++);
    if (ledIntensity >= PWMRANGE / 3 ) {
      shouldFadeOut = true; 
    } else if (ledIntensity <= PWMRANGE / 10) {
      shouldFadeOut = false; 
    }
    
    delay(WIFI_WAIT_DELAY);
    
    if ((millis() - startMillis) > WIFI_CONNECTION_TIMEOUT) {
      Serial.println("wifiSetup(): Wifi connection timeout.");
      break;            
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("wifiSetup(): Could not connect to network");
    systemError();
  } else {
    Serial.println("wifiSetup(): Wifi connected. IP: " + WiFi.localIP().toString());
    WiFi.setAutoReconnect(true);
    tone(BUZZER_PIN, NOTE_A5, 300);   

    while (ledIntensity < 1023) {
      analogWrite(LIGHT_PIN, ledIntensity++);
      delay(1);
    }
    digitalWrite(LIGHT_PIN, HIGH);
  }
}

/**
 * Triggered when a system error occur (tray jam, WiFi issues, etc)
 */
void systemError()
{
  if (blinkTimerId != NULL) {
    return;
  }

  blinkTimerId = timer.setInterval(BLINK_INTERVAL, blinkLed);
  deviceStatus = ERROR;
}

/**
 * Prepare webserver and API routes
 */
void webserverSetup()
{
  server.client().setTimeout(300);
  server.on("/api/dispenser/dispense", HTTP_POST, handleDispense);
  server.on("/api/dispenser/status", HTTP_GET, handleGetStatus);
  server.on("/api/dispenser/maintenance/tray/move-forward", HTTP_POST, handleMoveTrayForward);
  server.on("/api/dispenser/maintenance/tray/move-backwards", HTTP_POST, handleMoveTrayBackwards);
  server.on("/api/dispenser/maintenance/tray/status", HTTP_GET, handleGetTrayStatus);
  server.begin();   // Start the webserver;
}

void sendCORSHeaders()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, GET, PATCH");
  server.sendHeader("Access-Control-Max-Age", "1000");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

/**
 * Give me a chocolate-bar !
 * Moves tray backwards, waits a bit, then moves it forward to push the chocolate-bar
 * Must be non-blocking
 */
bool dispense()
{
  if (deviceStatus != IDLE) {
    return false;
  }
  
  Serial.println("Dispensing....");
  
  deviceStatus = DISPENSING;
  
  tone(BUZZER_PIN, NOTE_C4, 500);    
  delay(150);
  tone(BUZZER_PIN, NOTE_E4, 500);
  delay(150);
  tone(BUZZER_PIN, NOTE_G4, 500);
  delay(150);
  tone(BUZZER_PIN, NOTE_C5, 500); 
  
  moveTrayBackwards();
  // non-blocking delay
  if (deviceStatus != ERROR) {
    timer.setTimeout(1800, moveTrayForward);
    timer.setTimeout(3200, onDispenseEnded);
    return true;
  }

  return false;
}

void onDispenseEnded()
{
  if (deviceStatus == ERROR) {
    return;
  }

  deviceStatus = IDLE;
  
  playWinMelody(BUZZER_PIN);
}

/**
 * API dispense handler
 */
void handleDispense()
{
  Serial.println("Received dispense request");
  sendCORSHeaders();

  if (!dispense()) {
    server.send(500, "application/json", "{\"error\": true}");
    return;
  }
  
  server.send(200, "application/json", "{\"success\": true}");
}

/**
 * API dispense status handler
 */
void handleGetStatus()
{
  char result[200];
  char deviceStatusText[50];
  //Serial.println("Received device status request");

  sendCORSHeaders();
  
  switch (deviceStatus) {
    case IDLE:
      strcpy(deviceStatusText, "Idle");
      break;
    case DISPENSING:
      strcpy(deviceStatusText, "Dispensing");
      break;
    case ERROR:
      strcpy(deviceStatusText, "Tray jammed");
      break;
  }

  snprintf(result, sizeof(result), "{\"statusCode\": %d, \"statusText\": \"%s\"}", deviceStatus, deviceStatusText);
  server.send(200, "application/json", result);
}


/**
 * API tray maintenance move tray forward handler
 */
void handleMoveTrayForward()
{
  Serial.println("Received mvoe tray forward request");

  sendCORSHeaders();
  
  if (deviceStatus == ERROR) {
    server.send(500, "application/json", "{\"error\": true}");
    return;
  }
  
  moveTrayForward();
  server.send(200, "application/json", "{\"success\": true}");
}

/**
 * API tray maintenance move tray backwards handler
 */
void handleMoveTrayBackwards()
{
  Serial.println("Received move tray backwards request");

  sendCORSHeaders();
  
  if (deviceStatus == ERROR) {
    server.send(500, "application/json", "{\"error\": true}");
    return;
  }
  
  moveTrayBackwards();
  server.send(200, "application/json", "{\"success\": true}");
}

void handleGetTrayStatus()
{
  char result[200];
  char statusText[50];

  Serial.println("Received get tray status request");

  sendCORSHeaders();
  
  switch (trayStatus) {
    case MOVING_FORWARD:
      strcpy(statusText, "Moving forward");
      break;
    case MOVING_BACKWARDS:
      strcpy(statusText, "Moving backwards");
      break;
    case STOPPED:
      strcpy(statusText, "Stopped");
      break;
  }

  snprintf(
    result,
    sizeof(result),
    "{\"status\": %d, \"statusText\": %s, \"operationTime\": %d}",
    trayStatus,
    statusText,
    trayStatus != STOPPED ? (millis() - lastOperationTime) : 0
  );
  server.send(200, "application/json", result);
}

/**
 * Toggle led
 * Executed by a timer at regular intervals to blink led on and off
 */
void blinkLed()
{
  digitalWrite(LIGHT_PIN, !digitalRead(LIGHT_PIN));
  if (digitalRead(LIGHT_PIN)) {
    tone(BUZZER_PIN, NOTE_C2);
  } else {
    noTone(BUZZER_PIN);
  }  
}
