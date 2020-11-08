#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const char* ssid = "<Enter your wifi network name here>";
const char* password = "<Enter your wifi password here>";
bool wifi_connected = false;
#define WIFI_MAX_CONNECT_ATTEMPTS 3

BluetoothSerial SerialBT;

const int flash_delay = 500;
const int comms_timeout_delay = 500;
int last_time = 0;
int comms_timeout = 0;

bool HelloSent = 0;

#define LED1_PIN 14
#define LED2_PIN 12

#define PWM1_PIN 17
#define PWM2_PIN 16
#define PWM3_PIN 4
#define PWM4_PIN 2

void setup() {
  int wifi_attempts = 0;
  Serial.begin(115200);
  Serial.println("Booting");

  ///////////////////////////////////////////// OTA Programming Setup /////////////////////////////////////////////
  
  WiFi.mode(WIFI_STA);

  while((wifi_attempts < WIFI_MAX_CONNECT_ATTEMPTS) && (wifi_connected != true))
  {
    WiFi.begin(ssid, password);

    if(WiFi.waitForConnectResult() == WL_CONNECTED)
    {
      wifi_connected = true;
    }
    else
    {
      delay(5000);
      Serial.println("WiFi Connection Failed!");
      wifi_connected = false;
    }

    wifi_attempts ++;
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("Lobot");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  /////////////////////////////////////// Bluetooth Setup /////////////////////////////////////////
  SerialBT.begin("Lobot"); //Bluetooth device name
  Serial.println("Bluetooth serial started");
  

  //////////////////////////////////////// Lobot Setup ////////////////////////////////////////////

  pinMode(14,OUTPUT);
  pinMode(12,OUTPUT);

  ledcSetup(0, 2000, 8);
  ledcAttachPin(PWM1_PIN,0);

  ledcSetup(1, 2000, 8);
  ledcAttachPin(PWM2_PIN,1);

  ledcSetup(2, 2000, 8);
  ledcAttachPin(PWM3_PIN,2);  

  ledcSetup(3, 2000, 8);
  ledcAttachPin(PWM4_PIN,3); 

  //////////////////////////////// Timer Setup ////////////////////////////////////
  last_time = millis();
  comms_timeout = millis() + comms_timeout_delay;
}

void loop() 
{
  char BTCommand = 0;
  
  ArduinoOTA.handle();


 if ((millis() > (last_time + flash_delay)) && (millis() < (last_time + flash_delay *2)))
 {
      digitalWrite(LED1_PIN,HIGH);
      digitalWrite(LED2_PIN,LOW); 
 }
 else if (millis() > (last_time + flash_delay * 2))
 {
      digitalWrite(LED1_PIN,LOW);
      digitalWrite(LED2_PIN,HIGH);  

      last_time = millis();
 }
 
  
  if(SerialBT.available() > 0) //If Bluetooth is available then start de-queueing commands
  {
    BTCommand = SerialBT.read();

    if(HelloSent == 0)  //Let the user know Lobot is connected
    {
      SerialBT.println("Hi from Lobot!"); 
      HelloSent = 1;
    }

    switch(BTCommand)
    {
      case 'f':
        DriveForward(250);
        comms_timeout = millis() + comms_timeout_delay;
        break;
      case 'b':
        DriveBackward(250);
        comms_timeout = millis() + comms_timeout_delay;
        break;
      case 'l':
        DriveLeft(200);
        comms_timeout = millis() + comms_timeout_delay;
        break;
      case 'r':
        DriveRight(200);
        comms_timeout = millis() + comms_timeout_delay;
        break;
      case 's':
        DriveStop();
        comms_timeout = millis() + comms_timeout_delay;
        break;  
      default:
        //DriveStop();
        break;
    }
  }

  //If the comms timeout has been exceeded the stop the drive motors
  if(millis() > comms_timeout)
  {
    DriveStop();
  }
}

void DriveStop(void)
{
  Serial.println("Stop");

  digitalWrite(LED1_PIN,LOW);
  digitalWrite(LED2_PIN,LOW);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0); 
  ledcWrite(2, 0);
  ledcWrite(3, 0);  
}

void DriveBackward(uint16_t duty)
{
  Serial.print("Forward ");
  Serial.println(duty);

  digitalWrite(LED1_PIN,HIGH);
  digitalWrite(LED2_PIN,HIGH);
  
  ledcWrite(0, 0);
  ledcWrite(1, duty); 
  ledcWrite(2, duty);
  ledcWrite(3, 0);  
}

void DriveForward(uint16_t duty)
{
  Serial.print("Reverse ");
  Serial.println(duty);

  digitalWrite(LED1_PIN,LOW);
  digitalWrite(LED2_PIN,LOW);
  
  ledcWrite(0, duty);
  ledcWrite(1, 0); 
  ledcWrite(2, 0);
  ledcWrite(3, duty);  
}

void DriveRight(uint16_t duty)
{
  Serial.print("Left ");
  Serial.println(duty);

  digitalWrite(LED1_PIN,HIGH);
  digitalWrite(LED2_PIN,LOW);
  
  ledcWrite(0, 0);
  ledcWrite(1, duty); 
  ledcWrite(2, 0);
  ledcWrite(3, duty);  
}

void DriveLeft(uint16_t duty)
{
  Serial.print("Right ");
  Serial.println(duty);

  digitalWrite(LED1_PIN,LOW);
  digitalWrite(LED2_PIN,HIGH);  
  
  ledcWrite(0, duty);
  ledcWrite(1, 0); 
  ledcWrite(2, duty);
  ledcWrite(3, 0);  
}
