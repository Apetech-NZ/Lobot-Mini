#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Update.h>

#include "Lobot_Internal_EEPROM.h"

#define SOFTWARE_VERSION  "0.1"

//#define DEBUG_DRIVE_MOTORS

//For Tasks
TaskHandle_t LEDTask;

//For Internal EEPROM
LobotEEPROM lobot_eeprom;

//For Sleep mode
#define uS_TO_S_FACTOR 1000000
#define uS_TO_mS_FACTOR 1000

//For Bluetooth
#define BLUETOOTH_NAME "Lobot"
#define BLUETOOTH_PIN "lobot"
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

//For WiFi
#define WIFI_MAX_CONNECT_ATTEMPTS 2
#define HOSTNAME  "Lobot"
WebServer server(80);
bool wifi_connected = false;

//For command line interface
#define WIFI_SSID_COMMAND "ssid"

#define WIFI_KEY_COMMAND "key"

#define DATA_RATE_COMMAND "dataperiod"

#define DEVICE_ADDRESS_MAX 1000
#define DEVICE_ADDRESS_COMMAND "address"

#define WIFI_ENABLED_COMMAND "wifi"
#define WIFI_ENABLED  1
#define WIFI_DISABLED 0

#define RESET_COMMAND "reset"
#define ACK_COMMAND "ahoyhoy"
#define ACK_REPLY "ahoy"
#define IP_COMMAND "ip"
#define VERSION_COMMAND "version"

#define SLEEP_ENABLED_COMMAND "sleep"

#define DRIVE_STOP_COMMAND "s"
#define DRIVE_FORWARD_COMMAND "f"
#define DRIVE_BACKWARD_COMMAND "b"
#define DRIVE_LEFT_COMMAND "l"
#define DRIVE_RIGHT_COMMAND "r"
#define MOTOR_LEFT_COMMAND "ml"
#define MOTOR_RIGHT_COMMAND "mr"

#define ESC_DUTY_COMMAND "esc"

//For Lobot IO
#define LED0_PIN 14 
#define LED1_PIN 12

#define LOBOT_MINI_LED_PIN 32

#define BOOT_PIN 0    //Use the boot pin to disable sleep

#define SUPPLY_MEASURE_PIN  A6

#define PWM1_PIN 17
#define PWM2_PIN 16
#define PWM3_PIN 4
#define PWM4_PIN 2
#define ESC0_PIN 25

#define ESC0_COUNT_MIN      1200
#define ESC0_COUNT_MAX      8600
#define ESC0_INPUT_DUTY_MAX 255

#define WAKE_PIN_BITMASK 0x000000001  //Set the bit positon of the GPIO number

//For Lobot system
//#define HEARTBEAT_RESET_COUNT 100
//#define HEARTBEAT_TIMER_DELAY 120000

uint8_t systemStatus = 0;
//int heartbeat = HEARTBEAT_RESET_COUNT;
uint8_t sleep_en = 0;
int32_t data_delay = 60000;
//uint32_t lastTransmitTime = 0;
//uint32_t lastHeartbeatTime = 0;
uint8_t wifi_enable = 1;
int16_t lobot_address = 0;

//const int flash_delay = 500;
const int comms_timeout_delay = 500;
//int last_time = 0;
int comms_timeout = 0;


void setup() 
{
  uint32_t wifi_attempts = 0;
  String ssid = "empty";
  String key = "empty";

  //Setup GPIO
  pinMode(LED0_PIN,OUTPUT);
  digitalWrite(LED0_PIN,HIGH);
  pinMode(LED1_PIN,OUTPUT);
  digitalWrite(LED1_PIN,HIGH);
  pinMode(LOBOT_MINI_LED_PIN,OUTPUT);
  digitalWrite(LOBOT_MINI_LED_PIN,HIGH);

  pinMode(BOOT_PIN,INPUT_PULLUP);

  ledcSetup(0, 500, 8);
  ledcAttachPin(PWM1_PIN,0);

  ledcSetup(1, 500, 8);
  ledcAttachPin(PWM2_PIN,1);

  ledcSetup(2, 500, 8);
  ledcAttachPin(PWM3_PIN,2);  

  ledcSetup(3, 500, 8);
  ledcAttachPin(PWM4_PIN,3); 

  ledcSetup(4, 50, 16);
  ledcAttachPin(ESC0_PIN,4);
  ledcWrite(4,ESC0_COUNT_MIN); //Immediately set the count for the ESC PWM to try and stop the ESC getting upset

  //Turn the "power" LED on
  digitalWrite(LED0_PIN,true);
 
  Serial.begin(115200,SERIAL_8N1,3,1);  //Force the serial pins to UART0 pins
  Serial.println("Booting");
  

  ////////////////////////////// Bluetooth Setup ///////////////////////////////////
  //Construct the bluetooth identifier
  String bluetooth_id = (String(BLUETOOTH_NAME) + '_' + String(lobot_eeprom.ReadInt(DEVICE_ADDRESS_EEPROM_ADDRESS)));
  String bluetooth_pin = (String(BLUETOOTH_PIN));
  SerialBT.setPin(bluetooth_pin.c_str());
  //SerialBT.enableSSP();
  SerialBT.begin(bluetooth_id.c_str()); //Bluetooth device name
  SerialBT.setTimeout(1000);

  ////////////////////////////// EEPROM Setup ///////////////////////////////////
  //Try to intialise the EEPROM. If it initialises OK read out the system settings
  //if (!EEPROM.begin(EEPROM_SIZE))
  if(!lobot_eeprom.begin())
  {
    Serial.println("failed to initialise EEPROM");
  }
  else
  {
    data_delay = lobot_eeprom.ReadUint32(DATA_RATE_ADDRESS);
    //Since it's an int it could be negative. Just use the absolute value
    lobot_address = abs(lobot_eeprom.ReadInt(DEVICE_ADDRESS_EEPROM_ADDRESS));
    
    //Load the wifi and MQTT credentials from EEPROM
    ssid = lobot_eeprom.ReadString(WIFI_SSID_EEPROM_ADDRESS,WIFI_SSID_EEPROM_LENGTH);
    key = lobot_eeprom.ReadString(WIFI_KEY_EEPROM_ADDRESS,WIFI_KEY_EEPROM_LENGTH);
    wifi_enable = lobot_eeprom.ReadByte(WIFI_ENABLED_EEPROM_ADDRESS);
    sleep_en = lobot_eeprom.ReadByte(SLEEP_ENABLED_EEPROM_ADDRESS);
  }

  ////////////////////////////// Deep Sleep Setup ///////////////////////////////
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every <data_delay> milliseconds
  */
  esp_sleep_enable_timer_wakeup(data_delay * uS_TO_mS_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(data_delay/1000) +
  " Seconds");

  //Ext1 can operate in ultra-low power, EXT0 requires the GPIO peripheral to be powered up
  //Only RTC IO can be used as a source for external wake source. They are pins: 0,2,4,12-15,25-27,32-39.
  esp_sleep_enable_ext1_wakeup(WAKE_PIN_BITMASK,ESP_EXT1_WAKEUP_ALL_LOW);


  ////////////////////////////// WIFI SETUP /////////////////////////////////////
  WiFi.mode(WIFI_STA);
  
  while((wifi_attempts < WIFI_MAX_CONNECT_ATTEMPTS) && (wifi_connected != true))
  {
    WiFi.begin(ssid.c_str(), key.c_str());

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
  ArduinoOTA.setPassword("lobot");

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
      Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
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

  //Create a host name for the web updater
  String host = (String(HOSTNAME) + '_' + String(lobot_eeprom.ReadInt(DEVICE_ADDRESS_EEPROM_ADDRESS)));
  //Start the web updater server
  WebUpdaterBegin(host);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Ready");

  //////////////////////////////// Timer Setup ////////////////////////////////////
  //last_time = millis();
  comms_timeout = millis() + comms_timeout_delay;

  Serial.print("setup() finished on core ");
  Serial.println(xPortGetCoreID());

  ///////////////////////////////Create Tasks//////////////////////////////////////
  //TODO: Add different flash states and a queue to set states from different tasks
  xTaskCreatePinnedToCore(
      LEDTaskCode, /* Function to implement the task */
      "LEDTask", /* Name of the task */
      1000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &LEDTask,  /* Task handle. */
      0); /* Core where the task should run */
}


//############################################################ LOOP #################################################################
void loop() 
{
  uint32_t i = 0;
  String cmd_in;
  String arg_in;

  //Check to see if any commands have come in on the serial interfaces
  if(Serial.available() > 0)
  {
    ReceiveSerialCommand();
  }

  if(SerialBT.available() > 0)
  {
    ReceiveSerialBTCommand();
  }

  //If the comms timeout has been exceeded the stop the drive motors and ESC
  if(millis() > comms_timeout)
  {
    DriveStop();
    //esc_channel0.write(0);
  }

  if(wifi_connected == true)
  {
    //Run OTA handlers
    ArduinoOTA.handle();
    server.handleClient();
  }
}

//////////////////////////////////////////////////////////////////////// Motor Control functions /////////////////////////////////////////////////////////////////

void DriveStop(void)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.println("Stop");
  #endif

  digitalWrite(LED0_PIN,LOW);
  digitalWrite(LED1_PIN,LOW);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0); 
  ledcWrite(2, 0);
  ledcWrite(3, 0);  
}

void DriveForward(uint16_t duty)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("Forward ");
  Serial.println(duty);
  #endif

  digitalWrite(LED0_PIN,HIGH);
  digitalWrite(LED1_PIN,HIGH);
  
  ledcWrite(0, 0);
  ledcWrite(1, duty); 
  ledcWrite(2, duty);
  ledcWrite(3, 0);  
}

void DriveBackward(uint16_t duty)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("Reverse ");
  Serial.println(duty);
  #endif

  digitalWrite(LED0_PIN,LOW);
  digitalWrite(LED1_PIN,LOW);
  
  ledcWrite(0, duty);
  ledcWrite(1, 0); 
  ledcWrite(2, 0);
  ledcWrite(3, duty);  
}

void DriveLeft(uint16_t duty)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("Left ");
  Serial.println(duty);
  #endif

  digitalWrite(LED0_PIN,HIGH);
  digitalWrite(LED1_PIN,LOW);
  
  ledcWrite(0, 0);
  ledcWrite(1, duty); 
  ledcWrite(2, 0);
  ledcWrite(3, duty);  
}

void DriveRight(uint16_t duty)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("Right ");
  Serial.println(duty);
  #endif

  digitalWrite(LED0_PIN,LOW);
  digitalWrite(LED1_PIN,HIGH);  
  
  ledcWrite(0, duty);
  ledcWrite(1, 0); 
  ledcWrite(2, duty);
  ledcWrite(3, 0);  
}

void MotorLeft(int16_t duty)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("Left Motor ");
  Serial.println(duty);
  #endif

  digitalWrite(LED1_PIN,HIGH);  

  if(duty > 0)
  {
    ledcWrite(0, 0);
    ledcWrite(1, abs(duty));
  }
  else if (duty < 0)
  {
    ledcWrite(0, abs(duty));
    ledcWrite(1, 0); 
  }
  else
  {
    ledcWrite(0, 0);
    ledcWrite(1, 0); 
    digitalWrite(LED1_PIN,LOW);
  }

}

void MotorRight(int16_t duty)
{
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("Right Motor ");
  Serial.println(duty);
  #endif

  digitalWrite(LED0_PIN,HIGH);
  
 
  if(duty > 0)
  {
    ledcWrite(2, abs(duty));
    ledcWrite(3, 0); 
  }
  else if (duty < 0)
  {
    ledcWrite(2, 0);
    ledcWrite(3, abs(duty));     
  }
  else
  {
    ledcWrite(2, 0);
    ledcWrite(3, 0); 
    digitalWrite(LED0_PIN,LOW);
  }
}

void ESC0_SetDuty(int16_t duty)
{
  //For the ESC we will only accept positive values (since that's how the ESCs work)
  int16_t esc0_duty = abs(duty);
  
  #ifdef DEBUG_DRIVE_MOTORS
  Serial.print("ESC 0 ");
  Serial.println(duty);
  #endif

  //Scale the duty to 255, similar to the drive motors
  esc0_duty = (duty * (ESC0_COUNT_MAX - ESC0_COUNT_MIN))/ESC0_INPUT_DUTY_MAX;
  esc0_duty += ESC0_COUNT_MIN;

  //Apply limits to the count value that will set the PWM. Going outside min/max duty results in the ESC packing a sad
  if(esc0_duty < ESC0_COUNT_MIN) esc0_duty = ESC0_COUNT_MIN;
  else if (esc0_duty > ESC0_COUNT_MAX) esc0_duty = ESC0_COUNT_MAX;

  //Now set the pwm duty
  ledcWrite(4, esc0_duty); 
}

//////////////////////////////////////////////////////////////////////// CLI functions /////////////////////////////////////////////////////////////////
void ReceiveSerialCommand(void)
{
  String cmd_in = Serial.readStringUntil('=');
  String arg_in = Serial.readString();
  
  ParseCommand(cmd_in,arg_in,&Serial);
}

void ReceiveSerialBTCommand(void)
{
  String cmd_in = SerialBT.readStringUntil('=');
  String arg_in = SerialBT.readStringUntil('\r');
  SerialBT.flush();
  
  ParseCommand(cmd_in,arg_in,&SerialBT);
}

void ParseCommand(String cmd_in, String arg_in, Stream *cli_stream)
{
  bool command_valid = true;
    //Serial.print("command received: ");
    //Serial.print(cmd_in);
    //Serial.print(", with argument: ");
    //Serial.println(arg_in);

    if (cmd_in.equalsIgnoreCase(String(WIFI_SSID_COMMAND)))
    {
      ProcessArg(arg_in,String(WIFI_SSID_COMMAND),WIFI_SSID_EEPROM_ADDRESS,WIFI_SSID_EEPROM_LENGTH,true,cli_stream);
    }
    else if (cmd_in.equalsIgnoreCase(String(WIFI_KEY_COMMAND)))
    {
       ProcessArg(arg_in,String(WIFI_KEY_COMMAND),WIFI_KEY_EEPROM_ADDRESS,WIFI_KEY_EEPROM_LENGTH,false,cli_stream);
    } 
    else if (cmd_in.equalsIgnoreCase(String(DATA_RATE_COMMAND)))
    {
       ProcessArgUint32(arg_in,String(DATA_RATE_COMMAND),DATA_RATE_ADDRESS,DATA_RATE_LENGTH,true,cli_stream);
    } 
    else if (cmd_in.equalsIgnoreCase(String(WIFI_ENABLED_COMMAND)))
    {
       ProcessArgByte(arg_in,String(WIFI_ENABLED_COMMAND),WIFI_ENABLED_EEPROM_ADDRESS,true,cli_stream);
    }
    else if (cmd_in.equalsIgnoreCase(String(DEVICE_ADDRESS_COMMAND)))
    {
      ProcessArgInt16(arg_in,String(DEVICE_ADDRESS_COMMAND),DEVICE_ADDRESS_EEPROM_ADDRESS,DEVICE_ADDRESS_EEPROM_LENGTH,true,cli_stream);
    } 
    else if (cmd_in.equalsIgnoreCase(String(SLEEP_ENABLED_COMMAND)))
    {
       ProcessArgByte(arg_in,String(SLEEP_ENABLED_COMMAND),SLEEP_ENABLED_EEPROM_ADDRESS,true,cli_stream);
    }


    else if (cmd_in.equalsIgnoreCase(String(DRIVE_STOP_COMMAND)))
    {
      DriveStop();
    }
    else if (cmd_in.equalsIgnoreCase(String(DRIVE_FORWARD_COMMAND)))
    {
      DriveForward(ArgToUint16(arg_in));
    }
    else if (cmd_in.equalsIgnoreCase(String(DRIVE_BACKWARD_COMMAND)))
    {
      DriveBackward(ArgToUint16(arg_in));
    }
    else if (cmd_in.equalsIgnoreCase(String(DRIVE_LEFT_COMMAND)))
    {
      DriveLeft(ArgToUint16(arg_in));
    }
    else if (cmd_in.equalsIgnoreCase(String(DRIVE_RIGHT_COMMAND)))
    {
      DriveRight(ArgToUint16(arg_in));
    }
    else if (cmd_in.equalsIgnoreCase(String(MOTOR_LEFT_COMMAND)))
    {
      MotorLeft(ArgToInt16(arg_in));
    }
    else if (cmd_in.equalsIgnoreCase(String(MOTOR_RIGHT_COMMAND)))
    {
      MotorRight(ArgToInt16(arg_in));
    }
        else if (cmd_in.equalsIgnoreCase(String(ESC_DUTY_COMMAND)))
    {
      ESC0_SetDuty(ArgToInt16(arg_in));  
    }


    
    else if (cmd_in.equalsIgnoreCase(String(RESET_COMMAND)))
    {
      cli_stream->println("Resetting in 1 second");
      delay(1000);
      ESP.restart();
    }
    else if (cmd_in.equalsIgnoreCase(String(IP_COMMAND)))
    {
      cli_stream->println(String(String(IP_COMMAND) + "=" + (WiFi.localIP().toString())));
    } 
    else if (cmd_in.equalsIgnoreCase(String(VERSION_COMMAND)))
    {
      cli_stream->println(String(String(VERSION_COMMAND) + "=" + String(SOFTWARE_VERSION)));
    } 
    else if(cmd_in.equalsIgnoreCase(String(ACK_COMMAND)))
    {
      cli_stream->println(ACK_REPLY);
    } 
    else
    {
      cli_stream->print("Error: Command = ");
      cli_stream->print(cmd_in);
      cli_stream->print(" Arg = ");
      cli_stream->println(arg_in);
      
      command_valid = false; //Don't update the comms timeout for invalid commands
    }

    //Update the comms timeout if a valid command was received
    if(command_valid == true)
    {
      comms_timeout = millis() + comms_timeout_delay;
    }
  
}



void ProcessArg(String arg_string, String command, int eeprom_address, int eeprom_length, bool read_enable, Stream *cli_stream)
{
      if(IsQuery(arg_string))        //((arg_in.charAt(0) == '?') && ((arg_in.charAt(1) == '\r') || (arg_in.charAt(1) == '\n')))    //(arg_in == String("?"))
      {
        if(read_enable)
        {
          cli_stream->println(String(command + "=" + lobot_eeprom.ReadString(eeprom_address,eeprom_length)));
        }
        else
        {
          cli_stream->println(String(command + "=Private"));
        }
      }
      else if(IsInvalid(arg_string,eeprom_length))     //((arg_in.charAt(0) == '\r') || (arg_in.charAt(0) == '\n') || (arg_in.length() > WIFI_SSID_EEPROM_LENGTH))
      {
        cli_stream->println(String(command + "=invalid"));
      }
      else
      {
        //looks like Serial.readString() is now returning the new line and carrige return characters. Null these out as they screw up the wifi and MQTT setup
        arg_string.replace('\r',0);
        arg_string.replace('\n',0);
        
        lobot_eeprom.WriteString(arg_string,eeprom_address,eeprom_length);
        cli_stream->println(String(command + "=OK"));
      }
}


void ProcessArgInt16(String arg_string, String command, int eeprom_address, int eeprom_length, bool read_enable, Stream *cli_stream)
{
      //looks like Serial.readString() is now returning the new line and carrige return characters. Null these out as they screw up the atoi
      arg_string.replace('\r',0);
      arg_string.replace('\n',0);
      int16_t arg_int = arg_string.toInt();
      
      if(IsQuery(arg_string))        //((arg_in.charAt(0) == '?') && ((arg_in.charAt(1) == '\r') || (arg_in.charAt(1) == '\n')))    //(arg_in == String("?"))
      {
        if(read_enable)
        {
          cli_stream->println(String(command + "=" + String(lobot_eeprom.ReadInt(eeprom_address))));
        }
        else
        {
          cli_stream->println(String(command + "=Private"));
        }
      }
      else if(arg_int > 32767 || arg_int < -32768) //((arg_in.charAt(0) == '\r') || (arg_in.charAt(0) == '\n') || (arg_in.length() > WIFI_SSID_EEPROM_LENGTH))
      {
        cli_stream->println(String(command + "=invalid"));
      }
      else
      {
        lobot_eeprom.WriteInt(arg_int,eeprom_address);
        cli_stream->println(String(command + "=OK"));
      }
}


void ProcessArgUint32(String arg_string, String command, int eeprom_address, int eeprom_length, bool read_enable, Stream *cli_stream)
{
      //looks like Serial.readString() is now returning the new line and carrige return characters. Null these out as they screw up the atoi
      arg_string.replace('\r',0);
      arg_string.replace('\n',0);
      int32_t arg_int = arg_string.toInt();
      
      if(IsQuery(arg_string))        //((arg_in.charAt(0) == '?') && ((arg_in.charAt(1) == '\r') || (arg_in.charAt(1) == '\n')))    //(arg_in == String("?"))
      {
        if(read_enable)
        {
          cli_stream->println(String(command + "=" + String(lobot_eeprom.ReadUint32(eeprom_address))));
        }
        else
        {
          cli_stream->println(String(command + "=Private"));
        }
      }
      else if(arg_int > 4294967295) //((arg_in.charAt(0) == '\r') || (arg_in.charAt(0) == '\n') || (arg_in.length() > WIFI_SSID_EEPROM_LENGTH))
      {
        cli_stream->println(String(command + "=invalid"));
      }
      else
      {
        lobot_eeprom.WriteUint32(arg_int,eeprom_address);
        cli_stream->println(String(command + "=OK"));
      }
}


void ProcessArgByte(String arg_string, String command, int eeprom_address, bool read_enable, Stream *cli_stream)
{
      //looks like Serial.readString() is now returning the new line and carrige return characters. Null these out as they screw up the atoi
      arg_string.replace('\r',0);
      arg_string.replace('\n',0);
      uint8_t arg_byte = arg_string.toInt();
      
      if(IsQuery(arg_string))
      {
        if(read_enable)
        {
          cli_stream->println(String(command + "=" + String(lobot_eeprom.ReadByte(eeprom_address))));
        }
        else
        {
          cli_stream->println(String(command + "=Private"));
        }
      }
      else if(arg_byte > 255 || arg_byte < 0) 
      {
        cli_stream->println(String(command + "=invalid"));
      }
      else
      {
        lobot_eeprom.WriteByte(arg_byte,eeprom_address);
        cli_stream->println(String(command + "=OK"));
      }
}



bool IsQuery(String arg_string)
{
  if((arg_string.charAt(0) == '?') && ((arg_string.charAt(1) == '\r') || (arg_string.charAt(1) == '\n') || (arg_string.charAt(1) == 0)))    //(arg_in == String("?"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool IsInvalid(String arg_string, int max_length)
{
  if((arg_string.charAt(0) == '\r') || (arg_string.charAt(0) == '\n') || (arg_string.length() > max_length))
  {
    return true;
  }
  else
  {
    return false;
  }
}

uint16_t ArgToUint16(String arg_string)
{
  //looks like Serial.readString() is now returning the new line and carrige return characters. Null these out as they screw up the atoi
  arg_string.replace('\r',0);
  arg_string.replace('\n',0);
  //There is a possiblity the argument could be negative so read it into an int32 before taking the abs value
  int32_t arg_int = arg_string.toInt();

  //Cap the value of arg_int to 65536 before casting to uint16
  if (arg_int > 0xFFFF) arg_int = 0xFFFF;
  else if (arg_int < -0xFFFF) arg_int = -0xFFFF;

  //Take the absolute value of arg_int as we are casting to uint16
  arg_int = abs(arg_int);

  //Cast to uint16 and return that
  return uint16_t(arg_int);
}

int16_t ArgToInt16(String arg_string)
{
  //looks like Serial.readString() is now returning the new line and carrige return characters. Null these out as they screw up the atoi
  arg_string.replace('\r',0);
  arg_string.replace('\n',0);
  int16_t arg_int = arg_string.toInt();

  return arg_int;
}


///////////////////////////////////////////////////////////////// Deep Sleep Functions ///////////////////////////////////////////////////////////////////

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
      Serial.print("Wakeup pin was GPIO");
      Serial.println(GetEXT1Source());
      break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\r\n",wakeup_reason); break;
  }
}

uint16_t GetEXT1Source(void)
{
   uint16_t source_mask = esp_sleep_get_ext1_wakeup_status();
   return log(source_mask)/log(2);
}


//////////////////////////////////////////////////////////////////////// RTOS Tasks ///////////////////////////////////////////////////////////////////////
void LEDTaskCode( void * parameter) 
{
  Serial.print("LED Task running on core ");
  Serial.println(xPortGetCoreID());
  while(1) 
  {
    digitalWrite(LOBOT_MINI_LED_PIN, HIGH);
    delay(100);
    digitalWrite(LOBOT_MINI_LED_PIN, LOW);
    delay(200);
    digitalWrite(LOBOT_MINI_LED_PIN, HIGH);
    delay(100);
    digitalWrite(LOBOT_MINI_LED_PIN, LOW);
    delay(800);
  }
}


///////////////////////////////////////////////////////////////// Web Updater Functions //////////////////////////////////////////////////////////////////

/*
 * Login page
 */

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>Conspicor Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='conspicor' && form.pwd.value=='conspicor')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

void WebUpdaterBegin(String host)
{
   /*use mdns for host name resolution*/
  if (!MDNS.begin(host.c_str())) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
}
