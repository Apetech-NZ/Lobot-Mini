#ifndef LobotInternalEEPROM_h
#define LobotInternalEEPROM_h

#include <Arduino.h>

//For EEPROM
#include "EEPROM.h"
#define EEPROM_SIZE 1024

#define WIFI_SSID_EEPROM_ADDRESS 32
#define WIFI_SSID_EEPROM_LENGTH 32

#define WIFI_KEY_EEPROM_ADDRESS 64
#define WIFI_KEY_EEPROM_LENGTH 64

#define DATA_RATE_ADDRESS  128
#define DATA_RATE_LENGTH 4

#define DEVICE_ADDRESS_EEPROM_ADDRESS 150
#define DEVICE_ADDRESS_EEPROM_LENGTH 2

#define WIFI_ENABLED_EEPROM_ADDRESS  160
#define WIFI_ENABLED_EEPROM_LENGTH  1

#define MQTT_BROKER_EEPROM_ADDRESS  170
#define MQTT_BROKER_EEPROM_LENGTH   32

#define MQTT_PASS_EEPROM_ADDRESS    210
#define MQTT_PASS_EEPROM_LENGTH     32

#define MQTT_USER_EEPROM_ADDRESS    245
#define MQTT_USER_EEPROM_LENGTH     32

#define SLEEP_ENABLED_EEPROM_ADDRESS  280
#define SLEEP_ENABLED_EEPROM_ADDRESS  1

class LobotEEPROM 
{
    private:
    boolean initialized;
    
    public:
    LobotEEPROM();
    boolean begin(void);
    void WriteString(String data,int dataAddress, int dataLength);
    String ReadString(int dataAddress,int dataLength);
    int ReadInt(int dataAddress);
    void WriteInt(int data,int dataAddress);
    uint32_t ReadUint32(int dataAddress);
    void WriteUint32(uint32_t data, int dataAddress);
    void WriteByte(uint8_t data, int dataAddress);
    uint8_t ReadByte(int dataAddress);
};



#endif //LobotInternalEEPROM_h
