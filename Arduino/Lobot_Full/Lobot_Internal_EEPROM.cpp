#include "Lobot_Internal_EEPROM.h"


LobotEEPROM::LobotEEPROM(void)
{
  this->initialized = true;
}

boolean LobotEEPROM::begin(void)
{
  return EEPROM.begin(EEPROM_SIZE);
}

void LobotEEPROM::WriteString(String data,int dataAddress, int dataLength)
{
  for(int i=0; i < dataLength; i++)
  {
    EEPROM.write((dataAddress + i),data[i]);
    //Serial.print(data[i]);
  }  
  EEPROM.commit();

  //Serial.print("the following string has been written to EEPROM: ");
  //Serial.println(data);
}





String LobotEEPROM::ReadString(int dataAddress,int dataLength)
{
  String data;
  for(int i=0;i < dataLength; i++) 
  {
    data = data + char(EEPROM.read(dataAddress + i));
    //Serial.print(EEPROM.read(dataAddress + i));
  }  
  //Serial.print("the following string has been read from EEPROM: ");
  //Serial.println(data);

  return data;
}




int LobotEEPROM::ReadInt(int dataAddress)
{
  //create a union of shared memory space, which we will load with bytes of int stored in EEPROM
  union
  {
    int data;
    byte dataArray[2];
  } udata;

  udata.dataArray[0] = byte(EEPROM.read(dataAddress));
  udata.dataArray[1] = byte(EEPROM.read(dataAddress + 1));

  return udata.data; 
}


void LobotEEPROM::WriteInt(int data,int dataAddress)
{
  //create a union of shared memory space, which we will use to pass the bytes of int out to the EEPROM
  union
  {
    int dataInt;
    byte dataArray[2];
  } udata;

  udata.dataInt = data;

  EEPROM.write(dataAddress, udata.dataArray[0]);
  EEPROM.write((dataAddress + 1), udata.dataArray[1]);

  EEPROM.commit();
}



uint32_t LobotEEPROM::ReadUint32(int dataAddress)
{
  //create a union of shared memory space, which we will load with bytes of int stored in EEPROM
  union
  {
    uint32_t data;
    byte dataArray[4];
  } udata;

  udata.dataArray[0] = byte(EEPROM.read(dataAddress));
  udata.dataArray[1] = byte(EEPROM.read(dataAddress + 1));
  udata.dataArray[2] = byte(EEPROM.read(dataAddress + 2));
  udata.dataArray[3] = byte(EEPROM.read(dataAddress + 3));

  return udata.data; 
}


void LobotEEPROM::WriteUint32(uint32_t data,int dataAddress)
{
  //create a union of shared memory space, which we will use to pass the bytes of int out to the EEPROM
  union
  {
    uint32_t dataInt;
    byte dataArray[4];
  } udata;

  udata.dataInt = data;

  EEPROM.write(dataAddress, udata.dataArray[0]);
  EEPROM.write((dataAddress + 1), udata.dataArray[1]);
  EEPROM.write((dataAddress + 2), udata.dataArray[2]);
  EEPROM.write((dataAddress + 3), udata.dataArray[3]);

  EEPROM.commit();
}


void LobotEEPROM::WriteByte(uint8_t data, int dataAddress)
{
  EEPROM.write(dataAddress, data);

  EEPROM.commit();
}

uint8_t LobotEEPROM::ReadByte(int dataAddress)
{
  return byte(EEPROM.read(dataAddress));
}
