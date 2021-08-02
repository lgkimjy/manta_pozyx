/*
 *  uwb_data_extract.ino
 *
 *      Author : junyoung kim / lgkimjy
 */

#include "uwb_data_extract.h"

#define MAX_DEVICES    20
uint16_t devId[MAX_DEVICES];
int devices_found;

void setup(){
  Serial.begin(115200);
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  Serial.print("Local device: ");
  Pozyx.getNetworkId(&devId[devices_found]);
  Serial.print("0x");
  Serial.println(devId[0], HEX);
}

void loop(){

  /* Read Pozxy Sensor Data */
  Serial.println("----------------------------------------------------");

  coordinates_t position;
  quaternion_t quaternions;
  acceleration_t acceleration;
  euler_angles_t euler_angles;

  Pozyx.getAcceleration_mg(&acceleration);
  Pozyx.getEulerAngles_deg(&euler_angles);
  Pozyx.getQuaternion(&quaternions);

  int status = checkLocalNewPosition(&position);
  if(status == POZYX_SUCCESS){
    /* prints out the result */
    printCoordinates(position);
  }
  else{
    /* prints out the error code */
    printErrorCode("positioning");
  }
  printAcceleration(acceleration);
  printEulerAngles(euler_angles);
  printQuaternion(quaternions);

  delayMicroseconds(10);
}