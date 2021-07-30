/*
 *  uwb_data_extraction.ino
 *
 *      Author : junyoung kim / lgkimjy
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>


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
    // printAcceleration(acceleration);
    // printEulerAngles(euler_angles);
    // printQuaternion(quaternions);
  }
  else{
    /* prints out the error code */
    printErrorCode("positioning");
  }
}

int checkLocalNewPosition(coordinates_t *position)
{
  assert(position != NULL);
  int status;
  uint8_t int_status = 0;
  /* now wait for the positioning to finish or generate an error */
  if(Pozyx.waitForFlag_safe(POZYX_INT_STATUS_POS | POZYX_INT_STATUS_ERR, 2*POZYX_DELAY_INTERRUPT, &int_status)){
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR){
      /* An error occured during positioning */
      return POZYX_FAILURE;
    }
    else{
      status = Pozyx.getCoordinates(position);
      return status;
    }
  }
  else{
    return POZYX_TIMEOUT;
  }
}

void printCoordinates(coordinates_t coor){
  Serial.print("POSITION     :");
  Serial.print(" x(mm): ");
  Serial.print(coor.x);
  Serial.print(", y(mm): ");
  Serial.print(coor.y);
  Serial.print(", z(mm): ");
  Serial.println(coor.z);
}

void printErrorCode(String operation){
  uint8_t error_code;
  Pozyx.getErrorCode(&error_code);
  Serial.print("ERROR ");
  Serial.print(operation);
  Serial.print(", local error code: 0x");
  Serial.println(error_code, HEX);
}

void printEulerAngles(euler_angles_t euler_angles){
  Serial.print("Euler Angle  :");
  Serial.print(" r: ");
  Serial.print(euler_angles.roll);
  Serial.print(", p: ");
  Serial.print(euler_angles.pitch);
  Serial.print(", y: ");
  Serial.println(euler_angles.heading); 
}

void printAcceleration(acceleration_t acceleration){
  Serial.print("Acceleration :");
  Serial.print(" x: ");
  Serial.print(acceleration.x);
  Serial.print(", y: ");
  Serial.print(acceleration.y);
  Serial.print(", z: ");
  Serial.println(acceleration.z);
}

void printQuaternion(quaternion_t quaternions){
  Serial.print("Quaternion   :");
  Serial.print(" w: ");
  Serial.print(quaternions.weight);
  Serial.print(", x: ");
  Serial.print(quaternions.x);
  Serial.print(", y: ");
  Serial.print(quaternions.y);
  Serial.print(", z: ");
  Serial.println(quaternions.z);
}
