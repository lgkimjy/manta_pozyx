#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

/* Initialize */
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
  Serial.println("-------------------------------------------");

  // initialize the data container
  uint8_t who_am_i;
  uint8_t status = Pozyx.getWhoAmI(&who_am_i);
  
  // check the status to see if the read was successful. Handling failure is covered later.
  if (status == POZYX_SUCCESS) {
    // print the container. Note how a SingleRegister will print as a hex string by default.
    Serial.println(who_am_i, HEX); // will print '0x43'/
  }

  acceleration_t acceleration;
  Pozyx.getAcceleration_mg(&acceleration);
  // printAcceleration(acceleration);

  euler_angles_t euler_angles;
  Pozyx.getEulerAngles_deg(&euler_angles);
  // printEulerAngles(euler_angles);

  quaternion_t quaternions;
  Pozyx.getQuaternion(&quaternions);
  // printQuaternion(quaternions);
 
  delay(100);
}


void printCoordinates(coordinates_t coor){
  Serial.print("POS");
  Serial.print(", x(mm): ");
  Serial.print(coor.x);
  Serial.print(", y(mm): ");
  Serial.print(coor.y);
  Serial.print(", z(mm): ");
  Serial.println(coor.z);
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
