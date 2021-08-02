/*
 *  uwb_data_extract.h
 *
 *      Author : junyoung kim / lgkimjy
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>


int checkLocalNewPosition(coordinates_t *position);

void printCoordinates(coordinates_t coor);
void printErrorCode(String operation);
void printEulerAngles(euler_angles_t euler_angles);
void printAcceleration(acceleration_t acceleration);
void printQuaternion(quaternion_t quaternions);