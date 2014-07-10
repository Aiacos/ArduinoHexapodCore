/* 
 * File:   init.h
 * Author: Lorenzo
 * Description: Conficuration File
 * Created on 9 novembre 2013, 19.13
 */

#ifndef INIT_H
#define INIT_H

int ledPin = 13;
const int rideHeightOffset = -32; // 32 means body at 100mm off ground

/* Servo IDs */


const int RIGHT_FRONT = 0;
const int RIGHT_MIDDLE = 1;
const int RIGHT_REAR = 2;
const int LEFT_REAR = 3;
const int LEFT_MIDDLE = 4;
const int LEFT_FRONT = 5;


/* Body Dimensions */
const int X_COXA = 85; // MM between front and back legs /2
const int Y_COXA_FB = 60; // MM between front/back legs /2
const int Y_COXA_M = 51; // MM between two middle legs /2
const int COXA_ANGLE = 45; // Angle of coxa from straight forward (deg)

/* Legs */
#define LENGTH_COXA 3
#define LENGTH_FEMUR 9.50
#define LENGTH_TIBIA 12.00

const int SERVO_UPDATE_PERIOD = 20; //ms

/* Parameters for Commander input */
typedef struct {
    int Xspeed;
    int Yspeed;
    int Rspeed;
    int bodyTransX;
    int bodyTransY;
    int bodyTransZ;
    float bodyRotX;
    float bodyRotY;
    float bodyRotZ;
} RECEIVE_DATA_STRUCTURE;
extern RECEIVE_DATA_STRUCTURE commanderInput;

/* Leg parts */
typedef struct {
    int x;
    int y;
    int z;
} footPosStruct;

typedef struct {
    long x;
    long y;
    long z;
} footPosCalcStruct;

typedef struct {
    float coxa;
    float femur;
    float tibia;
} jointAnglesStruct;

typedef struct {
    int coxa;
    int femur;
    int tibia;
} servoPosStruct;

typedef struct {
    int x;
    int y;
    int z;
} initialFootPosStruct;

typedef struct {
    int x;
    int y;
    int z;
} legBasePosStruct;

typedef struct {
    footPosStruct footPos;
    footPosCalcStruct footPosCalc;
    jointAnglesStruct jointAngles;
    servoPosStruct servoPos;
    initialFootPosStruct initialFootPos;
    legBasePosStruct legBasePos;
    float bodyRotZ;
} legStruct;
//extern legStruct        leg[6];

/* Body parts */
typedef struct {
    float rotX;
    float rotY;
    float rotZ;
    int posX;
    int posY;
    int posZ;
} bodyStruct;
//extern bodyStruct body;

/* PERSONAL */
int set[6] = {
    0, 3, 6, 9, 12, 15
};

/* GAIT */
//modifica anche in hexapod::walk()
int caseStep[6] = {1, 3, 1, 3, 1, 3}; //for tripod gait
//int caseStep[6] = {1,3,5,7,9,11}; //for ripple gait

#endif

