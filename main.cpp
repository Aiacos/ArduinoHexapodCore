/* 
 * File:   main.cpp
 * Author: Lorenzo
 * Description: Class and Main Loop file
 * Created on 9 novembre 2013, 19.13
 */
#include <Arduino.h>
#include <Servo.h>

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

#include <C:\Users\Lorenzo\Dropbox\Arduino/libraries/EasyTransfer/EasyTransfer.h>
#include <C:\Users\Lorenzo\Dropbox\Arduino/libraries/EasyTransfer/EasyTransfer.cpp>
#include "init.h"

RECEIVE_DATA_STRUCTURE commanderInput;

/**** BEGIN CLASSES DECLARATION ****/
class ServoManager {
private:
    //ATTRIBUTI
    Servo servo[18];
protected:
    //METODI

    /* // COSTRUTTORE ELIMINATO, SOLO NELLA CALSSE HEXAPOD
    servoManager() {
        inizializza();
    }
     */

    void testServo() { //RESET devi riaggiustare gli angoli
        //int testDelay = 2; //o altro valore ES. 25, 0
        //int pAngolo1, pAngolo2, pAngolo3;
        for (int i = 0; i < 6; i++) { //scorre zampa per zampa
            if (set[i] <= 6) {
                movimentoServo(set[i], 75); //deposiziona
                delay(500);
                movimentoServo(set[i], 90); //riposiziona
                delay(500);
                movimentoServo(set[i] + 1, 120); //distende
                delay(500);
                movimentoServo(set[i] + 2, 120); //distende
                delay(550);
                movimentoServo(set[i], 75); //deposiziona
                delay(500);
                movimentoServo(set[i], 90); //riposiziona
                delay(500);
                movimentoServo(set[i] + 1, 45); //ritira
                delay(500);
                movimentoServo(set[i] + 2, 45); //ritira
                delay(550);
            } else {
                movimentoServo(set[i], 180 - 75); //deposiziona
                delay(500);
                movimentoServo(set[i], 180 - 90); //riposiziona
                delay(500);
                movimentoServo(set[i] + 1, 180 - 120); //distende
                delay(500);
                movimentoServo(set[i] + 2, 180 - 120); //distende
                delay(550);
                movimentoServo(set[i], 180 - 75); //deposiziona
                delay(500);
                movimentoServo(set[i], 180 - 90); //riposiziona
                delay(500);
                movimentoServo(set[i] + 1, 180 - 45); //ritira
                delay(500);
                movimentoServo(set[i] + 2, 180 - 45); //ritira
            }
        }
        delay(650);
    } //fine testServo

    void resetServo() { //RESET
        for (int i = 0; i < 18; i++) {
            servo[i].write(90);
        }
    }

    void inizializza() {
        for (int i = 0; i < 18; i++) {
            if (i == 8) {
                servo[i].attach(8);
            } else if (i == 17) {
                servo[i].attach(9);
            } else {
                servo[i].attach(30 + i);
            }
            delay(15);
        }
    }

    void movimentoServo(int j, int pAngolo) { //j è l'indice del servo
        if (pAngolo >= 180) {
            pAngolo = 179;
            servo[j].write(pAngolo);
        } else if (pAngolo <= 0) {
            pAngolo = 0;
            servo[j].write(pAngolo);
        } else {
            servo[j].write(pAngolo);
        }
        delay(5);
    } //fine movimentoServo
public:
};

class InverseKinematics : ServoManager {
    /******************************************************************************
     * Inverse Kinematics for hexapod
     *
     * FRONT VIEW       ^        ==0         0==
     *     /\___/\      |       |  0==[___]==0  |
     *    /       \     -Z      |               |
     *
     * TOP VIEW
     *    \       /     ^
     *     \_____/      |
     *  ___|     |___   X
     *     |_____|
     *     /     \      Y->
     *    /       \
     *****************************************************************************/
private:

protected:
    legStruct leg[6];

    /*********************************************************************************************************
        runIK()
     **********************************************************************************************************/
    void runIK() {
        footPosCalc();
        legIK();
        driveServos();
    }

    /**********************************************************************************************************
        footPosCalc()
        Calculates necessary foot position (leg space) to acheive commanded body rotations, translations, and gait inputs
     ***********************************************************************************************************/
    void footPosCalc() {

        float sinRotX, cosRotX, sinRotY, cosRotY, sinRotZ, cosRotZ;
        int totalX, totalY, totalZ;
        int tempFootPosX[6], tempFootPosY[6], tempFootPosZ[6];
        int bodyRotOffsetX[6], bodyRotOffsetY[6], bodyRotOffsetZ[6];

        sinRotX = sin(radians(-commanderInput.bodyRotX));
        cosRotX = cos(radians(-commanderInput.bodyRotX));
        sinRotY = sin(radians(-commanderInput.bodyRotY));
        cosRotY = cos(radians(-commanderInput.bodyRotY));
        sinRotZ = sin(radians(-commanderInput.bodyRotZ));
        cosRotZ = cos(radians(-commanderInput.bodyRotZ));

        for (int legNum = 0; legNum < 6; legNum++) {
            //Serial.print ("footPosCalc() Leg: "); Serial.println (legNum+1);

            //sinRotZ = sin(radians(leg[legNum].bodyRotZ - commanderInput.bodyRotZ));
            //cosRotZ = cos(radians(leg[legNum].bodyRotZ - commanderInput.bodyRotZ));

            totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;
            totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
            totalZ = leg[legNum].initialFootPos.z + leg[legNum].legBasePos.z;

            bodyRotOffsetX[legNum] = round((totalY * cosRotY * sinRotZ + totalY * cosRotZ * sinRotX * sinRotY + totalX * cosRotZ * cosRotY - totalX * sinRotZ * sinRotX * sinRotY - totalZ * cosRotX * sinRotY) - totalX);
            bodyRotOffsetY[legNum] = round(totalY * cosRotX * cosRotZ - totalX * cosRotX * sinRotZ + totalZ * sinRotX - totalY);
            bodyRotOffsetZ[legNum] = round((totalY * sinRotZ * sinRotY - totalY * cosRotZ * cosRotY * sinRotX + totalX * cosRotZ * sinRotY + totalX * cosRotY * sinRotZ * sinRotX + totalZ * cosRotX * cosRotY) - totalZ);

            // Calculated foot positions to acheive xlation/rotation input. Not coxa mounting angle corrected
            tempFootPosX[legNum] = leg[legNum].initialFootPos.x + bodyRotOffsetX[legNum] - commanderInput.bodyTransX + leg[legNum].footPos.x;
            tempFootPosY[legNum] = leg[legNum].initialFootPos.y + bodyRotOffsetY[legNum] - commanderInput.bodyTransY + leg[legNum].footPos.y;
            tempFootPosZ[legNum] = leg[legNum].initialFootPos.z + bodyRotOffsetZ[legNum] - commanderInput.bodyTransZ + leg[legNum].footPos.z;
        }

        // Rotates X,Y about coxa to compensate for coxa mounting angles.
        leg[0].footPosCalc.x = round(tempFootPosY[0] * cos(radians(COXA_ANGLE)) - tempFootPosX[0] * sin(radians(COXA_ANGLE)));
        leg[0].footPosCalc.y = round(tempFootPosY[0] * sin(radians(COXA_ANGLE)) + tempFootPosX[0] * cos(radians(COXA_ANGLE)));
        leg[0].footPosCalc.z = tempFootPosZ[0];
        leg[1].footPosCalc.x = round(tempFootPosY[1] * cos(radians(COXA_ANGLE * 2)) - tempFootPosX[1] * sin(radians(COXA_ANGLE * 2)));
        leg[1].footPosCalc.y = round(tempFootPosY[1] * sin(radians(COXA_ANGLE * 2)) + tempFootPosX[1] * cos(radians(COXA_ANGLE * 2)));
        leg[1].footPosCalc.z = tempFootPosZ[1];
        leg[2].footPosCalc.x = round(tempFootPosY[2] * cos(radians(COXA_ANGLE * 3)) - tempFootPosX[2] * sin(radians(COXA_ANGLE * 3)));
        leg[2].footPosCalc.y = round(tempFootPosY[2] * sin(radians(COXA_ANGLE * 3)) + tempFootPosX[2] * cos(radians(COXA_ANGLE * 3)));
        leg[2].footPosCalc.z = tempFootPosZ[2];
        leg[3].footPosCalc.x = round(tempFootPosY[3] * cos(radians(COXA_ANGLE * 5)) - tempFootPosX[3] * sin(radians(COXA_ANGLE * 5)));
        leg[3].footPosCalc.y = round(tempFootPosY[3] * sin(radians(COXA_ANGLE * 5)) + tempFootPosX[3] * cos(radians(COXA_ANGLE * 5)));
        leg[3].footPosCalc.z = tempFootPosZ[3];
        leg[4].footPosCalc.x = round(tempFootPosY[4] * cos(radians(COXA_ANGLE * 6)) - tempFootPosX[4] * sin(radians(COXA_ANGLE * 6)));
        leg[4].footPosCalc.y = round(tempFootPosY[4] * sin(radians(COXA_ANGLE * 6)) + tempFootPosX[4] * cos(radians(COXA_ANGLE * 6)));
        leg[4].footPosCalc.z = tempFootPosZ[4];
        leg[5].footPosCalc.x = round(tempFootPosY[5] * cos(radians(COXA_ANGLE * 7)) - tempFootPosX[5] * sin(radians(COXA_ANGLE * 7)));
        leg[5].footPosCalc.y = round(tempFootPosY[5] * sin(radians(COXA_ANGLE * 7)) + tempFootPosX[5] * cos(radians(COXA_ANGLE * 7)));
        leg[5].footPosCalc.z = tempFootPosZ[5];

        //for( int legNum=0; legNum<6; legNum++){ 
        //Serial.print ("footPosCalc() Leg: "); Serial.println (legNum+1);
        //Serial.print("footPosCalcX: "); Serial.println(leg[legNum].footPosCalc.x);  //these are off by +/- 1
        //Serial.print("footPosCalcY: "); Serial.println(leg[legNum].footPosCalc.y);
        //Serial.print("footPosCalcZ: "); Serial.println(leg[legNum].footPosCalc.z);
        //}

    }

    /**************************************************************************************************************
        legIK()
        Translates foot x,y,z positions (body space) to leg space and adds goal foot positon input (leg space).
        Calculates the coxa, femur, and tibia angles for these foot positions (leg space).
     ***************************************************************************************************************/
    void legIK() {

        float CoxaFootDist, IKSW, IKA1, IKA2, tibAngle;

        for (int legNum = 0; legNum < 6; legNum++) {

            //Serial.print ("legIK() Leg: "); Serial.println (legNum+1);

            CoxaFootDist = sqrt(sq(leg[legNum].footPosCalc.y) + sq(leg[legNum].footPosCalc.x));
            IKSW = sqrt(sq(CoxaFootDist - LENGTH_COXA) + sq(leg[legNum].footPosCalc.z));
            IKA1 = atan2((CoxaFootDist - LENGTH_COXA), leg[legNum].footPosCalc.z);
            IKA2 = acos((sq(LENGTH_TIBIA) - sq(LENGTH_FEMUR) - sq(IKSW)) / (-2 * IKSW * LENGTH_FEMUR));
            tibAngle = acos((sq(IKSW) - sq(LENGTH_TIBIA) - sq(LENGTH_FEMUR)) / (-2 * LENGTH_FEMUR * LENGTH_TIBIA));

            leg[legNum].jointAngles.coxa = degrees(atan2(leg[legNum].footPosCalc.y, leg[legNum].footPosCalc.x));
            leg[legNum].jointAngles.femur = degrees(IKA1 + IKA2);
            leg[legNum].jointAngles.tibia = degrees(tibAngle);

            /*
            leg[legNum].jointAngles.coxa  = 90 - degrees( atan2( leg[legNum].footPosCalc.y , leg[legNum].footPosCalc.x) ); 
            leg[legNum].jointAngles.femur = 90 - degrees( IKA1 + IKA2 );
            leg[legNum].jointAngles.tibia = 90 - degrees( tibAngle );
             */

            //Serial.print("Coxa Angle: "); Serial.println(leg[legNum].jointAngles.coxa);
            //Serial.print("Femur Angle: "); Serial.println(leg[legNum].jointAngles.femur); 
            //Serial.print("Tibia Angle: "); Serial.println(leg[legNum].jointAngles.tibia);
        }

        // Applies necessary corrections to servo angles to account for hardware

        /*
        for( int legNum=0; legNum<3; legNum++ ){
            leg[legNum].jointAngles.coxa  = leg[legNum].jointAngles.coxa;
            leg[legNum].jointAngles.femur = leg[legNum].jointAngles.femur - 13.58;              // accounts for offset servo bracket on femur
            leg[legNum].jointAngles.tibia = leg[legNum].jointAngles.tibia - 48.70 + 13.58 + 90; //counters offset servo bracket on femur, accounts for 90deg mounting, and bend of tibia
        }    
       for( int legNum=3; legNum<6; legNum++ ){
     
            leg[legNum].jointAngles.coxa  =   leg[legNum].jointAngles.coxa;
            leg[legNum].jointAngles.femur = -(leg[legNum].jointAngles.femur - 13.58);
            leg[legNum].jointAngles.tibia = -(leg[legNum].jointAngles.tibia - 48.70 + 13.58 + 90);
        }
         */

    }

    /*************************************************
        driveServos()
     **************************************************/
    void driveServos() { //USA IK
        //X:distanza dalla base, Y:altezza del corpo, Z:rotazione di root (sin(a))
        for (int legNum = 0; legNum < 6; legNum++) {
            int angolo1, angolo2, angolo3;
            angolo1 = leg[legNum].jointAngles.coxa;
            angolo2 = leg[legNum].jointAngles.femur;
            angolo3 = leg[legNum].jointAngles.tibia;

            if (legNum < 3) {
                movimentoServo(set[legNum], angolo1);
                movimentoServo(set[legNum] + 1, 180 - angolo2);
                movimentoServo(set[legNum] + 2, angolo3);
            } else {
                movimentoServo(set[legNum], 180 - angolo1);
                movimentoServo(set[legNum] + 1, angolo2);
                movimentoServo(set[legNum] + 2, 180 - angolo3);
            }
        }
    }

    void initInverseKinematics() {
        inizializza();
        /* INITIAL FOOT POSITIONS */
        leg[RIGHT_FRONT].initialFootPos.x = round(sin(radians(COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[RIGHT_FRONT].initialFootPos.y = round(cos(radians(COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[RIGHT_FRONT].initialFootPos.z = LENGTH_TIBIA - 7; // + rideHeightOffset;
        leg[RIGHT_FRONT].legBasePos.x = X_COXA;
        leg[RIGHT_FRONT].legBasePos.y = Y_COXA_FB;
        leg[RIGHT_FRONT].legBasePos.z = 0;
        leg[RIGHT_MIDDLE].initialFootPos.x = 0;
        leg[RIGHT_MIDDLE].initialFootPos.y = (LENGTH_COXA + LENGTH_FEMUR);
        leg[RIGHT_MIDDLE].initialFootPos.z = LENGTH_TIBIA - 7; // + rideHeightOffset;
        leg[RIGHT_MIDDLE].legBasePos.x = 0;
        leg[RIGHT_MIDDLE].legBasePos.y = Y_COXA_M;
        leg[RIGHT_MIDDLE].legBasePos.z = 0;
        leg[RIGHT_REAR].initialFootPos.x = round(sin(radians(-COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[RIGHT_REAR].initialFootPos.y = round(cos(radians(COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[RIGHT_REAR].initialFootPos.z = LENGTH_TIBIA - 7; // + rideHeightOffset;
        leg[RIGHT_REAR].legBasePos.x = -X_COXA;
        leg[RIGHT_REAR].legBasePos.y = Y_COXA_FB;
        leg[RIGHT_REAR].legBasePos.z = 0;
        leg[LEFT_REAR].initialFootPos.x = round(sin(radians(-COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[LEFT_REAR].initialFootPos.y = -round(cos(radians(COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[LEFT_REAR].initialFootPos.z = LENGTH_TIBIA - 7; // + rideHeightOffset;
        leg[LEFT_REAR].legBasePos.x = -X_COXA;
        leg[LEFT_REAR].legBasePos.y = -Y_COXA_FB;
        leg[LEFT_REAR].legBasePos.z = 0;
        leg[LEFT_MIDDLE].initialFootPos.x = 0;
        leg[LEFT_MIDDLE].initialFootPos.y = -(LENGTH_COXA + LENGTH_FEMUR);
        leg[LEFT_MIDDLE].initialFootPos.z = LENGTH_TIBIA - 7; // + rideHeightOffset;
        leg[LEFT_MIDDLE].legBasePos.x = 0;
        leg[LEFT_MIDDLE].legBasePos.y = -Y_COXA_M;
        leg[LEFT_MIDDLE].legBasePos.z = 0;
        leg[LEFT_FRONT].initialFootPos.x = round(sin(radians(COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[LEFT_FRONT].initialFootPos.y = -round(cos(radians(COXA_ANGLE))*(LENGTH_COXA + LENGTH_FEMUR));
        leg[LEFT_FRONT].initialFootPos.z = LENGTH_TIBIA - 7; // + rideHeightOffset;
        leg[LEFT_FRONT].legBasePos.x = X_COXA;
        leg[LEFT_FRONT].legBasePos.y = -Y_COXA_FB;
        leg[LEFT_FRONT].legBasePos.z = 0;
    }
public:
};

class Hexapod : InverseKinematics {
private:
    int tick;
public:

    void initHexapod() {
        //inizializza();
        initInverseKinematics();
    }

    void tripodGait() {

        float sinRotZ, cosRotZ;
        int totalX, totalY;
        int strideRotOffsetX[6], strideRotOffsetY[6];
        int height = -35;
        int duration;
        int numTicks;
        int speedX, speedY, speedR;
        //int strideX[6], strideY[6];

        if ((abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5)) {

            duration = 600; //duration of one step cycle (ms)      
            numTicks = duration / SERVO_UPDATE_PERIOD / 4; //total ticks divided into the four cases   

            speedX = 180 * commanderInput.Xspeed / 127; //200mm/s top speed
            speedY = 180 * commanderInput.Yspeed / 127; //200mm/s top speed
            speedR = 40 * commanderInput.Rspeed / 127; //40deg/s top rotation speed

            sinRotZ = sin(radians(speedR));
            cosRotZ = cos(radians(speedR));

            for (int legNum = 0; legNum < 6; legNum++) {

                totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;
                totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;

                strideRotOffsetX[legNum] = round(totalY * sinRotZ + totalX * cosRotZ - totalX);
                strideRotOffsetY[legNum] = round(totalY * cosRotZ - totalX * sinRotZ - totalY);

                if (abs(speedR * 5) > abs(speedX) && abs(speedR * 5) > abs(speedY)) height = -abs(speedR);
                else {
                    if (abs(speedX) >= abs(speedY)) height = -abs(speedX / 5);
                    else height = -abs(speedY / 5);
                }


                switch (caseStep[legNum]) {

                    case 1: //forward raise

                        leg[legNum].footPos.x = ((long) (speedX + strideRotOffsetX[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration - (speedX + strideRotOffsetX[legNum]) / 4;
                        leg[legNum].footPos.y = ((long) (speedY + strideRotOffsetY[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration - (speedY + strideRotOffsetY[legNum]) / 4;
                        leg[legNum].footPos.z = ((long) height * tick * SERVO_UPDATE_PERIOD) / (duration / 4);

                        if (tick >= numTicks - 1) caseStep[legNum] = 2;
                        break;

                    case 2: // forward lower

                        leg[legNum].footPos.x = ((long) (speedX + strideRotOffsetX[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration;
                        leg[legNum].footPos.y = ((long) (speedY + strideRotOffsetY[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration;
                        leg[legNum].footPos.z = height - ((long) height * tick * SERVO_UPDATE_PERIOD) / (duration / 4);

                        if (tick >= numTicks - 1) caseStep[legNum] = 3;
                        break;

                    case 3: // down pull back

                        leg[legNum].footPos.x = -((long) (speedX + strideRotOffsetX[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration + (speedX + strideRotOffsetX[legNum]) / 4;
                        leg[legNum].footPos.y = -((long) (speedY + strideRotOffsetY[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration + (speedY + strideRotOffsetY[legNum]) / 4;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 4;
                        break;

                    case 4: // down pull back

                        leg[legNum].footPos.x = -((long) (speedX + strideRotOffsetX[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration;
                        leg[legNum].footPos.y = -((long) (speedY + strideRotOffsetY[legNum]) * tick * SERVO_UPDATE_PERIOD) / duration;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 1;
                        break;

                }// end of case statement

            }// end of loop over legs
            if (tick < numTicks - 1) tick++;
            else tick = 0;

        }//end if joystick active

    }

    void rippleGait() {

        float sinRotZ, cosRotZ;
        int totalX, totalY;
        int strideRotOffsetX[6], strideRotOffsetY[6];
        int height[6];
        int duration;
        int numTicks;
        int speedX, speedY, speedR;
        int strideX[6], strideY[6];

        if ((abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5)) {
            Serial.print("***TICK***: ");
            Serial.println(tick);

            duration = 1000; //3000ms  
            Serial.print("duration: ");
            Serial.println(duration);

            numTicks = duration / SERVO_UPDATE_PERIOD / 4; //total ticks divided into the 12 cases   
            //Serial.print("numTicks: "); Serial.println(numTicks);

            speedX = 100 * commanderInput.Xspeed / 127; //100mm/s top speed
            //Serial.print("speedX: "); Serial.println(speedX);
            speedY = 100 * commanderInput.Yspeed / 127; //100mm/s top speed
            //Serial.print("speedY: "); Serial.println(speedY);
            speedR = 15 * commanderInput.Rspeed / 127; //15deg/s top rotation speed
            //Serial.print("speedR: "); Serial.println(speedR);

            sinRotZ = sin(radians(speedR));
            cosRotZ = cos(radians(speedR));


            for (int legNum = 0; legNum < 6; legNum++) {
                //Serial.print("Leg: "); Serial.println(legNum+1);

                totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;
                totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;

                strideRotOffsetX[legNum] = round(totalY * sinRotZ + totalX * cosRotZ - totalX);
                //Serial.print("strideRotOffsetX: "); Serial.println(strideRotOffsetX[legNum]);
                strideRotOffsetY[legNum] = round(totalY * cosRotZ - totalX * sinRotZ - totalY);
                //Serial.print("strideRotOffsetY: "); Serial.println(strideRotOffsetY[legNum]);

                strideX[legNum] = speedX * duration / 1000 + strideRotOffsetX[legNum]; // speedX*duration/1000 : mm/s * s
                //Serial.print("strideX: "); Serial.println(strideX[legNum]);
                strideY[legNum] = speedY * duration / 1000 + strideRotOffsetY[legNum];
                //Serial.print("strideY: "); Serial.println(strideY[legNum]);

                if (abs(strideX[legNum]) >= abs(strideY[legNum])) height[legNum] = -abs(strideX[legNum] / 2);
                else height[legNum] = -abs(strideY[legNum] / 2);
                //Serial.print("height: "); Serial.println(height[legNum]);   

                switch (caseStep[legNum]) {

                    case 1: //forward raise

                        //Serial.print("1ST QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = (strideX[legNum] * tick) / (2 * numTicks) - strideX[legNum] / 2;
                        leg[legNum].footPos.y = (strideY[legNum] * tick) / (2 * numTicks) - strideY[legNum] / 2;
                        leg[legNum].footPos.z = (height[legNum] * tick) / numTicks;

                        if (tick >= numTicks - 1) caseStep[legNum] = 2;
                        break;

                    case 2: // forward lower

                        //Serial.print("2ND QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = (strideX[legNum] * tick) / (2 * numTicks);
                        leg[legNum].footPos.y = (strideY[legNum] * tick) / (2 * numTicks);
                        leg[legNum].footPos.z = height[legNum] - (height[legNum] * tick) / numTicks;

                        if (tick >= numTicks - 1) caseStep[legNum] = 3;
                        break;

                    case 3: // down pull back

                        //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) + strideX[legNum] / 2;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) + strideY[legNum] / 2;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 4;
                        break;

                    case 4: // down pull back

                        //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) + strideX[legNum]*2 / 5;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) + strideY[legNum]*2 / 5;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 5;
                        break;

                    case 5: // down pull back

                        //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) + strideX[legNum]*3 / 10;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) + strideY[legNum]*3 / 10;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 6;
                        break;

                    case 6: // down pull back

                        //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) + strideX[legNum] / 5;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) + strideY[legNum] / 5;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 7;
                        break;

                    case 7: // down pull back

                        //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) + strideX[legNum] / 10;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) + strideY[legNum] / 10;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 8;
                        break;

                    case 8: // down pull back

                        //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks);
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks);
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 9;
                        break;

                    case 9: // down pull back

                        //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) - strideX[legNum] / 10;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) - strideY[legNum] / 10;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 10;
                        break;

                    case 10: // down pull back

                        //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) - strideX[legNum] / 5;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) - strideY[legNum] / 5;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 11;
                        break;

                    case 11: // down pull back

                        //Serial.print("3RD QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) - strideX[legNum]*3 / 10;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) - strideY[legNum]*3 / 10;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 12;
                        break;

                    case 12: // down pull back

                        //Serial.print("4TH QUARTER  Tick: "); Serial.println(tick);
                        leg[legNum].footPos.x = -(strideX[legNum] * tick) / (10 * numTicks) - strideX[legNum]*2 / 5;
                        leg[legNum].footPos.y = -(strideY[legNum] * tick) / (10 * numTicks) - strideY[legNum]*2 / 5;
                        leg[legNum].footPos.z = 0;

                        if (tick >= numTicks - 1) caseStep[legNum] = 1;
                        break;

                } // end of case statement

                //Serial.print("footPos.x: "); Serial.println(leg[legNum].footPos.x);
                //Serial.print("footPos.y: "); Serial.println(leg[legNum].footPos.y);
                //Serial.print("footPos.z: "); Serial.println(leg[legNum].footPos.z);


            }//loop over legs
            if (tick < numTicks - 1) tick++;
            else tick = 0;

        }//end if joystick active
    }

    void walk() {
        runIK();
        //Serial.print("time after runIK(): "); Serial.println(millis()-currentTime);
        tripodGait();
        //myEsapodo.rippleGait();
        //Serial.print("time after tripodWalk(): "); Serial.println(millis()-currentTime);
    }
};

/**** END CLASSES ****/

/**** INSTANCES AND VARIABLES ****/

EasyTransfer ET;
Hexapod myHexapod;
long currentTime;
long previousTime;

/**** END INSTANCES AND VARIABLES ****/

void setup() {
    /**** PROCEDURA DI ACCENSIONE ****/
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    myHexapod.initHexapod();
    Serial.println("Aiacos Hexapod");
    delay(1000);

    Serial.begin(9600);
    Serial1.begin(57600);
    ET.begin(details(commanderInput), &Serial1);
    myHexapod.walk();
    delay(500);
    digitalWrite(ledPin, LOW);
    //myEsapodo.resetServo();
    //myEsapodo.testServo();
}

void loop() {
    currentTime = millis();
    if (currentTime - previousTime >= SERVO_UPDATE_PERIOD) { // wait until its been 20ms for servo update  
        previousTime = currentTime;

        //Serial.println("Crunching... "); 
        // readCommandInputs(); // Read in input from controller
        if (ET.receiveData() > 0) {
            digitalWrite(ledPin, HIGH);
            ET.receiveData();
            digitalWrite(ledPin, LOW);
            Serial.println("Dati Ricevuti");
            //Serial.print("time after readCommandInputs(): "); Serial.println(millis()-currentTime);
            myHexapod.walk();
        }
    }
} //end main loop()
