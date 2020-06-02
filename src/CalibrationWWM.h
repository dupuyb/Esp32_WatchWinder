#ifndef CalibrationWWM_h
#define CalibrationWWM_h

#include <AccelStepper.h>

class CalibrationWWM {
public:
    CalibrationWWM(AccelStepper* stp){
        stepper=stp;
    }

    bool CalibrationFinished() {
        if (autoCalRun && !manuCalRun) 
            autoCalibre();
        if (manuCalRun && !autoCalRun) 
            manualCalibre();
        return (!manuCalRun && !autoCalRun ) ;
    }

    uint8_t initMove(uint8_t cCM) {
        //Serial.printf(">>> cadranCalibMode:%d START\n\r", cCM);
        pIn1 = pOut1 = pIn2 = pOut2 = 0;
        stepper->setCurrentPosition(1);
        stepper->move(oneTurnStep*3);
        return cCM + 1;
    }
    uint8_t middleEdge(uint8_t cCM){
        long turn = abs(pOut1-pOut2);
        long pos;
        oneTurnStep = turn;
        pos = pIn2;
        //Serial.printf(">>> cadranCalibMode:%d GOTO turn1=%ld pos=%ld \n\r", cCM,turn, pos) ;
        stepper->moveTo(pos);
        return cCM + 1;         
    }

    int manualCalibre() {
        switch (manuCalRun) {
            // Zero position setup
            case 1: // Start one trun
                countDelay = 0;
                manuCalRun = initMove(manuCalRun);
            break;
            case 2: // First interrupt ______pIn-----pOut____
            if ((pIn2 = !0) && (pOut2 != 0))  {
                if (stepper->isRunning()) {
                  stepper->stop();
                } else {
                  manuCalRun = middleEdge(manuCalRun);
                }
            }
            break;
            case 3: // Wait position is final destination
            if (!stepper->isRunning()) {
                //Serial.printf(">>> manuCalRun:%d FINISHED pos=%ld \n\r", manuCalRun, stepper->currentPosition());
                stepper->setCurrentPosition(0);
                manuCalRun = 4;
            } 
            // Delay protection.
            if (countDelay++>180)
                manuCalRun = 0;
            break;
            case 4:  {
                manuCalRun = 0;
            }
            break;
        }
        return manuCalRun;
    }

    int  autoCalibre() {
        // A voir pour ajouter un offset d'angle comme parametre.
        if (autoCalRun != 0) {
            if (autoCalRun==1) { // Start 
              pOut1 = pIn1 = pOut2 = pIn2 = 0;
            }
            long pos = stepper->currentPosition();
            autoCalRun++;
            if (pOut1==0){
                stepper->move(autoCalStep);
                if ( (pos > autoCalOut) && autoCalStep > 0) // Not Edge found in this direction
                autoCalStep=-autoCalStep; // No Edge back to other direction
            } else {
              if (pIn1==0){
                stepper->move(-autoCalStep);
              } else {
                stepper->moveTo(pIn1);
                if (!stepper->isRunning()) {
                  stepper->setCurrentPosition(0);
                  //Serial.printf("ClockDir autoCalibre FINisKED \n\r");
                  autoCalRun=0; // Is finish
                }
              } 
            } 
           //Serial.printf("autoCalibre pOut1:%ld, pIn1:%ld pos=%ld \n\r", pOut1, pIn1, pos);
        }
        return autoCalRun;
    }

    void init() {
        stepper->setMaxSpeed(800.0);
        stepper->setAcceleration(100.0);
        stepper->setSpeed(400);
    }

    uint8_t manuCalRun = 0; // OFF 
    int autoCalRun=1;       // Start aut calibration at beginning
    long pIn1;
    long pIn2;
    long pOut1;
    long pOut2;
    AccelStepper* stepper;
    long oneTurnStep = 4098;
    long autoCalOut=600;
    long autoCalStep=50;
    int countDelay;

};
#endif