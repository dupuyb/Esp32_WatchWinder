#ifndef CalibrationWWM_h
#define CalibrationWWM_h

#include <AccelStepper.h>

#define NV -1 // pos=NV not valid
class CalibrationWWM {
public:
    CalibrationWWM(AccelStepper* stp){
      stepper=stp;
    }

    void init(float speed, float speedMax, float accleration) {
      stepper->setMaxSpeed(speedMax);
      stepper->setAcceleration(speed);
      stepper->setSpeed(accleration);
    }

    bool CalibrationFinished() {
      if ( autoCalRun && !turnCalRun )  autoCalibre();
      if (!autoCalRun &&  turnCalRun )  turnCalibre();  
      return ( !autoCalRun && !turnCalRun ); // 0=false
    } 

    void turnCalibre(){
      dirCcw = stepper->distanceToGo()>0;
      if (turnCalRun==1) { // Init Edge=NV stepper is set to move ccw
        stepper->move(3 * Confwwm.config.oneTurnInStep ); 
        dirCcw = true;
        resetEdge();
      } else {
        if (ccwl!=NV && ccwh!=NV) {
          if (ccwl1!=NV && ccwh1!=NV && ccwl!=ccwl1 && ccwh!=ccwh1) {   // ccw direction only
            Confwwm.config.oneTurnInStep = ( ((ccwl-ccwl1) + (ccwh-ccwh1)) / 2 ); // Set new oneTurnInStep
            stop(); return;
          }
          if (ccwl1==NV) ccwl1=ccwl;
          if (ccwh1==NV) ccwh1=ccwh;
        }
      }
      if (turnCalRun++>90)  stop(); // TimeOut 90s
    }

    void stop() {
      stepper->stop();
      autoCalRun=0; turnCalRun=0;
    }

    void resetNorth() {
      ccwN=NV;acwN=NV;mean=NV;
    }

    void resetEdge() {
      resetNorth();
      ccwl1=NV;ccwh=NV;ccwh=NV;acwh=NV;ccwl=NV;acwl=NV;
    }  

    long getCcwN() {
      if(ccwh!=NV && ccwl!=NV && ccwh>ccwl) { return (ccwh + ccwl)/2; } // Detect ccw mode OK
      return NV;
    }

    long getAcwN() {
      if(acwh!=NV && acwl!=NV && acwh<acwl) { return (acwl + acwh)/2; } // Detect acw mode OK
      return NV;
    }

    // list of king   sens        South              
    // Moving (-->)   ccw ---ccwl____>>_____ccwh-----   Here value of ccwl < ccwh
    // Moving (<--)   acw ---acwh____<<_____acwl----    Here value of acwl > acwh
    void autoCalibre() {
      dirCcw = stepper->distanceToGo()>0;
      if (autoCalRun==1) { // Init Edge=NV stepper is set to move ccw
        resetEdge();
        stepper->move(2 * Confwwm.config.oneTurnInStep ); // Dir ccw for 3 turns
        dirCcw = true;
      }
      if (autoCalRun++>120)  stop(); // TimeOut 120s
      if (ccwN==NV) ccwN = getCcwN();
      if (acwN==NV) acwN = getAcwN();
      if (ccwN!=NV && acwN!=NV) { // North position found
        if (mean==NV) mean = (ccwN+acwN) / 2;
        if (mean==stepper->currentPosition()) { stepper->setCurrentPosition(0); stop(); }
        if (mean!=stepper->targetPosition())  stepper->moveTo(mean);
      } else {
        if (stepper->isRunning() && dirCcw && ccwh!=NV && ccwl!=NV  ) {// comtinu 2 sec
            stepper->move(-2 * Confwwm.config.oneTurnInStep); // Dir acw for 3 turns
        } 
      }
    }

    void forceZero(long val){
      if (val!=NV) {
        resetNorth();
        autoCalRun=2;
      }
    }

    long checkMean() {
      long cc = getCcwN();
      long cw = getAcwN();
      if (cc!=NV && cw!=NV) return (cc+cw) / 2;
      return NV;
    }

    AccelStepper* stepper;
    int turnCalRun = 0; // OFF 
    int autoCalRun = 1; // Start in calibration at beginning
    bool isAtNoth = false;
    bool dirCcw;  // Dir ccw oe acw
    long mean=NV; // mean (ccw+acw)/2
    long ccwN=NV; // North ccw
    long acwN=NV; // North acw
    long ccwh=NV; // Edge Higt ccw
    long acwh=NV; // Edge High acw
    long ccwl=NV; // Edge Low ccw
    long acwl=NV; // Edge Low acw
    long ccwl1; // tempo value 
    long ccwh1;
    bool testStepper=false;
};
#endif