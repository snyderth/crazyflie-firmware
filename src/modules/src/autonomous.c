#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <float.h>
#include <math.h>
#include "stabilizer_types.h"
#include "sound.h"
#include "sitaw.h" //for drop and tumble stuff
#include "commander.h"
#include "ledseq.h"
#include "autonomous.h"

#include "debug.h"
#include "locodeck.h"
#include "sensors.h"
#include "estimator_kalman.h"
#include "sequencer.h"

static void handleStateUninit();
static void exitStateUninit();

static void enterStateWaitPosLock();
static void handleStateWaitPosLock();
static void exitStateWaitPosLock();

static void enterStatePosLocked();
static void handleStatePosLocked();
static void exitStatePosLocked();

static void enterStateTakeOff();
static void handleStateTakeOff();
static void exitStateTakeOff();

static void enterStateLand();
static void handleStateLand();
static void exitStateLand();

static void enterStatePlayPreRecorded();
static void handleStatePlayPreRecorded();
static void exitStatePlayPreRecorded();

static void enterStateStop();
static void handleStateStop();
static void exitStateStop();


// #define SEQUENCE_DATA_CIRCLE
 #define SEQUENCE_DATA_SPIRAL
// #define SEQUENCE_DATA_SQUARE
//#define SEQUENCE_DATA_HOVER
#include "sequences.h"


/*
  Each sequence or actin will correspond to a state that the crazyflie is in
  It can enter the state, handle the state, and exit the state. These are
  Callback functions declared above and defined below
*/
typedef struct {
  void (*enter)();
  void (*handle)();
  void (*exit)();
} state_handler_t;

typedef enum {
  ST_UNINIT = 0,
  ST_WAIT_POS_LOCK,
  ST_POS_LOCKED,
  ST_TAKE_OFF,
  // ST_RECORD_TRACE,
  // ST_RETRACE,
  ST_PLAY_PRE_RECORDED,
  ST_LAND,
  ST_STOP
} at_state_t;


state_handler_t stateHandlers[] = {
  {enter: 0,                         handle: handleStateUninit,                          exit: exitStateUninit},
  {enter: enterStateWaitPosLock,     handle: handleStateWaitPosLock,     exit: exitStateWaitPosLock},
  {enter: enterStatePosLocked,       handle: handleStatePosLocked,       exit: exitStatePosLocked},
  {enter: enterStateTakeOff,         handle: handleStateTakeOff,         exit: exitStateTakeOff},
  // {enter: enterStateRecordTrace,     handle: handleStateRecordTrace,     exit: exitStateRecordTrace},
  // {enter: enterStateRetrace,         handle: handleStateRetrace,         exit: exitStateRetrace},
  {enter: enterStatePlayPreRecorded, handle: handleStatePlayPreRecorded, exit: exitStatePlayPreRecorded},
  {enter: enterStateLand,            handle: handleStateLand,            exit: exitStateLand},
  {enter: enterStateStop,            handle: handleStateStop,            exit: exitStateStop},
};

static xTimerHandle timer;
static bool isInit = false;
static void autonomousFlight(xTimerHandle timer);
static void changeState(at_state_t newState);
static at_state_t state = ST_UNINIT;

static void moveSetPoint(point_t* point);

static setpoint_t setpoint;

#define NR_OF_POSITIONS 2000
static point_t positions[NR_OF_POSITIONS];
static sequence_t sequence;
static sequence_t landSeq;
static sequence_t path;
static sequence_t takeOffSequence;
static bool hasFlown = false;

#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
static uint32_t lockWriteIndex;
static float lockData[LOCK_LENGTH][3];
static void resetLockData();




//Initialize autonomous flight
void autonomousInit(void){
  if(isInit){
    return;
  }
  // // consolePrintf("Initializing Autonomous flight...\n");
  // // consolePrintf("Creating Paths for flight...\n");
  sequenceInit(&sequence, NR_OF_POSITIONS, positions);
  sequenceInit(&path, sizeof(seqDataMain)/sizeof(point_t), seqDataMain);
  sequenceInitStatic(&landSeq, sizeof(seqDataLand)/sizeof(point_t), seqDataLand);
  sequenceInitStatic(&takeOffSequence, sizeof(seqDataLand)/sizeof(point_t), seqDataTakeOff);
  // // consolePrintf("Paths created\n");
  // /****NO CLUE WHAT THESE ARE FOR****/
  // // consolePrintf("Setting flight mode...\n");
  setpoint.setEmergency = false;
  setpoint.resetEmergency = true;
  setpoint.xmode = 0b0111;
  setpoint.ymode = 0b0111;
  setpoint.zmode = 0b0111;
  // /*************/
  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;
  // // consolePrintf("Flight modes set\nReseting Location Lock...\n");
  resetLockData();
  // // consolePrintf("Location Lock reset\n");
  // //create the timer that runs this every time
  // // consolePrintf("Creating OS timer for execution...\n");
  timer = xTimerCreate("AutonomousTimer", M2T(100), pdTRUE, NULL, autonomousFlight);
  //DEBUG_PRINT("Autonomous Timer Started\n");
  xTimerStart(timer, 100);
  // // consolePrintf("Timer created and started\n");
  isInit = true;
}



/*
* Reset lock on position
*/
static void resetLockData() {
  lockWriteIndex = 0;
  for(uint32_t i = 0; i < LOCK_LENGTH; i++) {
    lockData[i][0] = FLT_MAX;
    lockData[i][1] = FLT_MAX;
    lockData[i][2] = FLT_MAX;
  }
}


static bool hasLock() {
  bool result = false;

  //Store current state
  lockData[lockWriteIndex][0] = getVarPX();
  lockData[lockWriteIndex][1] = getVarPY();
  lockData[lockWriteIndex][2] = getVarPZ();

  lockWriteIndex++;
  if(lockWriteIndex >= LOCK_LENGTH) {//wrap around lock data
    lockWriteIndex = 0;
  }

  //check if locked
  int count = 0;

  float lXMax = FLT_MIN;
  float lYMax = FLT_MIN;
  float lZMax = FLT_MIN;

  float lXMin = FLT_MAX;
  float lYMin = FLT_MAX;
  float lZMin = FLT_MAX;

  for (int i = 0; i < LOCK_LENGTH; i++){
    if(lockData[i][0] != FLT_MAX){
      count++;

      lXMax = fmaxf(lXMax, lockData[i][0]);
      lYMax = fmaxf(lYMax, lockData[i][1]);
      lZMax = fmaxf(lZMax, lockData[i][2]);

      lXMin = fminf(lXMax, lockData[i][0]);//DEBUG MAYBE
      lYMin = fminf(lYMin, lockData[i][1]);
      lZMin = fminf(lZMin, lockData[i][2]);

    }
  }

  uint16_t state = locodeckGetAnchorState();
  int anchorCount = 0;
  for (int i = 0; i < 8; i++){
    if((1 << i) & state){
      anchorCount++;
    }
  }

  result = (count >= LOCK_LENGTH) && ((lXMax - lXMin) < LOCK_THRESHOLD) && (lYMax - lYMin < LOCK_THRESHOLD) && ((lZMax - lZMin) < LOCK_THRESHOLD && sensorsAreCalibrated() && anchorCount >= 4);

  return result;
}


static void autonomousFlight(xTimerHandle timer) {

  //This is the function that the OS runs everytime the timer expires
  stateHandlers[state].handle();
  //DEBUG_PRINT("Autonomous flight active\n");

}






bool autonomousTest(void){
  return isInit;
}


/*
* New setpoint, move the crazyflie
*/
static void moveSetPoint(point_t* point){
  static point_t lastpoint;
  static point_t lastVelocity;

  (setpoint.x)[0] = point->x; //position
  (setpoint.x)[1] = (point->x - lastpoint.x)*0.1f; //velocity
  (setpoint.x)[2] = (setpoint.x[1] - lastVelocity.x)*0.1f; //acceleration
  (setpoint.y)[0] = point->y;
  (setpoint.y)[1] = (point->y - lastpoint.y)*0.1f;
  (setpoint.y)[2] = (setpoint.y[1] - lastVelocity.y)*0.1f;
  (setpoint.z)[0] = point->z;
  (setpoint.z)[1] = (point->z - lastpoint.z)*0.1f;
  (setpoint.z)[3] = (setpoint.z[1] - lastVelocity.z)*0.1f; //DEBUG MAYBE
  setpoint.yaw[0] = 0;
  setpoint.yaw[1] = 0;

  setpoint.position = *point;

  commanderSetSetpoint(&setpoint, 3);

  lastpoint = *point;
  lastVelocity.x = setpoint.x[1];
  lastVelocity.y = setpoint.y[1];
  lastVelocity.z = setpoint.z[1];
}


/*
* Enter a new state, see stateHandlers array at top to see what callbacks and
* states there are
*/
static void changeState(at_state_t newState){
  if(state != newState){//if not already in the new state
    stateHandlers[state].exit();//exit the previous state
    state = newState;//change states
    stateHandlers[state].enter();//enter the new state
  }
}



static void enterStateWaitPosLock(){

}
static void handleStateWaitPosLock(){
  DEBUG_PRINT("Waiting for pos lock\n");
  if(hasLock()){
    changeState(ST_POS_LOCKED);
  }
}
static void exitStateWaitPosLock(){
  hasFlown = true;
}


static void enterStatePosLocked(){

}
static void handleStatePosLocked(){
  changeState(ST_TAKE_OFF);
}
static void exitStatePosLocked(){

}


static void enterStateTakeOff(){
  sequenceReset(&takeOffSequence);
}
static void handleStateTakeOff(){
  if(sequenceHasNext(&takeOffSequence)){//If there is another point in the sequence
    point_t* point = sequenceReplay(&takeOffSequence);
    moveSetPoint(point);
  }else{//Reached last point in the takeoff sequence! Time for another thing!
    changeState(ST_PLAY_PRE_RECORDED);
  }
}
static void exitStateTakeOff(){

}



static void enterStateLand(){
  sequenceReset(&landSeq);
}
static void handleStateLand(){
  if(sequenceHasNext(&landSeq)){//Keep coming on down
    point_t* point = sequenceReplay(&landSeq);
    moveSetPoint(point);
  }else{//Last point in landing sequence!
    changeState(ST_STOP);
  }
}
static void exitStateLand(){

}



static void enterStatePlayPreRecorded(){
  sequenceReset(&path);
}
static void handleStatePlayPreRecorded(){
  if(sequenceHasNext(&path)){
    point_t* point = sequenceReplay(&path);//advance one setpoint in the sequence
    moveSetPoint(point);
  }else{//Finished our sequence!
    changeState(ST_LAND);
  }
}
static void exitStatePlayPreRecorded(){

}



static void enterStateStop(){

}
static void handleStateStop(){
  setpoint.setEmergency = true;
  setpoint.resetEmergency = false;
  setpoint.mode.x = modeDisable;
  setpoint.mode.y = modeDisable;
  setpoint.mode.z = modeDisable;
  setpoint.thrust = 0;

  commanderSetSetpoint(&setpoint, 3);
}
static void exitStateStop(){

}



static void handleStateUninit(){
  changeState(ST_WAIT_POS_LOCK);
}
static void exitStateUninit(){
  //changeState(ST_TAKE_OFF);
}
