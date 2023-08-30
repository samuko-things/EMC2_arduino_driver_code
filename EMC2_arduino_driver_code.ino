#include <util/atomic.h>
#include <Wire.h>
#include "EMC2_global_params_eeprom.h"

///////// my sepcial delay function ///////////////
void delayMs(int ms) {
  for (int i = 0; i < ms; i += 1) {
    delayMicroseconds(1000);
  }
}
//////////////////////////////////////////////////


#include "low_pass_filter_setup.h"

// Filter instance
AdaptiveLowPassFilter lpfA(orderA, cutOffFreqA);
AdaptiveLowPassFilter lpfB(orderB, cutOffFreqB);

void lpfInit(){
  lpfA.setOrder(orderA);
  lpfA.setCutOffFreq(cutOffFreqA);

  lpfB.setOrder(orderB);
  lpfB.setCutOffFreq(cutOffFreqB);
}




////////// encoder setup ///////////
#include "encoder_setup.h"

int encA_clkPin = 2, encA_dirPin = 4; // encA_ppr parameter is decleared globally in the global_params_eeprom.h file.
int encB_clkPin = 3, encB_dirPin = 9; // encB_ppr parameter is decleared globally in the global_params_eeprom.h file.

QuadEncoder encA(encA_clkPin, encA_dirPin, encA_ppr);
QuadEncoder encB(encB_clkPin, encB_dirPin, encB_ppr);

void encoderInit() {
  encA.setPulsePerRev(encA_ppr);
  encB.setPulsePerRev(encB_ppr);

  encA.setStopFreqInUs(encA_stopFreq);
  encB.setStopFreqInUs(encB_stopFreq);
  
  attachInterrupt(digitalPinToInterrupt(encA.clkPin), readEncoderA, FALLING);
  attachInterrupt(digitalPinToInterrupt(encB.clkPin), readEncoderB, FALLING);
}

void readEncoderA() {
  encA.freqPerTick = 1000000.00 / (float)(micros() - encA.oldFreqTime);
  encA.oldFreqTime = micros();
  encA.checkFreqTime = micros();

  if (digitalRead(encA.dirPin) > 0) {
    encA.tickCount += 1;
    encA.frequency = encA.freqPerTick / (float)encA.pulsePerRev;
  } else {
    encA.tickCount -= 1;
    encA.frequency = -1.00 * encA.freqPerTick / (float)encA.pulsePerRev;
  }

}

void readEncoderB() {
  encB.freqPerTick = 1000000.00 / (float)(micros() - encB.oldFreqTime);
  encB.oldFreqTime = micros();
  encB.checkFreqTime = micros();

  if (digitalRead(encB.dirPin) > 0) {
    encB.tickCount += 1;
    encB.frequency = encB.freqPerTick / (float)encB.pulsePerRev;
  } else {
    encB.tickCount -= 1;
    encB.frequency = -1.00 * encB.freqPerTick / (float)encB.pulsePerRev;
  }

}
////////////////////////////////////////////////////////////////






////////// motor bridge control ////////////
#include "motor_bridge_control.h"
// motor A H-Bridge Connection
int IN1 = 7, IN2 = 8, enA = 5;
MotorControl motorA(IN1, IN2, enA);

// motor B H-Bridge Connection
int IN3 = 11, IN4 = 12, enB = 6;
MotorControl motorB(IN3, IN4, enB);
/////////////////////////////////////////////







////////// simple pid control ////////////
#include "simple_pid_control.h"

// parameters are decleared globally in the global_params_eeprom.h file.

// motorA pid control
SimplePID pidMotorA(kpA, kiA, kdA, outMin, outMax);

// motorA pid control
SimplePID pidMotorB(kpB, kiB, kdB, outMin, outMax);

void pidInit() {
  pidMotorA.setParameters(kpA, kiA, kdA, outMin, outMax);
  pidMotorB.setParameters(kpB, kiB, kdB, outMin, outMax);
  pidMotorA.begin();
  pidMotorB.begin();
}
/////////////////////////////////////////////










//////////////////////////////////////
#include "EMC2_uno_eeprom_setup.h"
#include "EMC2_serial_i2c_comm_api.h"
//////////////////////////////////////


unsigned long prevSerialAPIComTime, sampleSerialAPIComTime = 10; //ms -> (1000/sampleTime) hz

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);
  
  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////
  delay(500);
  
  Wire.begin(getI2CADDRESS());                
  Wire.onReceive(i2cSlaveReceiveData);
  Wire.onRequest(i2cSlaveSendData);

  initLed();
  offLed();
  delay(500);
  onLed();
  delay(1000);
  offLed();
  
  encoderInit();
  pidInit();
  lpfInit();
  /* motor needs no initialization as it used no global variable dependent on eeprom*/

  
  prevSerialAPIComTime = millis();
}


void loop() {
  ///// do not touch ////////
  ///// useful for velocity reading to check when rotation has stopped
  encA.resetFrequency();
  encB.resetFrequency();

  filteredAngVelA = lpfA.filt(encA.getAngVel());
  filteredAngVelB = lpfB.filt(encB.getAngVel());

  if (pidMode){
    outputA = pidMotorA.compute(targetA, filteredAngVelA); // targetA is among the global params
    outputB = pidMotorB.compute(targetB, filteredAngVelB); // targetB is among the global params

    motorA.sendPWM((int)outputA);
    motorB.sendPWM((int)outputB); 
  }
  ///////////////////////////


////////// using the serial communiaction API ////////////////////////
  if ((millis() - prevSerialAPIComTime) >= sampleSerialAPIComTime) {
    serialReceiveAndSendData();
    prevSerialAPIComTime = millis();
  }
/////////////////////////////////////////////////////////////////////

}
