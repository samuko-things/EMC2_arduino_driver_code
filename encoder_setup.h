////////////// ENCODERS ////////////////////////////

class QuadEncoder {
public:
  int clkPin, dirPin;
  float pulsePerRev;
  volatile long tickCount;
  float prevAngPos, angPos, angVel, freqPerTick, frequency;
  volatile unsigned long oldFreqTime, checkFreqTime, freqSampleTime=5000;
  long testTickCount = 0;

  QuadEncoder(int clk_pin, int dir_pin, float ppr) {
    clkPin = clk_pin;
    dirPin = dir_pin;
    pulsePerRev = ppr;

    pinMode(clkPin, INPUT_PULLUP);
    pinMode(dirPin, INPUT_PULLUP);

    oldFreqTime = micros();
    checkFreqTime = micros();
  }

  void setPulsePerRev(float ppr){
    pulsePerRev = ppr;
  }

  float getAngPos() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      return (2.00 * PI * (float)tickCount) / pulsePerRev;
    }  
  }

  float getAbsAngPosDeg() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      return absAngDeg((2.00 * PI * (float)tickCount) / pulsePerRev);
    }
  }

  float getAngVel() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      return 2.00 * PI * frequency;
    }
  }

  void setStopFreqInUs(unsigned long freq){
    freqSampleTime=freq;
  }
  
  void resetFrequency() {
    if (micros() - checkFreqTime >= freqSampleTime) {
      frequency = 0;
    }
  }


private:
  float absAngDeg(float incAngRad) {
    float incAngDeg = incAngRad * 180.0 / PI;
    return (int)incAngDeg % 360;
  }
};
