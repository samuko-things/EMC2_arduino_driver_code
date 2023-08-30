

///////////////////////////////////////////////////////////////////////////////
// store encoder pulsePerRev needed by encoder
int encA_ppr = 0;
int encB_ppr = 0;
unsigned long encA_stopFreq = 5000; // in us
unsigned long encB_stopFreq = 5000; // in us

////
float rdirA = 1.00;
float rdirB = 1.00;

// adaptive lowpass Filter
int orderA = 1;
float cutOffFreqA = 5.0;

int orderB = 1;
float cutOffFreqB = 5.0;

///////////////////////////////////////////////
double outMin = -255.0, outMax = 255.0;

// motorA pid control global params needed by pid
double kpA = 0.0, kiA = 0.0, kdA = 0.0;
double targetA = 0.00, filteredAngVelA;
double outputA;

// motorB pid control global params needed by pid
double kpB = 0.0, kiB = 0.0, kdB = 0.0;
double targetB = 0.00, filteredAngVelB;
double outputB;

// check if in PID or PWM mode
bool pidMode = true; // true-PID MODE, false-SETUP MODE

// initial i2cAddress
int i2cAddress = 0;
