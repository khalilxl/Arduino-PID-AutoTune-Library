/*
bundle config --local path vendor/bundle
bundle install
bundle exec arduino_ci_remote.rb  --skip-compilation
 */
#include "Arduino.h"
#include "ArduinoUnitTests.h"
#include "PID_AutoTune_v0.h"
#include "PID_v1.cpp"

byte ATuneModeRemember = 2;
double input = 80, output = 50, setpoint = 180;
double kp = 2, ki = 0.5, kd = 2;

double kpmodel = 1.5, taup = 100, theta[50];
double outputStart = 5;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

boolean tuning = false;
unsigned long modelTime;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);

void setup();
void loop();
void changeAutoTune();
void AutoTuneHelper(boolean start);
void report();
void DoModel();

void setup() {
  //   set up simulation data
  for (byte i = 0; i < 50; i++) {
    theta[i] = outputStart;
  }
  modelTime = 0;

  // Setup the pid
  myPID.SetMode(AUTOMATIC);

  // Set the output to the desired starting frequency.
  output = aTuneStartValue;
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  AutoTuneHelper(true);
  tuning = true;
}

void loop() {

  unsigned long now = millis();

  if (tuning) {
    byte val = (aTune.Runtime());
    if (val != 0) {
      tuning = false;
    }
    if (!tuning) { // we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
    }
  } else
    myPID.Compute();

  theta[30] = output;
  if (now >= modelTime) {
    modelTime += 100;
    DoModel();
  }
}

void changeAutoTune() {
  if (!tuning) {
    // Set the output to the desired starting frequency.
    output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  } else { // cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start) {
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void report() {
  // Replace with couts statements?
  std::cout << "setpoint: " << setpoint << std::endl;
  //   Serial.print("setpoint: ");
  //   Serial.print(setpoint);
  //   Serial.print(" ");
  //   Serial.print("input: ");
  //   Serial.print(input);
  //   Serial.print(" ");
  //   Serial.print("output: ");
  //   Serial.print(output);
  //   Serial.print(" ");
  //   if (tuning) {
  //     Serial.println("tuning mode");
  //   } else {
  //     Serial.print("kp: ");
  //     Serial.print(myPID.GetKp());
  //     Serial.print(" ");
  //     Serial.print("ki: ");
  //     Serial.print(myPID.GetKi());
  //     Serial.print(" ");
  //     Serial.print("kd: ");
  //     Serial.print(myPID.GetKd());
  //     Serial.println();
}

void DoModel() {
  // cycle the dead time
  for (byte i = 0; i < 49; i++) {
    theta[i] = theta[i + 1];
  }
  // compute the input
  input = (kpmodel / taup) * (theta[0] - outputStart) + input * (1 - 1 / taup) +
          ((float)random(-10, 10)) / 100;
}

unittest(test) {
  assertTrue(true);
  setup();
  for (byte i = 1; i < 100; i++) {
    delay(100);
    loop();
    report();
  }
}

unittest_main()
