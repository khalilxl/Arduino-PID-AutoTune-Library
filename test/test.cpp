/*
bundle config --local path vendor/bundle
bundle install
bundle exec arduino_ci_remote.rb  --skip-compilation
 */
#include "Arduino.h"
#include "ArduinoUnitTests.h"
#include "PID_AutoTune_v0.h"

unittest(test) { assertTrue(true); }

unittest_main()
