#include "arduino_motors.h"
#include "gflags.h"

DEFINE_string(arduino_port, "/dev/ttyO4", "Serial port for Arduino");

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);

  sailbot::ArduinoMotors arduino(FLAGS_arduino_port.c_str());
  arduino.Run();
}
