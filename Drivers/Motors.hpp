 // Motors driver
#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Servo.h>

namespace Drivers {
  class Motors : private Servo {
    public:

      void begin(int pin) {
          this->attach(pin);
          this->write(0.0f);    // init position
      }

      void write(float value) {
          // map to [0, 1]
          value = 0.5f * (value + 1.0f);

          // map to [min, max]
          value *= static_cast<float>(MAX_PULSE_WIDTH) - static_cast<float>(MIN_PULSE_WIDTH);
          value += static_cast<float>(MIN_PULSE_WIDTH);

          this->writeMicroseconds(static_cast<int>(value));
      }

      float read() {
          float value = static_cast<float>(this->readMicroseconds());

          // // map to [0, 1]
          value -= static_cast<float>(MIN_PULSE_WIDTH);
          value /= static_cast<float>(MAX_PULSE_WIDTH) - static_cast<float>(MIN_PULSE_WIDTH);

          // // map to [-1, 1]
          value *= 2.0f;
          value -= 1.0f;

          return value;
      }
  };
}

#endif