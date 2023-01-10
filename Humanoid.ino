#include <WiFiNINA.h>   // RGB LED
#include "pico/multicore.h"

#include "Drivers/Motors.hpp"
#include "Drivers/IMU.hpp"

#define NUM_OF_LEGS  6
#define NUM_OF_ARMS  0
#define TOTAL_MOTORS (NUM_OF_LEGS + NUM_OF_ARMS)

#define SERVO_SPEED_TO_TIME(angle)  (170 * angle / 60.0f)
char state_command = 'S';
char motor_command = 'M';

// Drivers
Drivers::Motors motors[TOTAL_MOTORS];
Drivers::mIMU imu;

//void core1_entry() {
//}

void setup() {
  // init LED
  pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);

  //multicore_launch_core1(core1_entry);

  // start serial port
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // init IMU
  imu.begin();

  // Map motors to pins
  motors[0].begin(2);
  motors[1].begin(3);
  motors[2].begin(4);
  motors[3].begin(5);
  motors[4].begin(6);
  motors[5].begin(7);

  // I'm ready
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDR, LOW);
}

void loop() {
  // IMU
  imu.read();

  if (Serial.available()) {
    char receivedData = Serial.read();

    if (receivedData == state_command)
    {
        // joint angle
        for (uint8_t i = 0; i < TOTAL_MOTORS; i++)
        {
          Serial.print(motors[i].read(), 6);
          Serial.print(';');
        }

        auto a = imu.getAccelerometer();
        Serial.print(a.x, 6);
        Serial.print(';');
        Serial.print(a.y, 6);
        Serial.print(';');
        Serial.print(a.z, 6);
        Serial.print(';');

        auto g = imu.getGyroscope();
        Serial.print(g.x, 6);
        Serial.print(';');
        Serial.print(g.y, 6);
        Serial.print(';');
        Serial.print(g.z, 6);
        Serial.print(';');

        auto v = imu.getEuler();
        Serial.print(v.x, 6);
        Serial.print(';');
        Serial.print(v.y, 6);
        Serial.print(';');
        Serial.print(v.z, 6);
        Serial.print(';');

        // Temperature
        Serial.print(imu.getTemp(), 6);
        Serial.print('\n');

        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, LOW);
    }
    else if (receivedData == motor_command)
    {
        int ptr = 0;
        float deltaAngle = 0.0, maxAngle = 0.0;
        while (Serial.available() > 0 && ptr < TOTAL_MOTORS) {
            auto value = Serial.parseFloat();

            // splitted by semicolon
            if (Serial.read() == ';') {
                // find maximal delta
                deltaAngle = abs(motors[ptr].read() - value);
                if (deltaAngle > maxAngle) 
                    maxAngle = deltaAngle;
                motors[ptr++].write(value);
            }
        }

        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);

        //if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
        //  return;
        //}
        //timestamp = millis();
        //delay(SERVO_SPEED_TO_TIME(maxAngle));   // servo stabilization
    }
  }
}
