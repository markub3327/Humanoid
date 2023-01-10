#include <WiFiNINA.h>   // RGB LED

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


void setup() {
  // init LED
  pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);

  // start serial port
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  // init IMU
  imu.begin();

  // Map motors to pins
  motors[0].begin(2);
  motors[1].begin(3);
  motors[2].begin(4);
  motors[3].begin(5);
  motors[4].begin(6);
  motors[5].begin(7);
}

void loop() {

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

        // IMU
        imu.read();
        Serial.print(imu.Ax, 6);
        Serial.print(';');
        Serial.print(imu.Ay, 6);
        Serial.print(';');
        Serial.print(imu.Az, 6);
        Serial.print(';');
        Serial.print(imu.Gx, 6);
        Serial.print(';');
        Serial.print(imu.Gy, 6);
        Serial.print(';');
        Serial.print(imu.Gz, 6);
        Serial.print(';');
        Serial.print(imu.pitch, 6);
        Serial.print(';');
        Serial.print(imu.roll, 6);
        Serial.print(';');
        Serial.print(imu.yaw, 6);
        Serial.print(';');

        // Temperature
        Serial.print(imu.temp, 6);
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

        delay(SERVO_SPEED_TO_TIME(maxAngle));   // servo stabilization delay
    }
  }
}
