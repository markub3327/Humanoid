#include <WiFiNINA.h>   // RGB LED
#include <Arduino_LSM6DSOX.h>

#include "Drivers/Motors.hpp"

#define NUM_OF_LEGS  6
#define NUM_OF_ARMS  0
#define TOTAL_MOTORS (NUM_OF_LEGS + NUM_OF_ARMS)

char state_command = 'S';
char motor_command = 'M';

// Drivers
Drivers::Motors motors[TOTAL_MOTORS];


void setup() {
  // init LED
  pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);

  // start serial port
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  // init IMU
  if (!IMU.begin()) {
	    Serial.println("Failed to initialize IMU!");
      while (1);
  } else {
      Serial.print("Accelerometer sample rate = ");
  	  Serial.print(IMU.accelerationSampleRate());
  	  Serial.println("Hz");
	
  	  Serial.print("Gyroscope sample rate = ");  
  	  Serial.print(IMU.gyroscopeSampleRate());
  	  Serial.println("Hz");

      Serial.println(sizeof(float));
  }

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
          Serial.print(motors[i].read());
          Serial.print(';');
        }

        // IMU
        if (IMU.accelerationAvailable()) {
          float Ax, Ay, Az;
          IMU.readAcceleration(Ax, Ay, Az);
          Serial.print(Ax, 6);
          Serial.print(';');
          Serial.print(Ay, 6);
          Serial.print(';');
          Serial.print(Az, 6);
          Serial.print(';');
        }
        if (IMU.gyroscopeAvailable()) {
          float Gx, Gy, Gz;
      	  IMU.readGyroscope(Gx, Gy, Gz);
          Serial.print(Gx, 6);
          Serial.print(';');
          Serial.print(Gy, 6);
          Serial.print(';');
          Serial.print(Gz, 6);
          Serial.print(';');
        }

        // Temperature
        if (IMU.temperatureAvailable()) {
          float temp;
          IMU.readTemperatureFloat(temp);
          Serial.print(temp, 6);
          //Serial.print(';');
        }

        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, LOW);

        Serial.print('\n');
    }
    else if (receivedData == motor_command)
    {
        int ptr = 0;
        while (Serial.available() > 0 && ptr < TOTAL_MOTORS) {
            auto value = Serial.parseFloat();

            // splitted by semicolon
            if (Serial.read() == ';') {
                motors[ptr++].write(value);
            }
        }
    
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);

        delay(200);   // servo stabilization delay
    }
  }
}
