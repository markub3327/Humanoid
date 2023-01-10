#include "api/Common.h"
 // IMU driver
#ifndef IMU_HPP
#define MOTORS_HPP

#define FILTER_UPDATE_RATE_HZ 100

#include <Adafruit_AHRS.h>
#include <Arduino_LSM6DSOX.h>

namespace Drivers {
  struct Vector {
    float x;
    float y;
    float z;
  };

  struct QuaternionVector {
    float w;
    float x;
    float y;
    float z;
  };

  class mIMU {
    public:
      void begin() {
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
          }

          // set lowest range
          //sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
          //sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

          // set slightly above refresh rate
          //sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
          //sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

          filter.begin(FILTER_UPDATE_RATE_HZ);
      }

      void read() {
        // Read the motion sensors
        if (IMU.accelerationAvailable() &&
             IMU.gyroscopeAvailable() &&
              IMU.temperatureAvailable()
        ) {
          IMU.readAcceleration(accelerometer.x, accelerometer.y, accelerometer.z);
          IMU.readGyroscope(gyroscope.x, gyroscope.y, gyroscope.z);
          IMU.readTemperatureFloat(temp);

          // Calibration
          //gyro.gyro.x -= gyro_zerorate[0];
          //gyro.gyro.y -= gyro_zerorate[1];
          //gyro.gyro.z -= gyro_zerorate[2];
          //accel.acceleration.x -= accel_zerog[0];
          //accel.acceleration.y -= accel_zerog[1];
          //accel.acceleration.z -= accel_zerog[2];
        
          // Update the SensorFusion filter
          filter.updateIMU(
            gyroscope.x, 
            gyroscope.z, 
            gyroscope.y, 
            accelerometer.x, 
            accelerometer.z, 
            accelerometer.y
          );
        }
      }

      Vector getAccelerometer() {
        return this->accelerometer;
      }

      Vector getGyroscope() {
        return this->gyroscope;
      }

      float getTemp() {
        return this->temp;
      }

      Vector getEuler() {
          Vector v = {
            .x = filter.getRoll(),
            .y = filter.getPitch(),
            .z = filter.getYaw(),
          };
          return v;        
      }

      QuaternionVector getQuaternion() {
          QuaternionVector q;
          filter.getQuaternion(&q.w, &q.x, &q.y, &q.z);
          return q;
      }
  
    private:
      Vector accelerometer;
      Vector gyroscope;
      float temp;

      // pick your filter! slower == better quality output
      //Adafruit_NXPSensorFusion filter; // slowest
      Adafruit_Madgwick filter;  // faster than NXP
      //Adafruit_Mahony filter;  // fastest/smalleset
  };
}

#endif