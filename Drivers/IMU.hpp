 // IMU driver
#ifndef IMU_HPP
#define MOTORS_HPP

#include <SensorFusion.h>
#include <Arduino_LSM6DSOX.h>

namespace Drivers {
  class mIMU {
    public:
      float Ax, Ay, Az, Gx, Gy, Gz;
      float pitch, roll, yaw;
      float temp;

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
      }

      void read() {
          if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            IMU.readAcceleration(Ax, Ay, Az);
            IMU.readGyroscope(Gx, Gy, Gz);

            deltat = fusion.deltatUpdate();
            //fusion.MahonyUpdate(Gx * DEG_TO_RAD, Gy * DEG_TO_RAD, Gz * DEG_TO_RAD, Ax, Ay, Az, deltat);
            fusion.MadgwickUpdate(Gz * DEG_TO_RAD, Gx * DEG_TO_RAD, Gy * DEG_TO_RAD, Az, Ax, Ay, deltat);

            pitch = fusion.getPitch();
            roll = fusion.getRoll();
            yaw = fusion.getYaw();
          }

          if (IMU.temperatureAvailable()) {
            IMU.readTemperatureFloat(temp);
          }  
      }

    private:
      SF fusion;
      float deltat;

  };
}

#endif