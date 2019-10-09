#include <kalmanFilter.h>
#include <measurement.h>

#include <MPU9255.h>// include MPU9255 library


#define g 9.81 // 1g ~ 9.81 m/s^2
#define magnetometer_cal 0.06 //magnetometer calibration

bool offset_check = false;
float roll_offset = 0.0;
float pitch_offset = 0.0;
float yaw_offset = 0.0;
KalmanFilter kf;

unsigned long starttime = millis();

MPU9255 mpu;

//process raw acceleration data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : acceleration in m/s^2
float process_acceleration(int input, scales sensor_scale )
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*g;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*g;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*g;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}

//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
float process_angular_velocity(int16_t input, scales sensor_scale )
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  //for +- 250 dps
  if(sensor_scale == scale_250dps)
  {
    return input/131;
  }

  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }

  return 0;
}

//process raw magnetometer data
//input = raw reading from the sensor, sensitivity =
//returns : magnetic flux density in μT (in micro Teslas)
float process_magnetic_flux(int16_t input, double sensitivity)
{
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output around 31 µT without any metal / walls around
  Note : This manetometer has really low initial calibration tolerance : +- 500 LSB !!!
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (input*magnetometer_cal*sensitivity)/0.6;
}

void setup() {
  Serial.begin(115200);// initialize Serial port

  if(mpu.init())
  {
  Serial.println("initialization failed");
  }
  else
  {
  Serial.println("initialization successful!");
  }

  
}

void loop() {
  //take readings
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();

  float ax = process_acceleration(mpu.ax,scale_2g);
  float ay = process_acceleration(mpu.ay,scale_2g);
  float az = process_acceleration(mpu.az,scale_2g);

  float gx = process_angular_velocity(mpu.gx,scale_250dps);
  float gy = process_angular_velocity(mpu.gy,scale_250dps);
  float gz = process_angular_velocity(mpu.gx,scale_250dps);

  float mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
  float my = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
  float mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);

  Measurement input(ax,ay,az,gx,gy,gz,mx,my,mz);

  
  if(!offset_check){
     for(int i = 0; i<100; i++){
      roll_offset += kf.computeRoll(input);
      pitch_offset +=kf.computePitch(input);
      yaw_offset += kf.computeYaw(input);    
     }
     roll_offset /= 100;
     pitch_offset /= 100;
     yaw_offset /= 100;
     kf.setYawOffset(yaw_offset);
     kf.setRollOffset(roll_offset);
     kf.setPitchOffset(pitch_offset);
     offset_check = true;
  }

  unsigned long time_diff = millis()-starttime;
  float dt = (float)time_diff/1000.0;
  starttime = millis();
  kf.processMeasurement(input,dt);
  float yaw = kf.readYaw();
  float pitch = kf.readPitch();
  float roll = kf.readRoll();
  float rollBias = kf.getRollBias();
  float pitchBias = kf.getPitchBias();
  Serial.print("time difference: ");
  Serial.print(dt);
  Serial.print(" roll: ");
  Serial.print(roll);
  Serial.print(" pitch: ");
  Serial.print(pitch);
  Serial.print(" yaw: ");
  Serial.print(yaw);
  Serial.print(" rollBias: ");
  Serial.print(rollBias);
  Serial.print(" pitchBias: ");
  Serial.print(pitchBias);
  Serial.println();

  /*
  Serial.print("AX: ");
  Serial.print(process_acceleration(mpu.ax,scale_2g));

  //Y axis
  Serial.print("  AY: ");
  Serial.print(process_acceleration(mpu.ay,scale_2g));

  //Z axis
  Serial.print("  AY: ");
  Serial.print(process_acceleration(mpu.az,scale_2g));


  ////process and print gyroscope data////
  //X axis
  Serial.print("      GX: ");
  Serial.print(process_angular_velocity(mpu.gx,scale_250dps));

  //Y axis
  Serial.print("  GY: ");
  Serial.print(process_angular_velocity(mpu.gy,scale_250dps));

  //Z axis
  Serial.print("  GZ: ");
  Serial.print(process_angular_velocity(mpu.gz,scale_250dps));


  ////process and print magnetometer data////
  //X axis
  Serial.print("      MX: ");
  Serial.print(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity));

  //Y axis
  Serial.print("  MY: ");
  Serial.print(process_magnetic_flux(mpu.my,mpu.my_sensitivity));

  //Z axis
  Serial.print("  MZ: ");
  Serial.println(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity));
*/
  delay(100);
}
