#ifndef MEASUREMENT_H
#define MEASUREMENT_H
struct Measurement{
    float ax = 0.0; //acc_x
    float ay = 0.0; //acc_y
    float az = 0.0; //acc_z

    float gx = 0.0; //gyro_x
    float gy = 0.0; //gyro_y
    float gz = 0.0; //gyro_z

    float mx = 0.0; //mag_x
    float my = 0.0; //mag_y
    float mz = 0.0; //mag_z

    Measurement(float ax, float ay, float az, float gx, float gy, float gz, float mx,float my, float mz):
    ax(ax),ay(ay),az(az),gx(gx),gy(gy),gz(gz),mx(mx),my(my),mz(mz){}
};

#endif