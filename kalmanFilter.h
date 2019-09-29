#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>
#include "measurement.h"

#define PI 3.1415926


class KalmanFilter{

public:
    KalmanFilter();
    
    float computePitch(Measurement &input);  //using the acclerometer to eatimate the pithch
    float computeRoll(Measurement &input);  //using the acclerometer to eatimate the roll
    float computeYaw(Measurement &input); //uing magnetometer to compute yaw angle
    
    float readYaw() const { return est_yaw-yawOffset;}
    float readPitch() const { return est_pitch-pitchOffset;}
    float readRoll() const { return est_roll-rollOffset;}
    void setYawOffset(float yaw_offset ){ yawOffset = yaw_offset;}
    void setPitchOffset(float pitch_offset) { pitchOffset = pitch_offset;}
    void setRollOffset(float roll_offset) { rollOffset = roll_offset;}

    void processMeasurement( Measurement &input, float dt);

private:
    float roll_acc;
    float pitch_acc;
    float yaw_acc;

    float est_roll ;
    float est_pitch ;
    float est_yaw ;

    float yawOffset;
    float pitchOffset;
    float rollOffset;
    
    float rollBias;
    float pitchBias;

    float** P_roll;
    float** P_pitch;
    float** Q_roll;
    float** Q_pitch;

    float mea_roll_noise = 0.1;
    float mea_pitch_noise = 0.1;

    bool is_initialized = false;

    void predict(float dt);  // using state-space model to predict result
    void update(Measurement &input);   // update the estimated result with the measuremnt
    float computeEstYaw(Measurement &input);
    float** matrixMutil(float** a, float** b);
    float** transpose(float** a);
    float** matrixPlus(float** a, float** b);
    float** matrixMinus(float** a, float** b);
    void freeMemory(float** a, int row);
};


#endif