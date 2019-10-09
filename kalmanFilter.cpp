#include "kalmanFilter.h"
#include <string.h>

KalmanFilter::KalmanFilter(){
    rollBias = 0.0;
    pitchBias = 0.0;
    yawOffset = 0.0;
    pitchOffset = 0.0;
    rollOffset = 0.0;
    est_roll = 0.0;
    est_pitch = 0.0;
    est_yaw = 0.0;
    roll_acc = 0.0;
    pitch_acc = 0.0;
    yaw_acc = 0.0;
}


float KalmanFilter::computePitch(Measurement &input){
    float ax = input.ax;
    float ay = input.ay;
    float az = input.az;
    float sqrt_root = sqrt(ay*ay+az*az);
    pitch_acc = atan(ax/sqrt_root)*180.0/PI;
    return pitch_acc;
}

float KalmanFilter::computeRoll(Measurement &input){
    float ax = input.ax;
    float ay = input.ay;
    float az = input.az;
    float sqrt_root = sqrt(ax*ax+az*az);
    roll_acc = atan(ay/sqrt_root)*180.0/PI;
    return roll_acc;
}

float KalmanFilter::computeYaw(Measurement &input){
    float mx = input.mx;
    float my = input.my;
    float mz = input.mz;
    float mag_x = mx*cos(pitch_acc/180.0*PI) + my*sin(roll_acc/180.0*PI)*sin(pitch_acc/180.0*PI)
                  + mz*cos(roll_acc/180.0*PI)*sin(pitch_acc/180.0*PI);
    float mag_y = my*cos(roll_acc/180.0*PI)-mz*sin(roll_acc/180*PI);

    yaw_acc = atan(-1.0*mag_y/mag_x)*180.0/PI;

    return yaw_acc;
}

float KalmanFilter::computeEstYaw(Measurement &input){
    float mx = input.mx;
    float my = input.my;
    float mz = input.mz;
    float mag_x = mx*cos(est_pitch/180.0*PI) + my*sin(est_roll/180.0*PI)*sin(est_pitch/180.0*PI)
                  + mz*cos(est_roll/180.0*PI)*sin(est_pitch/180.0*PI);
    float mag_y = my*cos(est_roll/180.0*PI)-mz*sin(est_roll/180*PI);

    est_yaw = atan(-1.0*mag_y/mag_x)*180.0/PI;

    return yaw_acc;
}

void KalmanFilter::processMeasurement(Measurement &input, float dt){
    if(!is_initialized){
        computeRoll(input);
        computePitch(input);
        est_roll = roll_acc;
        est_pitch = pitch_acc;
        est_yaw = computeEstYaw(input);
        is_initialized = true;
        return;
    }
    predict(dt);
    update(input);
}

void KalmanFilter::matrixMutil(float a[2][2], float b[2][2], float res[2][2]){
    float sum = 0.0;
    for(int i = 0; i<2; i++){
        for(int j = 0; j<2; j++){
            sum = 0.0;
            for(int k = 0; k<2; k++){
                sum += a[i][k] *b[k][j];
            }
            res[i][j] = sum;
        }
    }
}

void KalmanFilter::transpose(float a[2][2], float res[2][2]){
    for(int i = 0; i<2; i++){
        for(int j = 0; j<2; j++){
            res[i][j] = a[j][i];
        }
    }
}

void KalmanFilter::matrixPlus(float a[2][2], float b[2][2], float res[2][2]){
    for(int i = 0; i<2; i++){
        for(int j = 0; j<2; j++){
            res[i][j] = 0.0;
            res[i][j] += (a[i][j] + b[i][j]);
        }
    }
}

void KalmanFilter::matrixMinus(float a[2][2], float b[2][2], float res[2][2]){
    for(int i = 0; i<2; i++){
        for(int j = 0; j<2; j++){
            res[i][j] = 0.0;
            res[i][j] += (a[i][j] - b[i][j]);
        }
    }
}


void KalmanFilter::predict(float dt){
    est_roll = est_roll + dt*rollBias;
    rollBias = rollBias;
    est_pitch = est_pitch + dt*pitchBias;
    pitchBias = pitchBias;

    float F[2][2];
    F[0][0] = 1.0;
    F[0][1] = dt;
    F[1][0] = 1.0;
    F[1][1] = 0.0;
    float temp_roll[2][2];
    float transposeF[2][2];
    float new_temp_roll[2][2];
    matrixMutil(F,P_roll,temp_roll);
    transpose(F,transposeF);
    matrixMutil(temp_roll,transposeF,new_temp_roll);
    matrixPlus(new_temp_roll,Q_roll,P_roll);

    float temp_pitch[2][2];
    float new_temp_pitch[2][2];

    matrixMutil(F,P_pitch,temp_pitch);
    matrixMutil(temp_pitch,transposeF,new_temp_pitch);
    matrixPlus(new_temp_pitch,Q_pitch,P_pitch);

}

void KalmanFilter::update(Measurement &input){
    computePitch(input);
    computeRoll(input);
    float err_roll = roll_acc - est_roll;
    float err_pitch = pitch_acc - est_pitch;
    float s_roll = P_roll[0][0] + mea_roll_noise;
    float s_pitch = P_pitch[0][0] + mea_pitch_noise;

    float k_roll[2][2];
    k_roll[0][0] = P_roll[0][0]/s_roll;
    k_roll[1][0] = P_roll[1][0]/s_roll;

    float k_pitch[2][2];
    k_pitch[0][0] = P_pitch[0][0]/s_pitch;
    k_pitch[1][0] = P_pitch[1][0]/s_pitch;

    est_roll = est_roll + k_roll[0][0]*err_roll;
    rollBias = rollBias + k_roll[1][0]*err_roll;

    est_pitch = est_pitch + k_pitch[0][0]*err_pitch;
    pitchBias = pitchBias + k_pitch[1][0]*err_pitch;
    
    float identity[2][2] = {{1.0,0.0},{0.0,1.0}};

    float temp_roll[2][2];
    temp_roll[0][0] = k_roll[0][0];
    temp_roll[1][0] = k_roll[1][0];
    temp_roll[0][1] = 0.0;
    temp_roll[1][1] = 0.0;

    float new_temp_roll[2][2];
    matrixMinus(identity,temp_roll,new_temp_roll);
    float res[2][2];
    matrixMutil(new_temp_roll,P_roll,res);
    for(int i = 0; i<2; i++){
        memcpy(&P_roll[i],&res[i], sizeof(res[i]));
    }
    
    float temp_pitch[2][2];
    temp_pitch[0][0] = k_pitch[0][0];
    temp_pitch[1][0] = k_pitch[1][0];
    temp_pitch[0][1] = 0.0;
    temp_pitch[1][1] = 0.0;

    float new_temp_pitch[2][2];
    matrixMinus(identity,temp_pitch,new_temp_pitch);
    matrixMutil(new_temp_pitch,P_pitch,res);
    for(int i = 0; i<2; i++){
        memcpy(&P_pitch[i],&res[i], sizeof(res[i]));
    }
    computeEstYaw(input);
  
    
}

void KalmanFilter::freeMemory(float** a, int row){
    for(int i = 0; i<row; i++){
        delete[] a[i];
    }
    delete[] a;
}