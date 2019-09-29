#include "kalmanFilter.h"

KalmanFilter::KalmanFilter(){
    P_roll = new float*[2];
    P_roll[0] = new float[2];
    P_roll[1] = new float[2];

    P_pitch = new float*[2];
    P_pitch[0] = new float[2];
    P_pitch[1] = new float[2];

    Q_roll = new float*[2];
    Q_roll[0] = new float[2];
    Q_roll[1] = new float[2];

    Q_pitch = new float*[2];
    Q_pitch[0] = new float[2];
    Q_pitch[1] = new float[2];

    P_roll[0][0] = 0.0;
    P_roll[0][1] = 0.0;
    P_roll[1][0] = 0.0;
    P_roll[1][1] = 0.0;
    P_pitch[0][0] = 0.0;
    P_pitch[0][1] = 0.0;
    P_pitch[1][0] = 0.0;
    P_pitch[1][1] = 0.0;
    Q_roll[0][0] = 0.1;
    Q_roll[0][1] = 0.0;
    Q_roll[1][0] = 0.0;
    Q_roll[1][1] = 0.1;
    Q_pitch[0][0] = 0.1;
    Q_pitch[0][1] = 0.0;
    Q_pitch[1][0] = 0.0;
    Q_pitch[1][1] = 0.1;
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

float** KalmanFilter::matrixMutil(float** a, float** b){
    float** res = new float*[2];
    float sum = 0.0;
    for(int i = 0; i<2; i++){
        res[i] = new float[2];
        for(int j = 0; j<2; j++){
            sum = 0.0;
            for(int k = 0; k<2; k++){
                sum += a[i][k] *b[k][j];
            }
            res[i][j] = sum;
        }
    }
    return res;
}

float** KalmanFilter::transpose(float** a){
    float** res = new float*[2];
    for(int i = 0; i<2; i++){
        res[i] = new float[2];
        for(int j = 0; j<2; j++){
            res[i][j] = a[j][i];
        }
    }
    return res;
}

float** KalmanFilter::matrixPlus(float** a, float** b){
    float** res = new float*[2];
    for(int i = 0; i<2; i++){
        res[i] = new float[2];
        for(int j = 0; j<2; j++){
            res[i][j] = 0.0;
            res[i][j] += (a[i][j] + b[i][j]);
        }
    }
    return res;
}

float** KalmanFilter::matrixMinus(float** a, float** b){
    float** res = new float*[2];
    for(int i = 0; i<2; i++){
        res[i] = new float[2];
        for(int j = 0; j<2; j++){
            res[i][j] = 0.0;
            res[i][j] += (a[i][j] - b[i][j]);
        }
    }
    return res;
}


void KalmanFilter::predict(float dt){
    est_roll = est_roll + dt*rollBias;
    rollBias = rollBias;
    est_pitch = est_pitch + dt*pitchBias;
    pitchBias = pitchBias;

    float** F = new float*[2];
    F[0] = new float[2];
    F[1] = new float[2];

    F[0][0] = 1.0;
    F[0][1] = dt;
    F[1][0] = 1.0;
    F[1][1] = 0.0;

    float** temp_roll = matrixMutil(F,P_roll);
    float** transposeF = transpose(F);
    temp_roll = matrixMutil(temp_roll,transposeF);
    P_roll = matrixPlus(temp_roll,Q_roll);

    float** temp_pitch = matrixMutil(F,P_pitch);
    temp_pitch = matrixMutil(temp_pitch,transposeF);
    P_pitch = matrixPlus(temp_pitch,Q_pitch);

    freeMemory(temp_roll,2);
    freeMemory(transposeF,2);
    freeMemory(temp_pitch,2);
    freeMemory(F,2);
}

void KalmanFilter::update(Measurement &input){
    computePitch(input);
    computeRoll(input);
    float err_roll = roll_acc - est_roll;
    float err_pitch = pitch_acc - est_pitch;
    float s_roll = P_roll[0][0] + mea_roll_noise;
    float s_pitch = P_pitch[0][0] + mea_pitch_noise;

    float** k_roll  = new float*[2];
    k_roll[0] = new float[1];
    k_roll[1] = new float[1];

    k_roll[0][0] = P_roll[0][0]/s_roll;
    k_roll[1][0] = P_roll[1][0]/s_roll;

    float** k_pitch  = new float*[2];
    k_pitch[0] = new float[1];
    k_pitch[1] = new float[1];
    k_pitch[0][0] = P_pitch[0][0]/s_pitch;
    k_pitch[1][0] = P_pitch[1][0]/s_pitch;

    est_roll = est_roll + k_roll[0][0]*err_roll;
    rollBias = rollBias + k_roll[1][0]*err_roll;

    est_pitch = est_pitch + k_pitch[0][0]*err_pitch;
    pitchBias = pitchBias + k_pitch[1][0]*err_pitch;
    
    float** identity = new float*[2];
    identity[0] = new float[2];
    identity[1] = new float[2];
    identity[0][0] = 1.0;
    identity[0][1] = 0.0;
    identity[1][0] = 0.0;
    identity[1][1] = 1.0;

    float** temp_roll = new float*[2];
    temp_roll[0] = new float[2];
    temp_roll[1] = new float[2];
    temp_roll[0][0] = k_roll[0][0];
    temp_roll[1][0] = k_roll[1][0];
    temp_roll[0][1] = 0.0;
    temp_roll[1][1] = 0.0;

    temp_roll = matrixMinus(identity,temp_roll);
    P_roll = matrixMutil(temp_roll,P_roll);
    
    float** temp_pitch = new float*[2];
    temp_pitch[0] = new float[2];
    temp_pitch[1] = new float[2];
    temp_pitch[0][0] = k_pitch[0][0];
    temp_pitch[1][0] = k_pitch[1][0];
    temp_pitch[0][1] = 0.0;
    temp_pitch[1][1] = 0.0;


    temp_pitch = matrixMinus(identity,temp_pitch);
    P_pitch = matrixMutil(temp_pitch,P_pitch);

    computeEstYaw(input);
    freeMemory(identity,2);
    freeMemory(temp_pitch,2);
    freeMemory(temp_roll,2);
    freeMemory(k_pitch,2);
    freeMemory(k_roll,2);
    
}

void KalmanFilter::freeMemory(float** a, int row){
    for(int i = 0; i<row; i++){
        delete[] a[i];
    }
    delete[] a;
}