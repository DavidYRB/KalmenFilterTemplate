#include <cmath>

#include "constants.hpp"
#include "rangeSensor.hpp"

RangeSensor::RangeSensor(Eigen::MatrixXd& noise_matrix):
    noise_matrix(noise_matrix){}


void RangeSensor::readFromSensor(){
    // a running process keep get data from sensor

}

bool RangeSensor::ifSensorUpdated(){
    return data_updated;
}

Eigen::VectorXd RangeSensor::getMeasurement(){
    return sensor_data;
}
Eigen::MatrixXd RangeSensor::getHMatrix(Eigen::VectorXd& states){
    calcualteJacobian(states);
    return h_matrix;
}

Eigen::MatrixXd RangeSensor::getNoiseMatrix(){
    return noise_matrix;
}

Eigen::VectorXd RangeSensor::getEstimatedTrans(Eigen::VectorXd& states){
    Eigen::VectorXd z_trans(3);
    double px = states[0];
    double py = states[1];
    double vx = states[2];
    double vy = states[3];

    z_trans(0) = sqrt(px * px + py * py);
    if(z_trans(0) < THRESHOLD){
        z_trans(0) = THRESHOLD;
        z_trans(2) = 0;
    }
    else{
        z_trans(2) = (px * vx + py * vy)/z_trans(0);
    }
    z_trans(1) = atan2(py, px);

    return z_trans;
}

