#include <cmath>

#include "constants.hpp"
#include "rangeSensor.hpp"

RangeSensor::RangeSensor(Eigen::MatrixXd& noise_matrix):
    noise_matrix(noise_matrix){
        sensor_data.resize(3);
        h_matrix << 0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0;
    }


void RangeSensor::readFromSensor(){
    // a running process keep get data from sensor
    // This function will be implemented once the source of data is determined
    // It will update the sensor_data, and if the sensor_data is updated, 
    // boolean member data_updated will be set to true which means the data aquired in this object 
    // is lates and usable. 
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

Eigen::VectorXd RangeSensor::measurementToState(){
    Eigen::VectorXd state_foramt(4);
    double theta = sensor_data(1);
    double range = sensor_data(0);
    double range_rate = sensor_data(2);
    state_foramt(0) = range * cos(theta);
    state_foramt(1) = range * sin(theta);
    state_foramt(2) = range_rate * cos(theta);
    state_foramt(3) = range_rate * sin(theta);

    return state_foramt;
}

void RangeSensor::calcualteJacobian(Eigen::VectorXd& states){

    double px = states(0);
    double py = states(1);
    double vx = states(2);
    double vy = states(3);

    double c1 = pow(px, 2)+pow(py,2);
    double c2 = sqrt(c1);
    double c3 = (c1*c2);

    if(fabs(c1) < 0.0001)
        return;

    h_matrix << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
}
