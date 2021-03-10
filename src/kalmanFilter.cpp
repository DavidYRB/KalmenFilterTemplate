#include "kalmanFilter.hpp"
#include "constants.hpp"
#include "rangeSensor.hpp"

KalmanFilter::KalmanFilter(){
        states_len = 4;
        states.resize(states_len);
        state_inited = false;
        process_noise_matrix = Eigen::MatrixXd::Zero(4, 4);
        transition_matrix << 1, 0, 1, 0,
                        0, 1, 0, 1,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
    }


void KalmanFilter::sensors_init() {
    // use constant.hpp to create sensor
    Eigen::MatrixXd noise_matrix(3,3);
    noise_matrix << RANGE_SENSOR_NOISE, 0, 0,
                    0, RANGE_SENSOR_NOISE, 0,
                    0, 0, RANGE_SENSOR_NOISE;
    std::shared_ptr<Sensor> range_sensor = std::make_shared<RangeSensor>(noise_matrix);
    sensors.push_back(range_sensor);
}

// Do initialization
void KalmanFilter::init() {
    sensors_init();
    for(int i = 0; i < sensors.size(); ++i){
        if(!state_inited && sensors[i]->ifSensorUpdated()){
            states = sensors[i]->measurementToState();
            prev_time = std::chrono::steady_clock::now();
            state_inited = true;
            break;
        }
    }
}


void KalmanFilter::updateProcessCovMatrix(double del_t){
    double delta_sqr = del_t * del_t;
    double delta_cub = delta_sqr * del_t;
    double delta_qua = delta_cub * del_t;
    process_noise_matrix(0, 0) = delta_qua / 4 * ACCE_X_NOISE;
    process_noise_matrix(0, 1) = delta_cub / 2 * ACCE_X_NOISE;
    process_noise_matrix(1, 1) = delta_qua / 4 * ACCE_Y_NOISE;
    process_noise_matrix(1, 3) = delta_cub / 2 * ACCE_Y_NOISE;
    process_noise_matrix(2, 0) = delta_cub / 2 * ACCE_X_NOISE;
    process_noise_matrix(2, 3) = delta_sqr * ACCE_X_NOISE;
    process_noise_matrix(3, 1) = delta_cub / 2 * ACCE_Y_NOISE;
    process_noise_matrix(3, 3) = delta_sqr * ACCE_Y_NOISE;
}

void KalmanFilter::predict(){
    auto curr_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = curr_time - prev_time;
    double delta_t = elapsed_seconds.count();
    updateProcessCovMatrix(delta_t);
    transition_matrix(0, 2) = delta_t;
    transition_matrix(1, 3) = delta_t;
    states = transition_matrix * states;
    cov_matrix = transition_matrix * cov_matrix * transition_matrix.transpose() + process_noise_matrix;
}

void KalmanFilter::update(){
    prev_time = std::chrono::steady_clock::now();
    int sensor_num = sensors.size();
    for(int i = 0; i < sensor_num; ++i){
        std::shared_ptr<Sensor> curr_sensor = sensors[i];
        if(curr_sensor->ifSensorUpdated()){
            Eigen::VectorXd measurement = curr_sensor->getMeasurement();
            Eigen::VectorXd est_measure = curr_sensor->getEstimatedTrans(states);
            Eigen::MatrixXd h_matrix = curr_sensor->getHMatrix(states);
            Eigen::MatrixXd noise_matrix = curr_sensor->getNoiseMatrix();
            
            Eigen::VectorXd y = measurement - est_measure;

            Eigen::MatrixXd s = h_matrix * cov_matrix * h_matrix.transpose() + noise_matrix;
            Eigen::MatrixXd k_gain = cov_matrix * h_matrix.transpose() * s.inverse();

            states = states + k_gain * y;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(states_len, states_len);
            cov_matrix = (I - k_gain * h_matrix) * cov_matrix;
        }
    }
}

Eigen::VectorXd KalmanFilter::getState(){
    return states;
}

Eigen::MatrixXd KalmanFilter::getCovMatrix(){
    return cov_matrix;
}