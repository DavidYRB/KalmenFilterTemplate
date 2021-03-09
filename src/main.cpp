#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "kalmanFilter.hpp"
#include "sensor.hpp"

int main(){
    // create Kalman filter object
    std::shared_ptr<Filter> filter_ptr = std::make_shared<KalmanFilter>();
    // initialization sensors: including sensor parameter setup, reading connection
    Eigen::VectorXd init_state;
    Eigen::MatrixXd init_cov_matrix;
    filter_ptr->init();
    bool keep_loop{true};
    // 
    while(keep_loop){
        filter_ptr->predict();
        filter_ptr->update();
    }
    
}