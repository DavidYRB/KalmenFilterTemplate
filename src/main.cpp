#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>
#include "kalmanFilter.hpp"
#include "sensor.hpp"

int main(){
    // create Kalman filter object
    std::shared_ptr<Filter> filter_ptr = std::make_shared<KalmanFilter>();
    // initialization sensors: including sensor parameter setup, reading connection
    filter_ptr->init();
    bool keep_loop{true};
    // The following loop will be running in the right way
    // once the reading data part is determined.
    while(keep_loop){
        filter_ptr->predict();
        filter_ptr->update();
        // to get the state visible to outside
        filter_ptr->getState();
        filter_ptr->getCovMatrix();
    }
    
}