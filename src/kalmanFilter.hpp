#include <chrono>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "filter.hpp"
#include "sensor.hpp"

class KalmanFilter : public Filter{
private:
    /* data */
    std::vector< std::shared_ptr<Sensor> > sensors;
    Eigen::VectorXd states;
    long states_len;
    Eigen::MatrixXd cov_matrix;
    bool state_inited;
    std::chrono::time_point<std::chrono::steady_clock> prev_time;

    Eigen::MatrixXd transition_matrix;
    Eigen::MatrixXd process_noise_matrix;

public:
    KalmanFilter(Eigen::MatrixXd& trans_matrix, Eigen::MatrixXd& noise_matrix){};
    void updateProcessCovMatrix(double del_t){};
    virtual void init() override{};
    virtual void sensors_init() override{};
    virtual void update() override{};
    virtual void predict() override{};
};