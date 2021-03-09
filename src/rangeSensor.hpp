#ifndef RANGE_SENSOR_H_
#define RANGE_SENSOR_H_

#include "sensor.hpp"

class RangeSensor : public Sensor{
private:
    /* data */
    Eigen::VectorXd sensor_data;
    bool data_updated;
    const Eigen::MatrixXd h_matrix;
    const Eigen::MatrixXd noise_matrix;

public:
    RangeSensor(Eigen::MatrixXd& noise_matrix){};
    void calcualteJacobian(Eigen::VectorXd& states){};
    virtual void readFromSensor() override{};
    virtual bool ifSensorUpdated() override{};
    virtual Eigen::VectorXd getMeasurement() override{};
    virtual Eigen::VectorXd getEstimatedTrans(Eigen::VectorXd& states) override{};
    virtual Eigen::MatrixXd getHMatrix(Eigen::VectorXd& states) override{};
    virtual Eigen::MatrixXd getNoiseMatrix() override{};
};

#endif