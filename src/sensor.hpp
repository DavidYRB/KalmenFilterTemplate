#ifndef SENSOR_H_
#define SENSOR_H_

#include <vector>
#include <Eigen/Dense>

class Sensor
{
public:
    virtual void readFromSensor() = 0;
    virtual bool ifSensorUpdated() = 0;
    virtual Eigen::VectorXd getMeasurement() = 0;
    virtual Eigen::VectorXd getEstimatedTrans(Eigen::VectorXd& states) = 0;
    virtual Eigen::MatrixXd getHMatrix(Eigen::VectorXd& states) = 0;
    virtual Eigen::MatrixXd getNoiseMatrix() = 0;
    virtual Eigen::VectorXd measurementToState() = 0;
    virtual ~Sensor(){}
};

#endif