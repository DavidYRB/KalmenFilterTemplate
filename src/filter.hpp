#ifndef FILTER_H_
#define FILTER_H_
#include <eigen3/Eigen/Dense>

class Filter
{
public:
    virtual void init() = 0;
    virtual void sensors_init() = 0;
    virtual void update() = 0;
    virtual void predict() = 0;
    virtual Eigen::VectorXd getState() = 0;
    virtual Eigen::MatrixXd getCovMatrix() = 0;
    virtual ~Filter(){}
};

#endif