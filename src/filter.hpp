#ifndef FILTER_H_
#define FILTER_H_
#include <Eigen/Dense>

class Filter
{
public:
    virtual void init() = 0;
    virtual void sensors_init() = 0;
    virtual void update() = 0;
    virtual void predict() = 0;
    virtual ~Filter(){}
};

#endif