#ifndef EKF_TEST_UTILITIES_H
#define EKF_TEST_UTILITIES_H

#include "ekf_test/Input.h"
#include <Eigen/Dense>

class Utilities {
 private:
    /* data */
 public:
    Utilities(/* args */);
    ~Utilities();
    explicit predictMotion(const ekf_test::Input::ConstPtr& msg) {
       
    }
};

Utilities::Utilities(/* args */) {

}

Utilities::~Utilities() {

}


#endif
