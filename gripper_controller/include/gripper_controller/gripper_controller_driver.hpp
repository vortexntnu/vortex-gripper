#ifndef GRIPPER_CONTROLLER_DRIVER_HPP
#define GRIPPER_CONTROLLER_DRIVER_HPP

#include <iostream>
#include <unistd.h>
#include <vector>
#include <ranges>
#include <algorithm>
#include <functional>

class GripperControllerDriver {
public:
    GripperControllerDriver();
    
    void update();
    std::vector<double> compute();

    std::vector<double> pos;
    std::vector<double> ref;

private:
    
};


#endif // GRIPPER_CONTROLLER_DRIVER_HPP