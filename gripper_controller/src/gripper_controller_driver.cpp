#include "gripper_controller/gripper_controller_driver.hpp"

GripperControllerDriver::GripperControllerDriver() {
    this->pos = {0.0, 0.0, 0.0};
    this->ref = {0.0, 0.0, 0.0};
}

void GripperControllerDriver::update() {
    // Update the internal state of the controller if needed
    // This is a placeholder for any additional logic you might want to implement
}

std::vector<double> GripperControllerDriver::compute() {
    std::vector<double> uin(pos.size());
    std::transform(ref.begin(), ref.end(), pos.begin(), uin.begin(), std::minus<>());
    return uin;
}