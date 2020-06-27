#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <thread>
#include <chrono>
#include "examples_common.h"

#include <Eigen/Core>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    try
    {
        franka::Robot robot(argv[1]);
        setDefaultBehavior(robot);
        size_t count = 0;
        robot.read([&count](const franka::RobotState &robot_state) {
            // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
            // should not be done in a control loop.
            std::cout << "force/torques: ";
            printArray(robot_state.K_F_ext_hat_K.data(), robot_state.K_F_ext_hat_K.size());
            Eigen::Matrix<double,3,1> force_vec(robot_state.K_F_ext_hat_K.data());
            std::cout << "force norm: " << force_vec.norm()<<std::endl;
            std::cout << "joint speeds: ";
            printArray(robot_state.dq.data(), robot_state.dq.size());
            Eigen::Matrix<double,7,1> q_dot(robot_state.dq.data());
            std::cout << "joint speed norm: " << q_dot.norm()<<std::endl;
            std::this_thread::sleep_for(500ms);
            return count++ < 100;
        });
        std::cout << "Done." << std::endl;
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}
