#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>
#include "kinematics/Kinematics.hpp"
#include "drivers/Servo.hpp"



int main(int argc, const char** argv)
{
    using namespace std::chrono_literals;

    // auto sleep_duration = 10ms;
    // bool is_active_next = false;
    // bool is_active_prev = false;

    // while (true) {
    //     // Standing up
    //     if (is_active_next == true && is_active_prev == false) {
    //         if (/* body.z < STANDING_HEIGHT */) {
    //             /* body.z -= 0.001; */
    //         } else {
    //             is_active_prev = true;
    //         }
    //     }
    //     // Standing down
    //     if (is_active_next == false && is_active_prev == true) {
    //         if (/* body.z > 0 */) {
    //             /* body.z += 0.001; */
    //         } else {
    //             is_active_prev = false;
    //         }
    //     }
    //     // Ready
    //     if (is_active_next == true && is_active_prev == true) {
    //     }
    //     std::this_thread::sleep_for(sleep_duration);
    // }

    const double pi = 3.14159265358979;
    const double limit_upper = pi / 2;
    const double limit_lower = -pi / 2;
    const double limit = 127 / ((limit_upper - limit_lower) / 2);
    double angles[18] = {0.0};
    double target[18] = {0.0};

    auto chain = buildChain(0.087, -0.051, -KDL::PI * 1 / 3, 1);
    Solver solver(chain);
    //SerialServoController controller("/dev/ttyACM0");

    double x0 = 0.14208;
    double y0 = -0.14555;
    double z0 = -0.11245;
    double x = x0;
    double y = y0;
    double z = z0;
    double phi = 0;
    int i = 0;
    while (i++ < 100) {
        target[0] = x;
        target[1] = y;
        target[2] = z;
        if (solver.inverse(target, angles)) {
            // std::cout
            //     << "IK solution found: "
            //     << "Cartesian space coordinates: "
            //     << target[0] << ", " << target[1] << ", " << target[2]
            //     << " "
            //     << "Joint space coordinates: "
            //     << 180 / pi * angles[0] << ", " << 180 / pi * angles[1] << ", " << 180 / pi * angles[2]
            //     << std::endl;
            std::cout << static_cast<int>(127 + angles[0] * limit) << "\t"
                      << static_cast<int>(127 + angles[1] * limit) << "\t"
                      << static_cast<int>(127 + angles[2] * limit) << std::endl;
        }
        //controller.set(0, static_cast<uint8_t>(127 + angles[0] * limit));
        //controller.set(1, static_cast<uint8_t>(127 - angles[1] * limit));
        //controller.set(2, static_cast<uint8_t>(127 + angles[2] * limit));

    	x = x0 + 0.05 * cos(phi);
		y = y0 + 0.05 * sin(phi);
        phi += 0.05;
        std::this_thread::sleep_for(0.01s);
    }

    return 0;
}