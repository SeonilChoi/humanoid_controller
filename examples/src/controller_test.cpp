#include <thread>
#include <chrono>
#include <iostream>

#include "controller/amp_controller.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <model file>" << std::endl;
        return 1;
    }

    amp::AmpController controller(argv[1]);
    controller.initialize();

    while (true) {
        std::vector<double> observation(102);
        std::vector<double> action(12);
        controller.update(observation, action);
        std::cout << "Action: " << action[0] << " " << action[1] << " " << action[2] << " " << action[3] << " " << action[4] << " " << action[5] << " " << action[6] << " " << action[7] << " " << action[8] << " " << action[9] << " " << action[10] << " " << action[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    controller.shutdown();

    return 0;
}