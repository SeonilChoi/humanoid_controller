#include <thread>
#include <chrono>
#include <iostream>

#include "robot/robots/olaf.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }

    const std::string config_file = argv[1];

    olaf::Olaf olaf(config_file);

    olaf.start();

    for (int i = 0; i < 1000; ++i) {
        const auto observation = olaf.observation();

        std::cout << "Observation: " << observation.size() << std::endl;

        for (size_t i = 0; i < observation.size(); ++i) {
            std::cout << observation[i] << " ";
        }
        std::cout << std::endl;

        olaf.control();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    olaf.stop();

    return 0;
}