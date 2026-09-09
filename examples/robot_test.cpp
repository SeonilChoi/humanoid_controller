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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    olaf.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (int i = 0; i < 1000; ++i) {
        const auto observation = olaf.observation();

        std::cout << "heading: "
                  << observation[0] << " "
                  << observation[1] << " "
                  << observation[2] << " "
                  << observation[3] << " "
                  << observation[4] << " "
                  << observation[5] << " "
                  << std::endl;

        std::cout << "angular velocity: "
                  << observation[6] << " "
                  << observation[7] << " "
                  << observation[8] << " "
                  << std::endl;

        std::cout << "joint_velocity: "
                  << observation[81] << " "
                  << observation[82] << " "
                  << observation[83] << " "
                  << observation[84] << " "
                  << observation[85] << " "
                  << observation[86] << " "
                  << observation[87] << " "
                  << observation[88] << " "
                  << observation[89] << " "
                  << observation[90] << " "
                  << observation[91] << " "
                  << observation[92]
                  << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
        olaf.control();
    }

    olaf.stop();
    return 0;
}