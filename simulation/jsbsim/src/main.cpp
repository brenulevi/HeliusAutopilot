#include <iostream>
#include <helius/estimator.h>
#include <FGFDMExec.h>

int main()
{
    std::cout << "Hello World!" << std::endl;
    std::cout << helius_estimator_get_version() << std::endl;
    std::cout << JSBSim::FGFDMExec::GetVersion() << std::endl;

    return 0;
}