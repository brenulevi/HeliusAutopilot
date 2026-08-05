#include <iostream>
#include <helius/estimator.h>
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <csignal>

volatile std::sig_atomic_t g_signal_status = 0;

#define MODEL "c172x"
#define INITIAL_CONDITION "reset00.xml"

JSBSim::FGFDMExec* g_fdmexec = nullptr;

void signal_handler(int signal)
{
    g_signal_status = signal;
}

int main()
{
    std::signal(SIGINT, signal_handler);

    std::cout << "Hello World!" << std::endl;
    std::cout << helius_estimator_get_version() << std::endl;
    std::cout << JSBSim::FGFDMExec::GetVersion() << std::endl;

    g_fdmexec = new JSBSim::FGFDMExec();
    g_fdmexec->SetDebugLevel(16);

    g_fdmexec->SetRootDir(SGPath("."));
    g_fdmexec->SetAircraftPath(SGPath("aircraft"));
    g_fdmexec->SetEnginePath(SGPath("engine"));
    g_fdmexec->SetSystemsPath(SGPath("systems"));

    if(!g_fdmexec->LoadModel(MODEL))
    {
        std::cerr <<  "[JSBSimAdapter] Failed to load aircraft model: " << MODEL << std::endl;
        return -1;
    }

    auto ic = g_fdmexec->GetIC();
    if(!ic->Load(SGPath(INITIAL_CONDITION)))
    {
        std::cerr << "[JSBSimAdapter] Failed to load initial condition:" << INITIAL_CONDITION << std::endl;
        return -1;
    }
    
    g_fdmexec->RunIC();

    while(g_signal_status == 0)
    {

    }

    return 0;
}