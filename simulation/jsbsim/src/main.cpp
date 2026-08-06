#include <iostream>
#include <helius/estimator.h>
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <csignal>
#include <chrono>
#include <iomanip>
#include <thread>

volatile std::sig_atomic_t g_signal_status = 0;

#define MODEL "c172x"
#define INITIAL_CONDITION "reset00.xml"
#define UPDATE_FREQ 1000.0f    // 1khz

#define RAD_TO_DEG 57.295779513082320876798154814105f

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

    g_fdmexec->Setdt(1.0 / UPDATE_FREQ);
    
    g_fdmexec->RunIC();

    std::cout << std::fixed << std::setprecision(4);
    std::cout   << std::setw(6)     << "Time (s)"
                << std::setw(12)    << "Phi (deg)"
                << std::setw(18)    << "Theta (deg)"
                << std::setw(24)    << "Psi (deg)" << std::endl;

    const auto step_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(1 / UPDATE_FREQ));
    auto next_step_time = std::chrono::steady_clock::now();

    unsigned long step_counter = 0;
    while(g_signal_status == 0)
    {
        next_step_time += step_duration;

        if(!g_fdmexec->Run())
        {
            std::cerr << "[JSBSimAdapter] Failed to run simulation step" << std::endl;
            return -1;
        }

        if(step_counter % 1000 == 0)
        {
            const float sim_time = g_fdmexec->GetSimTime();
            const float true_roll_deg = g_fdmexec->GetPropertyValue("attitude/phi-rad") * RAD_TO_DEG;
            const float true_pitch_deg = g_fdmexec->GetPropertyValue("attitude/theta-rad") * RAD_TO_DEG;
            const float true_yaw_deg = g_fdmexec->GetPropertyValue("attitude/psi-rad") * RAD_TO_DEG;

            std::cout   << std::setw(6)     << sim_time
                        << std::setw(10)    << true_roll_deg
                        << std::setw(10)    << true_pitch_deg
                        << std::setw(10)    << true_yaw_deg << std::endl;
        }

        step_counter++;

        std::this_thread::sleep_until(next_step_time);
    }

    return 0;
}