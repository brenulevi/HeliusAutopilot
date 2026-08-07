#include <iostream>
#include <csignal>
#include <chrono>
#include <iomanip>
#include <cmath>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>

// Header C do seu estimador
extern "C" {
    #include <helius/estimator.h>
}

volatile std::sig_atomic_t g_signal_status = 0;

#define MODEL "c172x"
#define INITIAL_CONDITION "reset00.xml"
#define UPDATE_FREQ 1000.0f    // 1kHz
#define RAD_TO_DEG 57.295779513082320876f

JSBSim::FGFDMExec* g_fdmexec = nullptr;

void signal_handler(int signal) {
    g_signal_status = signal;
}

int main()
{
    std::signal(SIGINT, signal_handler);

    g_fdmexec = new JSBSim::FGFDMExec();
    g_fdmexec->SetDebugLevel(0); // Reduzido para log limpo

    g_fdmexec->SetRootDir(SGPath("."));
    g_fdmexec->SetAircraftPath(SGPath("aircraft"));
    g_fdmexec->SetEnginePath(SGPath("engine"));
    g_fdmexec->SetSystemsPath(SGPath("systems"));

    if (!g_fdmexec->LoadModel(MODEL)) {
        std::cerr << "[JSBSimAdapter] Failed to load aircraft model: " << MODEL << std::endl;
        return -1;
    }

    auto ic = g_fdmexec->GetIC();
    if (!ic->Load(SGPath(INITIAL_CONDITION))) {
        std::cerr << "[JSBSimAdapter] Failed to load initial condition: " << INITIAL_CONDITION << std::endl;
        return -1;
    }

    const float dt = 1.0f / UPDATE_FREQ;
    g_fdmexec->Setdt(dt);
    g_fdmexec->RunIC();

    // 1. Inicializa o Estimador Helius
    helius_estimator_t estimator;
    helius_noise_config_t noise = {
        .gyro_noise_std  = 0.001f,
        .accel_noise_std = 0.01f,
        .gyro_bias_std   = 0.0001f,
        .accel_bias_std  = 0.001f
    };
    helius_estimator_init(&estimator, &noise);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(8)  << "Time(s)"
              << std::setw(12) << "TrueRoll"
              << std::setw(12) << "EstRoll"
              << std::setw(12) << "TruePitch"
              << std::setw(12) << "EstPitch"
              << std::setw(14) << "P_att_var" << std::endl;

    unsigned long step_counter = 0;

    while (g_signal_status == 0)
    {
        if (!g_fdmexec->Run()) {
            std::cerr << "[JSBSimAdapter] Failed to run simulation step" << std::endl;
            return -1;
        }

        // 2. Coleta leituras de sensores simulados do JSBSim
        helius_imu_t imu = {
            .gyro_x_rps   = (float)g_fdmexec->GetPropertyValue("sensor/imu/gyroX_rps"),
            .gyro_y_rps   = (float)g_fdmexec->GetPropertyValue("sensor/imu/gyroY_rps"),
            .gyro_z_rps   = (float)g_fdmexec->GetPropertyValue("sensor/imu/gyroZ_rps"),
            .accel_x_mps2 = (float)g_fdmexec->GetPropertyValue("sensor/imu/accelX_mps2"),
            .accel_y_mps2 = (float)g_fdmexec->GetPropertyValue("sensor/imu/accelY_mps2"),
            .accel_z_mps2 = (float)g_fdmexec->GetPropertyValue("sensor/imu/accelZ_mps2")
        };

        helius_imu_t static_imu = {
            .gyro_x_rps = 0.0f, .gyro_y_rps = 0.0f, .gyro_z_rps = 0.0f,
            .accel_x_mps2 = 0.0f, .accel_y_mps2 = 0.0f, .accel_z_mps2 = -9.80665f
        };

        // 3. Propagação do Estado (Predição do ESKF)
        helius_estimator_predict(&estimator, &imu, dt);

        // 4. (Opcional) Executar Updates de Sensores em frequências menores (ex: GPS @ 10Hz, Baro @ 50Hz)
        /*
        if (step_counter % 100 == 0) { // 10 Hz GPS
            float gps_n = g_fdmexec->GetPropertyValue("position/pn-ft") * 0.3048f;
            float gps_e = g_fdmexec->GetPropertyValue("position/pe-ft") * 0.3048f;
            float gps_d = -g_fdmexec->GetPropertyValue("position/h-sl-ft") * 0.3048f;
            // helius_estimator_update_gps(&estimator, ...);
        }
        */

        // 5. Log e Comparação de Desempenho (a cada 100ms de simulação)
        if (step_counter % 100 == 0)
        {
            const float sim_time = g_fdmexec->GetSimTime();

            // Ground Truth do JSBSim
            const float true_roll  = g_fdmexec->GetPropertyValue("attitude/phi-rad") * RAD_TO_DEG;
            const float true_pitch = g_fdmexec->GetPropertyValue("attitude/theta-rad") * RAD_TO_DEG;

            // Extração da atitude estimada a partir do Quatérnio do Estimador
            // (Assumindo representação q = [w, x, y, z])
            float qw = estimator.attitude_rad.w, qx = estimator.attitude_rad.x, qy = estimator.attitude_rad.y, qz = estimator.attitude_rad.z;
            float est_roll  = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;
            float est_pitch = asinf(2.0f * (qw * qy - qz * qx)) * RAD_TO_DEG;

            std::cout << std::setw(8)  << sim_time
                      << std::setw(12) << true_roll
                      << std::setw(12) << est_roll
                      << std::setw(12) << true_pitch
                      << std::setw(12) << est_pitch
                      << std::setw(14) << estimator.P->p[0][0] // Incerteza de atitude
                      << std::endl;
        }

        step_counter++;
    }

    delete g_fdmexec;
    return 0;
}