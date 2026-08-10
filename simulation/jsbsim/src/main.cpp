#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>

extern "C"
{
#include <helius/estimator.h>
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static float wrap_deg_0_360(float angle_deg)
{
    float wrapped = std::fmod(angle_deg, 360.0f);
    if (wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped;
}

static float wrap_deg_pm_180(float angle_deg)
{
    float wrapped = wrap_deg_0_360(angle_deg);
    if (wrapped > 180.0f)
    {
        wrapped -= 360.0f;
    }
    return wrapped;
}

static void quat_to_euler_deg(const quat_t& q, float& roll_deg, float& pitch_deg, float& yaw_deg)
{
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float roll = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    const float pitch = std::asin(clampf(sinp, -1.0f, 1.0f));

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float yaw = std::atan2(siny_cosp, cosy_cosp);

    constexpr float kRadToDeg = 57.29577951308232f;
    roll_deg = roll * kRadToDeg;
    pitch_deg = pitch * kRadToDeg;
    yaw_deg = yaw * kRadToDeg;
}

static quat_t quat_from_euler_rad(float roll, float pitch, float yaw)
{
    const float cr = std::cos(0.5f * roll);
    const float sr = std::sin(0.5f * roll);
    const float cp = std::cos(0.5f * pitch);
    const float sp = std::sin(0.5f * pitch);
    const float cy = std::cos(0.5f * yaw);
    const float sy = std::sin(0.5f * yaw);

    quat_t q = {
        .w = cr * cp * cy + sr * sp * sy,
        .x = sr * cp * cy - cr * sp * sy,
        .y = cr * sp * cy + sr * cp * sy,
        .z = cr * cp * sy - sr * sp * cy,
    };
    return quat_normalize(q);
}

struct heading_test_case_t
{
    const char* name;
    float estimator_heading_offset_deg;
};

struct heading_test_result_t
{
    std::string name;
    std::string csv_file;
    bool converged;
    float final_yaw_error_deg;
    float mean_abs_yaw_error_last_window_deg;
    float max_abs_yaw_error_last_window_deg;
};

static heading_test_result_t run_static_heading_test_case(const heading_test_case_t& test_case)
{
    constexpr float kDt = 0.001f;          // 1 kHz
    constexpr float kDurationSec = 8.0f;  // 8 seconds per heading case
    constexpr int kSteps = static_cast<int>(kDurationSec / kDt);
    constexpr int kMagRefInitStartSample = 2000;
    constexpr int kMagRefInitSamples = 500;
    constexpr float kRadToDeg = 57.29577951308232f;
    constexpr float kDegToRad = 0.017453292519943295f;
    constexpr int kConvergenceWindowSamples = 2000;
    constexpr float kConvergenceMeanAbsYawDeg = 3.0f;
    constexpr float kConvergenceMaxAbsYawDeg = 6.0f;

    heading_test_result_t result = {
        .name = test_case.name,
        .csv_file = (std::fabs(test_case.estimator_heading_offset_deg) < 1e-6f)
            ? "ahrs_static_c172x_reset00.csv"
            : "ahrs_static_c172x_reset00_" + std::string(test_case.name) + ".csv",
        .converged = false,
        .final_yaw_error_deg = 0.0f,
        .mean_abs_yaw_error_last_window_deg = 0.0f,
        .max_abs_yaw_error_last_window_deg = 0.0f,
    };

    JSBSim::FGFDMExec fdm_exec;
    fdm_exec.SetDebugLevel(0);

    // Use copied resources from the executable working directory.
    fdm_exec.SetRootDir(SGPath("."));
    fdm_exec.SetAircraftPath(SGPath("aircraft"));
    fdm_exec.SetEnginePath(SGPath("engine"));
    fdm_exec.SetSystemsPath(SGPath("systems"));

    if (!fdm_exec.LoadModel("c172x"))
    {
        std::cerr << "Failed to load aircraft model c172x" << std::endl;
        return result;
    }

    auto ic = fdm_exec.GetIC();
    if (ic == nullptr || !ic->Load(SGPath("reset00.xml")))
    {
        std::cerr << "Failed to load initial condition reset00.xml" << std::endl;
        return result;
    }

    fdm_exec.Setdt(kDt);
    fdm_exec.RunIC();

    // Keep the aircraft parked/neutral for the static IMU test.
    fdm_exec.SetPropertyValue("fcs/throttle-cmd-norm", 0.0);
    fdm_exec.SetPropertyValue("fcs/mixture-cmd-norm", 0.0);
    fdm_exec.SetPropertyValue("fcs/aileron-cmd-norm", 0.0);
    fdm_exec.SetPropertyValue("fcs/elevator-cmd-norm", 0.0);
    fdm_exec.SetPropertyValue("fcs/rudder-cmd-norm", 0.0);
    fdm_exec.SetPropertyValue("fcs/flap-cmd-norm", 0.0);
    fdm_exec.SetPropertyValue("fcs/left-brake-cmd-norm", 1.0);
    fdm_exec.SetPropertyValue("fcs/right-brake-cmd-norm", 1.0);

    estimator_t estimator;
    estimator_init(&estimator, 0.002f, 0.0002f);
    estimator.attitude = quat_from_euler_rad(0.0f, 0.0f, test_case.estimator_heading_offset_deg * kDegToRad);

    std::ofstream csv(result.csv_file, std::ios::out | std::ios::trunc);
    if (!csv.is_open())
    {
        std::cerr << "Failed to open output file " << result.csv_file << std::endl;
        return result;
    }

    csv << std::fixed << std::setprecision(8);
    csv << "step,time_s,imu_gyro_x_rps,imu_gyro_y_rps,imu_gyro_z_rps,imu_accel_x_mps2,imu_accel_y_mps2,imu_accel_z_mps2,imu_mag_x_uT,imu_mag_y_uT,imu_mag_z_uT,";
    csv << "true_roll_deg,true_pitch_deg,true_yaw_deg,true_mag_heading_deg,mag_declination_deg,";
    csv << "est_roll_deg,est_pitch_deg,est_yaw_deg,yaw_error_wrapped_deg,est_mag_heading_error_wrapped_deg,qw,qx,qy,qz,bgx_rps,bgy_rps,bgz_rps,";
    csv << "P00,P11,P22,P33,P44,P55\n";

    bool mag_ref_initialized = false;
    float mag_declination_deg = 0.0f;
    vec3_t mag_nav_ref_accum = vec3_zero();
    int mag_nav_ref_samples = 0;

    float yaw_abs_error_sum_window = 0.0f;
    float yaw_abs_error_max_window = 0.0f;
    int yaw_window_samples = 0;

    for (int i = 0; i <= kSteps; i++)
    {
        if (!fdm_exec.Run())
        {
            std::cerr << "JSBSim step failed at iteration " << i << " for case " << test_case.name << std::endl;
            break;
        }

        const float t = static_cast<float>(fdm_exec.GetSimTime());

        const vec3_t gyro_meas_rps = {
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/gyroX_rps")),
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/gyroY_rps")),
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/gyroZ_rps"))
        };

        const vec3_t accel_meas_mps2 = {
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/accelX_mps2")),
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/accelY_mps2")),
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/accelZ_mps2"))
        };

        const vec3_t mag_meas_uT = {
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/magX_uT")),
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/magY_uT")),
            static_cast<float>(fdm_exec.GetPropertyValue("sensor/imu/magZ_uT"))
        };

        const float true_roll_rad = static_cast<float>(fdm_exec.GetPropertyValue("attitude/phi-rad"));
        const float true_pitch_rad = static_cast<float>(fdm_exec.GetPropertyValue("attitude/theta-rad"));
        const float true_yaw_rad = static_cast<float>(fdm_exec.GetPropertyValue("attitude/psi-rad"));

        if (!mag_ref_initialized && i >= kMagRefInitStartSample)
        {
            quat_t true_q_bn = quat_from_euler_rad(true_roll_rad, true_pitch_rad, true_yaw_rad);
            vec3_t mag_nav_ref = quat_rotate_vec3(true_q_bn, mag_meas_uT);
            if (vec3_is_finite(mag_nav_ref))
            {
                mag_nav_ref_accum = vec3_add(mag_nav_ref_accum, mag_nav_ref);
                mag_nav_ref_samples += 1;
            }

            if (mag_nav_ref_samples >= kMagRefInitSamples)
            {
                vec3_t mag_nav_ref_mean = vec3_scale(mag_nav_ref_accum, 1.0f / static_cast<float>(mag_nav_ref_samples));
                estimator_set_mag_reference(&estimator, &mag_nav_ref_mean);
                mag_ref_initialized = estimator.mag_ref_valid;
                if (mag_ref_initialized)
                {
                    mag_declination_deg = wrap_deg_pm_180(std::atan2(estimator.mag_nav_ref.y, estimator.mag_nav_ref.x) * kRadToDeg);
                }
            }
        }

        estimator_predict(&estimator, &gyro_meas_rps, kDt);
        estimator_update_accel(&estimator, &accel_meas_mps2);
        estimator_update_mag(&estimator, &mag_meas_uT);

        const float true_roll_deg = true_roll_rad * kRadToDeg;
        const float true_pitch_deg = true_pitch_rad * kRadToDeg;
        const float true_yaw_deg = wrap_deg_0_360(true_yaw_rad * kRadToDeg);

        float est_roll_deg = 0.0f;
        float est_pitch_deg = 0.0f;
        float est_yaw_deg = 0.0f;
        quat_to_euler_deg(estimator.attitude, est_roll_deg, est_pitch_deg, est_yaw_deg);
        est_yaw_deg = wrap_deg_0_360(est_yaw_deg);

        const float true_mag_heading_deg = wrap_deg_0_360(true_yaw_deg + mag_declination_deg);
        const float est_mag_heading_deg = wrap_deg_0_360(est_yaw_deg + mag_declination_deg);
        const float yaw_error_wrapped_deg = wrap_deg_pm_180(est_yaw_deg - true_yaw_deg);
        const float est_mag_heading_error_wrapped_deg = wrap_deg_pm_180(est_mag_heading_deg - true_mag_heading_deg);

        result.final_yaw_error_deg = yaw_error_wrapped_deg;
        if (i > (kSteps - kConvergenceWindowSamples))
        {
            const float yaw_abs_err = std::fabs(yaw_error_wrapped_deg);
            yaw_abs_error_sum_window += yaw_abs_err;
            yaw_abs_error_max_window = fmaxf(yaw_abs_error_max_window, yaw_abs_err);
            yaw_window_samples += 1;
        }

        csv << i << ',' << t << ','
            << gyro_meas_rps.x << ',' << gyro_meas_rps.y << ',' << gyro_meas_rps.z << ','
            << accel_meas_mps2.x << ',' << accel_meas_mps2.y << ',' << accel_meas_mps2.z << ','
            << mag_meas_uT.x << ',' << mag_meas_uT.y << ',' << mag_meas_uT.z << ','
            << true_roll_deg << ',' << true_pitch_deg << ',' << true_yaw_deg << ',' << true_mag_heading_deg << ',' << mag_declination_deg << ','
            << est_roll_deg << ',' << est_pitch_deg << ',' << est_yaw_deg << ',' << yaw_error_wrapped_deg << ',' << est_mag_heading_error_wrapped_deg << ','
            << estimator.attitude.w << ',' << estimator.attitude.x << ','
            << estimator.attitude.y << ',' << estimator.attitude.z << ','
            << estimator.gyro_bias_rps.x << ',' << estimator.gyro_bias_rps.y << ','
            << estimator.gyro_bias_rps.z << ','
            << estimator.P[0][0] << ',' << estimator.P[1][1] << ','
            << estimator.P[2][2] << ',' << estimator.P[3][3] << ','
            << estimator.P[4][4] << ',' << estimator.P[5][5] << '\n';
    }

    csv.close();

    if (yaw_window_samples > 0)
    {
        result.mean_abs_yaw_error_last_window_deg = yaw_abs_error_sum_window / static_cast<float>(yaw_window_samples);
        result.max_abs_yaw_error_last_window_deg = yaw_abs_error_max_window;
    }

    result.converged = mag_ref_initialized
        && (result.mean_abs_yaw_error_last_window_deg <= kConvergenceMeanAbsYawDeg)
        && (result.max_abs_yaw_error_last_window_deg <= kConvergenceMaxAbsYawDeg);

    return result;
}

int main()
{
    const std::vector<heading_test_case_t> heading_tests = {
        {"heading_0deg", 0.0f},
        {"heading_p15deg", 15.0f},
        {"heading_m15deg", -15.0f},
        {"heading_p45deg", 45.0f},
        {"heading_m45deg", -45.0f},
    };

    std::vector<heading_test_result_t> results;
    results.reserve(heading_tests.size());

    bool all_converged = true;
    for (const heading_test_case_t& test_case : heading_tests)
    {
        std::cout << "Running JSBSim heading convergence case: " << test_case.name
            << " (initial estimator heading offset = " << test_case.estimator_heading_offset_deg << " deg)" << std::endl;

        heading_test_result_t result = run_static_heading_test_case(test_case);
        all_converged = all_converged && result.converged;
        results.push_back(result);
    }

    std::cout << "\nHeading convergence summary:\n";
    for (const heading_test_result_t& result : results)
    {
        std::cout << "  [" << (result.converged ? "PASS" : "FAIL") << "] "
            << result.name
            << " | final_yaw_error_deg=" << result.final_yaw_error_deg
            << " | mean_abs_last2s_deg=" << result.mean_abs_yaw_error_last_window_deg
            << " | max_abs_last2s_deg=" << result.max_abs_yaw_error_last_window_deg
            << " | csv=" << result.csv_file
            << std::endl;
    }

    return all_converged ? 0 : 2;
}