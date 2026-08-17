# Helius Autopilot

## Overview

Helius Autopilot is an embedded flight-control project focused on attitude estimation and, later, full aircraft state estimation and control.

The current firmware runs on an STM32 and implements a quaternion-based **6-state Error-State Kalman Filter (ESKF)** for attitude estimation.

The current error state is:

```text
δx = [δθx δθy δθz bgx bgy bgz]ᵀ
```

Where:

* `δθ` represents the attitude error.
* `bg` represents the gyroscope bias.

The nominal attitude is represented by a unit quaternion and propagated using gyroscope measurements.

Accelerometer measurements are currently used as a gravity reference to correct roll and pitch.

The current development hardware consists of an **STM32 Black Pill** and a **GY-91** sensor module.

The GY-91 driver acts as an abstraction layer over the individual sensors:

* **MPU9250** — accelerometer and gyroscope
* **AK8963** — 3-axis magnetometer
* **BMP280** — barometer

Each sensor has its own independent low-level driver, while the GY-91 layer handles board-specific behavior such as initialization and coordinate-frame alignment.

---

## Specifications

### Hardware

* STM32 Black Pill
* GY-91 sensor module

  * MPU9250 accelerometer
  * MPU9250 gyroscope
  * AK8963 magnetometer
  * BMP280 barometer
* USB CDC for telemetry and debugging
* I²C sensor communication

### Sensor Rates

| Component                   |    Rate |
| --------------------------- | ------: |
| Gyroscope / AHRS prediction | 1000 Hz |
| Accelerometer correction    |  100 Hz |
| Magnetometer                |  100 Hz |
| Logging                     |   20 Hz |

---

## MPU9250

The current MPU9250 driver supports:

* Accelerometer range configuration
* Gyroscope range configuration
* Digital Low-Pass Filter (DLPF) configuration
* Configurable sampling rate
* Acceleration output in `m/s²`
* Angular velocity output in `rad/s`
* Temperature measurement
* Auxiliary I²C master control
* Auxiliary I²C bypass mode

---

## AK8963

The current AK8963 driver supports:

* Device identification using `WHO_AM_I`
* 14-bit and 16-bit measurement resolution
* Continuous measurement modes
* Factory sensitivity adjustment using `ASA`
* Raw measurement conversion to `µT`
* Data-ready detection
* Magnetic overflow detection
* Coordinate transformation to the MPU9250/body frame

### Coordinate Frame Alignment

The AK8963 coordinate system does not directly match the MPU9250 accelerometer/gyroscope coordinate system.

The GY-91 layer converts:

```text
BODY X =  MAG Y
BODY Y =  MAG X
BODY Z = -MAG Z
```

Equivalent transformation matrix:

```text
              [ 0  1  0 ]
Rmag->body =  [ 1  0  0 ]
              [ 0  0 -1 ]
```

All measurements exposed to the estimator should use the same **BODY frame**.

---

## AHRS

The current AHRS implementation uses a quaternion nominal state and a 6-state error model.

### Error State

```text
[ attitude error (3) | gyroscope bias (3) ]
```

or:

```text
δx = [δθx δθy δθz bgx bgy bgz]ᵀ
```

### Current Features

* Quaternion attitude representation
* Gyroscope-based attitude prediction
* Gyroscope bias estimation
* Error covariance propagation
* Accelerometer gravity correction
* Joseph-form covariance update
* Quaternion normalization after prediction and correction

### Current Observability

The accelerometer provides the gravity direction and therefore constrains:

* Roll
* Pitch

Yaw is not observable using only gyroscope and accelerometer measurements.

The AK8963 magnetometer will later provide an absolute heading reference for yaw correction.

---

## Magnetometer Calibration

Before using the magnetometer in the AHRS, the measurements must be calibrated.

The intended calibration model is:

```text
m_cal = S * (m_body - b)
```

Where:

* `m_body` is the magnetometer measurement in the BODY frame.
* `b` is the hard-iron bias vector.
* `S` is the 3×3 soft-iron correction matrix.
* `m_cal` is the calibrated magnetic-field vector.

The complete magnetometer processing pipeline is intended to be:

```text
AK8963 raw
    ↓
Factory ASA correction
    ↓
µT
    ↓
AK8963 → BODY axis alignment
    ↓
Hard-iron correction
    ↓
Soft-iron correction
    ↓
AHRS
```

---

## TODO

### Magnetometer

* [ ] Collect magnetometer measurements over a full 3D range of orientations
* [ ] Implement hard-iron calibration
* [ ] Implement soft-iron calibration
* [ ] Store magnetometer calibration parameters
* [ ] Apply hard-iron and soft-iron correction
* [ ] Validate magnetic-field magnitude after calibration
* [ ] Implement magnetic innovation gating
* [ ] Implement `ahrs_update_mag()`
* [ ] Validate yaw convergence
* [ ] Validate long-term heading stability

Magnetometer calibration is currently postponed because the development setup uses a single Black Pill connected through USB, making unrestricted 3D rotation difficult without disconnecting the USB connection.

### BMP280

* [ ] Implement BMP280 driver
* [ ] Read factory compensation parameters
* [ ] Implement pressure compensation
* [ ] Implement temperature compensation
* [ ] Convert pressure to barometric altitude
* [ ] Integrate BMP280 into the GY-91 abstraction

### AHRS

* [ ] Add accelerometer magnitude gating during significant linear acceleration
* [ ] Add magnetometer correction
* [ ] Validate covariance behavior
* [ ] Validate numerical stability
* [ ] Validate gyroscope bias convergence
* [ ] Add innovation monitoring
* [ ] Add configurable measurement noise
* [ ] Verify quaternion and coordinate-frame conventions
* [ ] Initialize attitude using accelerometer and magnetometer measurements
* [ ] Add automated estimator tests using synthetic sensor measurements

### Drivers

* [ ] Complete GY-91 abstraction
* [ ] Ensure all GY-91 outputs use a common BODY coordinate frame
* [ ] Add detailed sensor status propagation
* [ ] Add sensor health checks
* [ ] Add communication timeout handling
* [ ] Add I²C bus recovery for locked-bus conditions
* [ ] Evaluate SPI for high-rate MPU9250 communication

### Timing and Performance

* [ ] Measure MPU9250 transaction execution time
* [ ] Measure AK8963 transaction execution time
* [ ] Measure AHRS prediction execution time
* [ ] Measure AHRS correction execution time
* [ ] Verify the complete 1 kHz execution budget
* [ ] Detect and count missed loop deadlines
* [ ] Measure CPU usage
* [ ] Measure RAM and Flash usage

### Future Estimator Development

* [ ] Extend the AHRS into a navigation ESKF
* [ ] Add accelerometer bias states
* [ ] Add velocity states
* [ ] Add position states
* [ ] Integrate GNSS measurements
* [ ] Integrate barometric altitude
* [ ] Add multi-rate sensor fusion
* [ ] Evaluate 15-, 18-, or higher-state estimator architectures

### Flight-Control System

* [ ] Implement angular-rate controller
* [ ] Implement attitude controller
* [ ] Implement actuator mixing
* [ ] Implement servo outputs
* [ ] Add flight modes
* [ ] Add failsafe logic
* [ ] Add sensor-health monitoring
* [ ] Add persistent calibration storage
