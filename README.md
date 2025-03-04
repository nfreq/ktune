# ktune
ktune is a servo control tuning utility for matching real2sim. It utilizes Kscales kos and kos-sim.


# ktune - Actuator Sim2Real Tuning Utility

**ktune** is a command line tool for running simple actuator tests (sine, step, and chirp) on both simulation and real robot systems. It is designed to help you tune actuator parameters, collect performance data, and compare responses between simulation and hardware.

## Features

- **Sine Test:** Command an actuator with a sine wave and log both commanded and measured positions/velocities.
- **Step Test:** Perform step changes to evaluate actuator response, including overshoot analysis.
- **Chirp Test:** Execute a chirp waveform to test actuator dynamics over a frequency sweep.
- **Sim2Real Comparison:** Run tests concurrently on a simulator and a real robot, then plot and compare the results.
- **Servo Configuration:** Easily enable or disable additional servos on the real robot via command line options.


## Installation
```
pip install ktune
```


*Note:* Ensure that the `pykos` library is installed and correctly configured for your setup.

## Usage

```
ktune --help

options:
  -h, --help            show this help message and exit
  --name NAME           Name For Plot titles
  --sim_ip SIM_IP       Simulator KOS IP address (default=localhost)
  --ip IP               Real robot KOS IP address (default=192.168.42.1)
  --actuator-id ACTUATOR_ID
                        Actuator ID to test.
  --test {step,sine,chirp}
                        Type of test to run.
  --chirp-amp CHIRP_AMP
                        Chirp amplitude (degrees)
  --chirp-init-freq CHIRP_INIT_FREQ
                        Chirp initial frequency (Hz)
  --chirp-sweep-rate CHIRP_SWEEP_RATE
                        Chirp sweep rate (Hz per second)
  --chirp-duration CHIRP_DURATION
                        Chirp test duration (seconds)
  --freq FREQ           Sine frequency (Hz)
  --amp AMP             Sine amplitude (degrees)
  --duration DURATION   Sine test duration (seconds)
  --step-size STEP_SIZE
                        Step size (degrees)
  --step-hold-time STEP_HOLD_TIME
                        Time to hold at step (seconds)
  --step-count STEP_COUNT
                        Number of steps to take
  --sim-kp SIM_KP       Proportional gain
  --sim-kv SIM_KV       Damping gain
  --kp KP               Proportional gain
  --kd KD               Derivative gain
  --ki KI               Integral gain
  --acceleration ACCELERATION
                        Acceleration (deg/s^2)
  --max-torque MAX_TORQUE
                        Max torque
  --torque-off          Disable torque for test?
  --no-log              Do not record/plot data
  --log-duration-pad LOG_DURATION_PAD
                        Pad (seconds) after motion ends to keep logging
  --sample-rate SAMPLE_RATE
                        Data collection rate (Hz)
  --enable-servos ENABLE_SERVOS
                        Comma delimited list of servo IDs to enable on the real robot (e.g., 11,12,13)
  --disable-servos DISABLE_SERVOS
                        Comma delimited list of servo IDs to disable on the real robot (e.g., 31,32,33)
```
### Running a Step Test

Perform a step test with a step size of 10° and a hold time of 3 seconds per step, running for 2 cycles:
```
ktune  --actuator-id 11 --test step --step-size 10.0 --step-hold-time 3.0 --step-count 2
```
<img width="1397" alt="ktune_step_test" src="https://github.com/user-attachments/assets/f5548bb9-3029-4002-980a-d7752e4484aa" />


### Running a Sine Test

Run a sine wave test on actuator 11 with a frequency of 1.0 Hz, amplitude of 5.0°, and duration of 5 seconds:
```
ktune --actuator-id 11 --test sine --freq 1.0 --amp 5.0 --duration 5.0
```
<img width="1399" alt="ktune_sine_test" src="https://github.com/user-attachments/assets/b63acd63-d7d2-41ec-9d90-5a046b6b4516" />


### Running a Chirp Test

Execute a chirp test with an amplitude of 5.0°, initial frequency of 1.0 Hz, sweep rate of 0.5 Hz/s, and duration of 5 seconds:
```
ktune --actuator-id 11 --test chirp --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 --chirp-duration 5.0
```
<img width="1398" alt="ktune_chirp_test" src="https://github.com/user-attachments/assets/30f22eb3-0f9d-4eb7-be67-e2e5d23a9bb5" />


### Configuring Additional Servos

Enable or disable additional servos on the real robot:
```bash
# Enable servos with IDs 11, 12, and 13:
ktune --enable-servos 11,12,13

# Disable servos with IDs 31, 32, and 33:
ktune --disable-servos 31,32,33
```

## Command Line Options

Below is a summary of the key command line arguments:

- **General Settings:**
  - `--sim_ip`: Simulator KOS-SIM IP address (default: `127.0.0.1`)
  - `--ip`: Real robot KOS IP address (default: `192.168.42.1`)
  - `--actuator-id`: Actuator ID to test (default: `11`)
  - `--test`: Test type to run (`sine`, `step`, `chirp`)

- **Sine Test Parameters:**
  - `--freq`: Sine wave frequency (Hz)
  - `--amp`: Sine wave amplitude (degrees)
  - `--duration`: Test duration (seconds)

- **Step Test Parameters:**
  - `--step-size`: Step size (degrees)
  - `--step-hold-time`: Time to hold at each step (seconds)
  - `--step-count`: Number of step cycles

- **Chirp Test Parameters:**
  - `--chirp-amp`: Chirp amplitude (degrees)
  - `--chirp-init-freq`: Chirp initial frequency (Hz)
  - `--chirp-sweep-rate`: Chirp sweep rate (Hz/s)
  - `--chirp-duration`: Chirp test duration (seconds)

- **Actuator Configuration:**
  - `--kp`, `--kd`, `--ki`: Gains for real actuator control
  - `--sim-kp`, `--sim-kv`: Gains for simulation
  - `--acceleration`: Actuator acceleration (deg/s²)
  - `--max-torque`: Maximum torque limit
  - `--torque-off`: Disable actuator torque if specified

- **Data Logging and Plotting:**
  - `--no-log`: Disable data logging and plotting
  - `--log-duration-pad`: Additional logging duration after motion ends (seconds)
  - `--sample-rate`: Data collection rate (Hz)

- **Servo Enable/Disable:**
  - `--enable-servos`: Comma-separated list of servo IDs to enable on the real robot
  - `--disable-servos`: Comma-separated list of servo IDs to disable on the real robot

## Data Logging and Plotting

ktune logs both command and response data for the actuators and generates comparison plots between simulation and real robot performance. Plots are saved to the `plots/` directory with a timestamp in the filename.

## License

This project is licensed under the [MIT License](LICENSE).

## Contributing

Contributions, issues, and feature requests are welcome! Feel free to open an issue or submit a pull request.
