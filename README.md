# ktune
ktune is a servo control tuning utility for matching real2sim. It utilizes Kscales kos and kos-sim.


```markdown
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

The `ktune.py` script can run in different test modes. Use the `--help` flag for full details on available options.

### Running a Sine Test

Run a sine wave test on actuator 11 with a frequency of 1.0 Hz, amplitude of 5.0°, and duration of 5 seconds:
```bash
./ktune.py --actuator-id 11 --test sine --freq 1.0 --amp 5.0 --duration 5.0
```

### Running a Step Test

Perform a step test with a step size of 10° and a hold time of 3 seconds per step, running for 2 cycles:
```bash
./ktune.py --actuator-id 11 --test step --step-size 10.0 --step-hold-time 3.0 --step-count 2
```

### Running a Chirp Test

Execute a chirp test with an amplitude of 5.0°, initial frequency of 1.0 Hz, sweep rate of 0.5 Hz/s, and duration of 5 seconds:
```bash
./ktune.py --actuator-id 11 --test chirp --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 --chirp-duration 5.0
```

### Configuring Additional Servos

Enable or disable additional servos on the real robot:
```bash
# Enable servos with IDs 11, 12, and 13:
./ktune.py --enable-servos 11,12,13

# Disable servos with IDs 31, 32, and 33:
./ktune.py --disable-servos 31,32,33
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

By default, ktune logs both command and response data for the actuators and generates comparison plots between simulation and real robot performance. Plots are saved to the `plots/` directory with a timestamp in the filename. Use the `--no-log` flag to disable data logging and plotting if not needed.

## License

This project is licensed under the [MIT License](LICENSE).

## Contributing

Contributions, issues, and feature requests are welcome! Feel free to open an issue or submit a pull request.