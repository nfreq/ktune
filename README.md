# ktune - Actuator Tuning and System Identification Toolkit

**ktune** is a powerful command-line tool designed to streamline actuator parameter tuning, data collection, and system identification for robotic systems. It facilitates running structured tests (sine, step, chirp) simultaneously in simulation and real hardware environments, enabling detailed performance analysis and simulation-to-reality comparisons.

## Features

- **Tuning Tests**:
  - **Sine Test**: Commands an actuator with a sine waveform.
  - **Step Test**: Performs step-response analysis.
  - **Chirp Test**: Executes frequency-swept chirp tests.
- **System Identification (SysID)**:
  - Conduct detailed system identification experiments, including pendulum-based tests, to characterize motor and actuator dynamics.
- **Flexible Configuration**:
  - YAML-based configurations for complex experiments.
  - Advanced CLI using the `click` library for easy parameter setting and overrides.
- **Integrated Data Logging**:
  - Automated data logging and plotting for quick visual assessment and comparison between simulations and real-world tests.
- **Servo Management**:
  - Easily enable or disable specific servos directly via CLI commands.

## Installation
```bash
pip install ktune
```
Ensure the `pykos` library is also installed and configured for your hardware.

## Usage

### General CLI Usage
```bash
ktune --help
```

### Running Tuning Tests

**Step Test Example:**
```bash
ktune tune step --actuator-id 11 --size 10.0 --hold-time 3.0 --count 2
```

**Sine Test Example:**
```bash
ktune tune sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0
```

**Chirp Test Example:**
```bash
ktune tune chirp --actuator-id 11 --amp 5.0 --init-freq 1.0 --sweep-rate 0.5 --duration 5.0
```

### Servo Configuration

Enable servos 11, 12, 13:
```bash
ktune tune --enable-servos 11,12,13
```

Disable servos 31, 32, 33:
```bash
ktune tune --disable-servos 31,32,33
```

## System Identification (SysID)

Perform SysID experiments with detailed pendulum setups defined through YAML configurations or CLI parameters.

### Pendulum SysID Example

Run a pendulum system identification test with specific parameters:
```bash
ktune sysid pendulum --mass 0.535 --length 0.150 --trajectory lift_and_drop --kp 32
```

### YAML-based SysID Configuration

Define complex system identification experiments in YAML files:

```yaml
sysid:
  actuator_id: 11
  motor_name: "sts3215v12"
  ip: "192.168.42.1"
  mass: 0.535
  arm_mass: 0.036
  length: 0.150
  error_gain: 0.16489
  winding_resistance: 1.0
  torque_constant: 1.0787
  repetitions: 4
  kp_values: [4, 8, 12, 16, 24, 32]
  trajectories: 
    - "lift_and_drop"
    - "sin_sin"
    - "up_and_down"
    - "sin_time_square"
    - "brutal"
```

Run the experiment based on the YAML configuration:
```bash
ktune sysid pendulum --config path/to/pendulum_config.yaml
```

## Command Line Reference

- **General Settings**:
  - `--sim-ip`, `--real-ip`, `--actuator-id`, `--start-pos`

- **Tuning Tests**:
  - Sine: `--freq`, `--amp`, `--duration`
  - Step: `--size`, `--hold-time`, `--count`
  - Chirp: `--amp`, `--init-freq`, `--sweep-rate`, `--duration`

- **Actuator Configuration**:
  - Gains: `--kp`, `--kd`, `--ki`
  - Torque and Acceleration Limits: `--max-torque`, `--acceleration`, `--torque-off`

- **Simulation Configuration**:
  - `--sim-kp`, `--sim-kv`

- **Data Logging Options**:
  - `--no-log`, `--log-duration-pad`, `--sample-rate`

- **Servo Management**:
  - `--enable-servos`, `--disable-servos`

## Data Logging
Data and plots are saved automatically to the `logs/` and `plots/` directories, respectively, with timestamps for easy tracking.

## Acknowledgements
Special thanks to [Rhoban](https://github.com/Rhoban/bam) and their [Better Actuator Model paper](https://arxiv.org/pdf/2410.08650v1) for valuable insights and contributions to actuator modeling and tuning methodologies.

## License
[MIT License](LICENSE)

## Contributing
Your contributions, feature requests, and bug reports are welcome! Please open issues or submit pull requests.

