# ktune - Sim2Real Toolkit

**ktune** is a command-line utility for actuator tuning and simulation-to-reality (Sim2Real) validation. It enables running standardized motion tests (sine, step, chirp) simultaneously in simulation and hardware, providing quantitative comparison for actuator dynamics and control performance.


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

### Running Tests

ktune supports three modes of operation:
- `real`: Run tests on real hardware only
- `sim`: Run tests in simulation only
- `compare`: Run tests simultaneously on both real hardware and simulation

Each mode supports three types of tests:
- Sine wave tests
- Step response tests
- Chirp (frequency sweep) tests

### Connection Parameters

ktune works with both real hardware (via KOS) and simulation (via KOS-SIM):

- **Real Hardware (KOS)**:
  - Default IP: `192.168.42.1`
  - Use `--real-ip` to specify a different address
  - Typically used when connected to physical robot's network

- **Simulation (KOS-SIM)**:
  - Default IP: `127.0.0.1` (localhost)
  - Use `--sim-ip` to specify a different address
  - Requires running kos-sim instance

#### Real Hardware Examples

```bash
# Default connection (192.168.42.1)
ktune real sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0
ktune real step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2
ktune real chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 --chirp-duration 5.0

# Custom IP address
ktune real sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0 --real-ip 192.168.1.100
ktune real step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2 --real-ip 192.168.1.100
ktune real chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 --real-ip 192.168.1.100

# With control parameters
ktune real sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0 \
    --kp 20.0 --kd 5.0 --ki 0.0 --max-torque 50.0

ktune real step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2 \
    --kp 20.0 --kd 5.0 --ki 0.0 --max-torque 50.0

ktune real chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 \
    --kp 20.0 --kd 5.0 --ki 0.0 --max-torque 50.0
```

#### Simulation Examples

```bash
# Default connection (localhost)
ktune sim sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0
ktune sim step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2
ktune sim chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5

# Custom simulator IP
ktune sim sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0 --sim-ip 192.168.1.50
ktune sim step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2 --sim-ip 192.168.1.50
ktune sim chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 --sim-ip 192.168.1.50

# With simulation-specific parameters
ktune sim sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0 \
    --sim-kp 20.0 --sim-kd 5.0 --stream-delay 0.0

ktune sim step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2 \
    --sim-kp 20.0 --sim-kd 5.0 --stream-delay 0.0

ktune sim chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 \
    --sim-kp 20.0 --sim-kd 5.0 --stream-delay 0.0
```

#### Comparison Examples

```bash
# Default connections
ktune compare sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0
ktune compare step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2
ktune compare chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5

# Custom IPs for both real and simulation
ktune compare sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0 \
    --real-ip 192.168.1.100 --sim-ip 192.168.1.50

ktune compare step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2 \
    --real-ip 192.168.1.100 --sim-ip 192.168.1.50

ktune compare chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 \
    --real-ip 192.168.1.100 --sim-ip 192.168.1.50

# Full configuration example
ktune compare sine --actuator-id 11 --freq 1.0 --amp 5.0 --duration 5.0 \
    --real-ip 192.168.1.100 --sim-ip 192.168.1.50 \
    --kp 20.0 --kd 5.0 --ki 0.0 \
    --sim-kp 20.0 --sim-kd 5.0 --stream-delay 0.0 \
    --max-torque 50.0 --sample-rate 100.0

ktune compare step --actuator-id 11 --step-size 10.0 --step-hold-time 3.0 --step-count 2 \
    --real-ip 192.168.1.100 --sim-ip 192.168.1.50 \
    --kp 20.0 --kd 5.0 --ki 0.0 \
    --sim-kp 20.0 --sim-kd 5.0 --stream-delay 0.0 \
    --max-torque 50.0 --sample-rate 100.0

ktune compare chirp --actuator-id 11 --chirp-amp 5.0 --chirp-init-freq 1.0 --chirp-sweep-rate 0.5 \
    --real-ip 192.168.1.100 --sim-ip 192.168.1.50 \
    --kp 20.0 --kd 5.0 --ki 0.0 \
    --sim-kp 20.0 --sim-kd 5.0 --stream-delay 0.0 \
    --max-torque 50.0 --sample-rate 100.0
```

### Common Options

All test commands support these common options:
- `--actuator-id`: ID of the actuator to test (default: 11)
- `--start-pos`: Starting position in degrees (default: 0.0)
- `--kp`: Proportional gain (default: 20.0)
- `--kd`: Derivative gain (default: 5.0)
- `--ki`: Integral gain (default: 0.0)
- `--max-torque`: Maximum torque limit (default: 100.0)
- `--acceleration`: Acceleration limit in deg/s² (default: 0.0)
- `--sample-rate`: Data collection rate in Hz (default: 100.0)


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

