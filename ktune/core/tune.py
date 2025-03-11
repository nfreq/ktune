# ktune/core/tune.py
import asyncio
import math
import time
import json
import os
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional
from pykos import KOS
from ktune.core.utils.datalog import DataLog
from ktune.core.utils.plots import Plot
from ktune.core.utils import metrics
# Configure logging
logging.getLogger('matplotlib').setLevel(logging.WARNING)
os.environ["PYTHONWARNINGS"] = "ignore"
logging.getLogger().setLevel(logging.ERROR)

@dataclass
class TuneConfig:
    """Configuration for tuning tests"""
    # Connection settings
    name: str = "Zeroth01"
    sim_ip: str = "127.0.0.1"
    real_ip: str = "192.168.42.1"
    actuator_id: int = 11
    start_pos: float = 0.0

    # Actuator gains
    kp: float = 20.0
    kd: float = 55.0
    ki: float = 0.01
    
    # Actuator config
    acceleration: float = 0.0
    max_torque: float = 100.0
    torque_off: bool = False

    # Simulation gains
    sim_kp: float = 24.0
    sim_kv: float = 0.75

    # Logging config
    no_log: bool = False
    log_duration_pad: float = 2.0
    sample_rate: float = 100.0

    # Servo control
    enable_servos: Optional[List[int]] = None
    disable_servos: Optional[List[int]] = None

    # Test parameters (will be set by specific test commands)
    test: Optional[str] = None
    # Sine parameters
    freq: Optional[float] = None
    amp: Optional[float] = None
    duration: Optional[float] = None
    # Step parameters
    step_size: Optional[float] = None
    step_hold_time: Optional[float] = None
    step_count: Optional[int] = None
    # Chirp parameters
    chirp_amp: Optional[float] = None
    chirp_init_freq: Optional[float] = None
    chirp_sweep_rate: Optional[float] = None
    chirp_duration: Optional[float] = None

class Tune:
    def __init__(self, config: Dict):
        tune_config = config.get('tune', {})
        self.config = TuneConfig(**tune_config)
        
        self.sim_data = {
            "time": [], "position": [], "velocity": [],
            "cmd_time": [], "cmd_pos": [], "cmd_vel": []
        }
        self.real_data = {
            "time": [], "position": [], "velocity": [],
            "cmd_time": [], "cmd_pos": [], "cmd_vel": []
        }

    async def setup_connections(self):
        """Initialize and test connections to real and simulated systems"""
        print("Testing KOS-SIM connection performance...")
        self.sim_kos = KOS(self.config.sim_ip)
        sim_start = time.time()
        for _ in range(100):  # Test 100 samples
            await self.sim_kos.actuator.get_actuators_state([self.config.actuator_id])
        sim_end = time.time()
        sim_rate = 100 / (sim_end - sim_start)

        print("Testing KOS-REAL connection performance...")
        self.real_kos = KOS(self.config.real_ip)
        real_start = time.time()
        for _ in range(100):  # Test 100 samples
            await self.real_kos.actuator.get_actuators_state([self.config.actuator_id])
        real_end = time.time()
        real_rate = 100 / (real_end - real_start)

        print(f"Max KOS-SIM sampling rate: {sim_rate:.1f} Hz")
        print(f"Max KOS-REAL sampling rate: {real_rate:.1f} Hz")
        print(f"Required sampling rate: {self.config.sample_rate} Hz")

        if sim_rate < self.config.sample_rate or real_rate < self.config.sample_rate:
            raise ValueError(
                f"Requested sampling rate ({self.config.sample_rate} Hz) exceeds "
                "maximum achievable rates. Try re-running kos-sim --no-render or "
                "reduce the sampling rate and try again"
            )
        
    def _print_test_config(self):
        """Print test configuration and motor settings."""
        print("\nTest Configuration:")
        print(f"Test Type: {self.config.test}")
        print(f"Actuator ID: {self.config.actuator_id}")
        print(f"Start Position: {self.config.start_pos}°")
        print(f"Sample Rate: {self.config.sample_rate} Hz")
        
        print("\nMotor Settings:")
        print("Real System:")
        print(f"  Kp: {self.config.kp}")
        print(f"  Kd: {self.config.kd}")
        print(f"  Ki: {self.config.ki}")
        print(f"  Max Torque: {self.config.max_torque}")
        print(f"  Acceleration: {self.config.acceleration}")
        print("Simulation:")
        print(f"  Kp: {self.config.sim_kp}")
        print(f"  Kv: {self.config.sim_kv}")
        
        # Print test-specific parameters
        if self.config.test == "step":
            print("\nStep Test Parameters:")
            print(f"Step Size: {self.config.step_size}°")
            print(f"Hold Time: {self.config.step_hold_time}s")
            print(f"Step Count: {self.config.step_count}")
            total_duration = (self.config.step_hold_time * (2 * self.config.step_count + 1) + 
                            self.config.log_duration_pad)
            print(f"Total Duration: {total_duration}s")
        
        elif self.config.test == "sine":
            print("\nSine Test Parameters:")
            print(f"Frequency: {self.config.freq} Hz")
            print(f"Amplitude: {self.config.amp}°")
            print(f"Duration: {self.config.duration}s")
            print(f"Total Duration: {self.config.duration + self.config.log_duration_pad}s")
        
        elif self.config.test == "chirp":
            print("\nChirp Test Parameters:")
            print(f"Initial Frequency: {self.config.chirp_init_freq} Hz")
            print(f"Sweep Rate: {self.config.chirp_sweep_rate} Hz/s")
            print(f"Amplitude: {self.config.chirp_amp}°")
            print(f"Duration: {self.config.chirp_duration}s")
            print(f"Total Duration: {self.config.chirp_duration + self.config.log_duration_pad}s")

    def run_test(self, test_type: Optional[str] = None):
        """Main entry point for running tests"""
        if test_type is None:
            test_type = self.config.test
        if test_type is None:
            raise ValueError("No test type specified")
        
        asyncio.run(self._run_test(test_type))
        self.save_and_plot_results()

    async def _run_test(self, test_type: str):
        """Async implementation of test execution"""
        await self.setup_connections()
        
        # Configure servos if needed
        if self.config.enable_servos:
            await self._enable_servos(self.config.enable_servos)
        if self.config.disable_servos:
            await self._disable_servos(self.config.disable_servos)

        # Run the specified test
        if test_type == "sine":
            await self._run_sine_test()
        elif test_type == "step":
            await self._run_step_test()
        elif test_type == "chirp":
            await self._run_chirp_test()
        else:
            raise ValueError(f"Unknown test type: {test_type}")

        # Clean up connections
        await self.sim_kos.close()
        await self.real_kos.close()

    async def _enable_servos(self, servo_ids: List[int]):
        """Enable specified servos"""
        for kos in [self.sim_kos, self.real_kos]:
            await kos.actuator.enable_actuators(servo_ids)
        print(f"Enabled servos: {servo_ids}")

    async def _disable_servos(self, servo_ids: List[int]):
        """Disable specified servos"""
        for kos in [self.sim_kos, self.real_kos]:
            await kos.actuator.disable_actuators(servo_ids)
        print(f"Disabled servos: {servo_ids}")

    def _log_actuator_state(self, response, data_dict, current_time):
        """Log actuator state data with normalized time.
        
        Args:
            response: Actuator state response
            data_dict: Dictionary to store data
            current_time: Current normalized time (seconds from start)
        """
        if response.states:
            state = response.states[0]
            if state.position is not None:
                data_dict["position"].append(state.position)
            if state.velocity is not None:
                data_dict["velocity"].append(state.velocity)
            data_dict["time"].append(current_time)

    

    async def _run_step_test(self):
        """Run step response test on both sim and real systems"""
        self._print_test_config()
        # Construct step sequence
        vel = 0.0  # Default velocity limit
        steps = [(0.0, vel, self.config.step_hold_time)]
        for _ in range(self.config.step_count):
            steps.append((self.config.step_size, vel, self.config.step_hold_time))
            steps.append((0.0, vel, self.config.step_hold_time))

        # Calculate total duration including padding
        total_duration = (sum(step[2] for step in steps) + 
                        self.config.log_duration_pad)

        # Configure actuators
        for kos, is_real in [(self.sim_kos, False), (self.real_kos, True)]:
            # Select gains based on system type
            if is_real:
                kp, kd, ki = (self.config.kp, self.config.kd, self.config.ki)
            else:
                kp, kd, ki = (self.config.sim_kp, self.config.sim_kv, 0.0)

            await kos.actuator.configure_actuator(
                actuator_id=self.config.actuator_id,
                kp=kp, kd=kd, ki=ki,
                acceleration=self.config.acceleration,
                max_torque=self.config.max_torque,
                torque_enabled=not self.config.torque_off
            )

        # Move to start position and wait
        for kos in [self.sim_kos, self.real_kos]:
            await kos.actuator.command_actuators([{
                'actuator_id': self.config.actuator_id,
                'position': self.config.start_pos,
                'velocity': 0.0
            }])
        await asyncio.sleep(2)

        # Start test
        start_time = time.time()
        current_time = 0.0
        step_idx = 0

        while current_time < total_duration:
            current_time = time.time() - start_time

            # Determine current step target
            while (step_idx < len(steps) and 
                   current_time > sum(step[2] for step in steps[:step_idx + 1])):
                step_idx += 1

            if step_idx < len(steps):
                target_pos = steps[step_idx][0] + self.config.start_pos

                # Command both systems
                for kos, data_dict in [(self.sim_kos, self.sim_data),
                                     (self.real_kos, self.real_data)]:
                    # Send command
                    await kos.actuator.command_actuators([{
                        'actuator_id': self.config.actuator_id,
                        'position': target_pos,
                        'velocity': 0.0
                    }])
                    
                    # Log command
                    data_dict["cmd_time"].append(current_time)
                    data_dict["cmd_pos"].append(target_pos)
                    data_dict["cmd_vel"].append(0.0)  # No velocity command for steps

                    # Get and log state
                    response = await kos.actuator.get_actuators_state(
                        [self.config.actuator_id]
                    )
                    self._log_actuator_state(response, data_dict, current_time)

            await asyncio.sleep(1.0 / self.config.sample_rate)


    async def _run_sine_test(self):
        """Run sine wave test on both sim and real systems"""
        self._print_test_config()

        # Calculate total duration including padding
        total_duration = self.config.duration + self.config.log_duration_pad

        # Configure actuators
        for kos, is_real in [(self.sim_kos, False), (self.real_kos, True)]:
            # Select gains based on system type
            if is_real:
                kp, kd, ki = (self.config.kp, self.config.kd, self.config.ki)
            else:
                kp, kd, ki = (self.config.sim_kp, self.config.sim_kv, 0.0)

            await kos.actuator.configure_actuator(
                actuator_id=self.config.actuator_id,
                kp=kp, kd=kd, ki=ki,
                acceleration=self.config.acceleration,
                max_torque=self.config.max_torque,
                torque_enabled=not self.config.torque_off
            )

        # Move to start position and wait
        for kos in [self.sim_kos, self.real_kos]:
            await kos.actuator.command_actuators([{
                'actuator_id': self.config.actuator_id,
                'position': self.config.start_pos
            }])
        await asyncio.sleep(2)

        # Start test
        start_time = time.time()
        current_time = 0.0

        while current_time < total_duration:
            current_time = time.time() - start_time

            if current_time <= self.config.duration:
                # Calculate sine wave position and velocity
                omega = 2.0 * math.pi * self.config.freq
                phase = omega * current_time
                
                target_pos = (self.config.amp * math.sin(phase) + 
                            self.config.start_pos)
                target_vel = self.config.amp * omega * math.cos(phase)

                # Command both systems
                for kos, data_dict in [(self.sim_kos, self.sim_data),
                                     (self.real_kos, self.real_data)]:
                    # Send command with both position and velocity
                    await kos.actuator.command_actuators([{
                        'actuator_id': self.config.actuator_id,
                        'position': target_pos,
                        'velocity': target_vel
                    }])
                    
                    # Log command
                    data_dict["cmd_time"].append(current_time)
                    data_dict["cmd_pos"].append(target_pos)
                    data_dict["cmd_vel"].append(target_vel)

                    # Get and log state
                    response = await kos.actuator.get_actuators_state(
                        [self.config.actuator_id]
                    )
                    self._log_actuator_state(response, data_dict, current_time)

            await asyncio.sleep(1.0 / self.config.sample_rate)

        # Calculate tracking metrics
        def compute_tracking_error(cmd_time, cmd_pos, actual_time, actual_pos):
            """Compute RMS tracking error"""
            # Interpolate commanded positions to actual timestamps
            from scipy.interpolate import interp1d
            cmd_interp = interp1d(cmd_time, cmd_pos, bounds_error=False)
            cmd_at_actual = cmd_interp(actual_time)
            
            # Compute RMS error where we have both commanded and actual
            valid_idx = ~np.isnan(cmd_at_actual)
            if not np.any(valid_idx):
                return float('nan')
            
            errors = cmd_at_actual[valid_idx] - np.array(actual_pos)[valid_idx]
            rms_error = np.sqrt(np.mean(np.square(errors)))
            return rms_error

        sim_error = compute_tracking_error(
            self.sim_data["cmd_time"],
            self.sim_data["cmd_pos"],
            self.sim_data["time"],
            self.sim_data["position"]
        )
        real_error = compute_tracking_error(
            self.real_data["cmd_time"],
            self.real_data["cmd_pos"],
            self.real_data["time"],
            self.real_data["position"]
        )

        # Print results
        print("\nTest Results:")
        print(f"Sim RMS Error: {sim_error:.3f}°")
        print(f"Real RMS Error: {real_error:.3f}°")

    async def _run_chirp_test(self):
        """Run chirp test on both sim and real systems"""
        self._print_test_config()
        # Calculate total duration including padding
        total_duration = self.config.chirp_duration + self.config.log_duration_pad

        # Configure actuators
        for kos, is_real in [(self.sim_kos, False), (self.real_kos, True)]:
            # Select gains based on system type
            if is_real:
                kp, kd, ki = (self.config.kp, self.config.kd, self.config.ki)
            else:
                kp, kd, ki = (self.config.sim_kp, self.config.sim_kv, 0.0)

            await kos.actuator.configure_actuator(
                actuator_id=self.config.actuator_id,
                kp=kp, kd=kd, ki=ki,
                acceleration=self.config.acceleration,
                max_torque=self.config.max_torque,
                torque_enabled=not self.config.torque_off
            )

        # Move to start position and wait
        for kos in [self.sim_kos, self.real_kos]:
            await kos.actuator.command_actuators([{
                'actuator_id': self.config.actuator_id,
                'position': self.config.start_pos
            }])
        await asyncio.sleep(2)

        # Start test
        start_time = time.time()
        current_time = 0.0

        while current_time < total_duration:
            current_time = time.time() - start_time

            if current_time <= self.config.chirp_duration:
                # Calculate chirp signal
                # Phase integral for linearly increasing frequency:
                # phi(t) = 2π * (f0*t + k*t^2/2), where k is sweep rate
                f0 = self.config.chirp_init_freq
                k = self.config.chirp_sweep_rate
                phase = 2.0 * math.pi * (f0 * current_time + 
                                       0.5 * k * current_time * current_time)
                
                # Instantaneous frequency and angular velocity
                freq = f0 + k * current_time
                omega = 2.0 * math.pi * freq
                
                # Calculate position and velocity
                target_pos = (self.config.chirp_amp * math.sin(phase) + 
                            self.config.start_pos)
                target_vel = self.config.chirp_amp * omega * math.cos(phase)

                # Command both systems
                for kos, data_dict in [(self.sim_kos, self.sim_data),
                                     (self.real_kos, self.real_data)]:
                    # Send command with both position and velocity
                    await kos.actuator.command_actuators([{
                        'actuator_id': self.config.actuator_id,
                        'position': target_pos,
                        'velocity': target_vel
                    }])
                    
                    # Log command
                    data_dict["cmd_time"].append(current_time)
                    data_dict["cmd_pos"].append(target_pos)
                    data_dict["cmd_vel"].append(target_vel)

                    # Get and log state
                    response = await kos.actuator.get_actuators_state(
                        [self.config.actuator_id]
                    )
                    self._log_actuator_state(response, data_dict, current_time)

            await asyncio.sleep(1.0 / self.config.sample_rate)
        
        # Compute frequency response
        sim_freq_response, real_freq_response = metrics.analyze_frequency_response(
            self.sim_data, 
            self.real_data
        )

         # Store frequency response data
        self.sim_data["freq_response"] = sim_freq_response
        self.real_data["freq_response"] = real_freq_response

        print("\nFrequency Response Data:")
        if 'freq_response' in self.sim_data and self.sim_data['freq_response']:
            print(f"Sim frequencies: {len(self.sim_data['freq_response'].get('freq', []))}")
            sim_mag = self.sim_data['freq_response'].get('magnitude', [])
            if sim_mag:
                print(f"Sim magnitude range: {min(sim_mag)} to {max(sim_mag)}")
            else:
                print("No sim magnitude data")
        else:
            print("No sim frequency response data")

        if 'freq_response' in self.real_data and self.real_data['freq_response']:
            print(f"Real frequencies: {len(self.real_data['freq_response'].get('freq', []))}")
            real_mag = self.real_data['freq_response'].get('magnitude', [])
            if real_mag:
                print(f"Real magnitude range: {min(real_mag)} to {max(real_mag)}")
            else:
                print("No real magnitude data")
        else:
            print("No real frequency response data")
            

        # Compute bandwidths
        sim_bandwidth = metrics.compute_bandwidth(
            sim_freq_response["freq"], 
            sim_freq_response["magnitude"]
        )
        real_bandwidth = metrics.compute_bandwidth(
            real_freq_response["freq"], 
            real_freq_response["magnitude"]
        )

        print("\nTest Results:")
        if sim_bandwidth:
            print(f"Sim Bandwidth (-3dB): {sim_bandwidth:.1f} Hz")
        if real_bandwidth:
            print(f"Real Bandwidth (-3dB): {real_bandwidth:.1f} Hz")


    def save_and_plot_results(self):
        """Save data to files and generate plots"""
        if self.config.no_log:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_dir = os.path.join(os.getcwd(), "data")
        plot_dir = os.path.join(os.getcwd(), "plots")
        os.makedirs(data_dir, exist_ok=True)
        os.makedirs(plot_dir, exist_ok=True)

        # Save data
        logger = DataLog(self.config, self.sim_data, self.real_data)
        logger.save_data(timestamp, data_dir)

        # Create plots
        plotter = Plot(self.config, self.sim_data, self.real_data)
        plotter.create_plots(timestamp, plot_dir)

